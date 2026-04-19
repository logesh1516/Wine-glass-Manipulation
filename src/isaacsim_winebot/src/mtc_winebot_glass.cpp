#include <Eigen/Eigen>
#include <boost/variant/get.hpp>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/task.h>
#include <moveit_msgs/msg/detail/collision_object__struct.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/detail/mesh__struct.hpp>
#include <shape_msgs/msg/mesh.h>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace moveit::task_constructor;

int main(int argc, char *argv[]) {
  // ----------(ROS2 NODE & EXECUTOR)----------------------
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "MTC_node",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
          true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor] { executor.spin(); });

  // ----------(COLLISION OBJECT - WINE GLASS)----------------------
  moveit_msgs::msg::CollisionObject wine_glass;
  wine_glass.header.frame_id = "world";
  wine_glass.id = "wine_glass";

  shapes::Mesh *mesh = shapes::createMeshFromResource(
      "package://isaacsim_winebot/meshes/glass.dae");

  shape_msgs::msg::Mesh mesh_msgs;
  shapes::ShapeMsg mesh_msg_tmp;
  shapes::constructMsgFromShape(mesh, mesh_msg_tmp);
  mesh_msgs = boost::get<shape_msgs::msg::Mesh>(mesh_msg_tmp);

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.45;
  pose.position.y = 0.15;
  pose.position.z = 0.0;
  pose.orientation.w = 1.0;

  wine_glass.meshes.push_back(mesh_msgs);
  wine_glass.mesh_poses.push_back(pose);
  wine_glass.operation = wine_glass.ADD;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(wine_glass);

  // ----------(TASK)----------------------
  Task t;
  t.setName("wine_glass_manipulation");
  t.loadRobotModel(node);

  // ----------(TASK PROPERTIES)----------------------
  t.setProperty("group", "panda_arm");
  t.setProperty("eef", "hand");
  t.setProperty("hand", "hand");
  t.setProperty("ik_frame", "panda_link8");

  // ----------(PLANNERS)----------------------
  auto pipeline_planner = std::make_shared<solvers::PipelinePlanner>(node);
  pipeline_planner->setMaxAccelerationScalingFactor(0.2);
  pipeline_planner->setMaxVelocityScalingFactor(0.2);

  auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
  cartesian_planner->setMaxAccelerationScalingFactor(0.2);
  cartesian_planner->setMaxVelocityScalingFactor(0.2);
  cartesian_planner->setStepSize(0.002);
  cartesian_planner->setJumpThreshold(2.0);

  // ----------(CURRENT STATE)----------------------
  Stage *Initial_pointer = nullptr;

  {
    auto stage = std::make_unique<stages::CurrentState>("Initial Position");
    t.add(std::move(stage));
  }

  // ----------(GRIPPER OPEN)----------------------
  {
    auto stage =
        std::make_unique<stages::MoveTo>("Gripper Open", pipeline_planner);
    stage->setGroup("hand");
    stage->setGoal("open");
    Initial_pointer = stage.get();
    t.add(std::move(stage));
  }

  // ----------(CONNECT - Initial state to Pick container)----------------------
  {
    auto stage = std::make_unique<stages::Connect>(
        "move to pick",
        stages::Connect::GroupPlannerVector{{"panda_arm", pipeline_planner}});
    stage->setTimeout(2.0);
    t.add(std::move(stage));
  }

  // ---(PICK CONTAINER with exposed to task properties)--------
  Stage *pick_stage_ptr = nullptr;
  {
    auto pick_container = std::make_unique<SerialContainer>("Pick");
    t.properties().exposeTo(pick_container->properties(),
                            {"eef", "hand", "group", "ik_frame"});
    pick_container->properties().configureInitFrom(
        Stage::PARENT, {"eef", "hand", "group", "ik_frame"});

    // ----------(APPROACH OBJECT)----------------------
    {
      auto stage = std::make_unique<stages::MoveRelative>("Approach Object",
                                                          cartesian_planner);
      stage->properties().configureInitFrom(Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.05, 0.1);
      geometry_msgs::msg::Vector3Stamped direction;
      direction.header.frame_id =
          "panda_link8"; // approach obeject does not influence the Grasp pose
      direction.vector.z = 0.1;
      stage->setMinMaxDistance(0.1, 0.15);
      stage->setDirection(direction);
      pick_container->insert(std::move(stage));
    }

    // ---(GENERATEGRASPPOSE WITH COMPUTEIK WRAPPER)------------
    {
      auto stage = std::make_unique<stages::GenerateGraspPose>("Grasp Pose");
      stage->setObject("wine_glass");
      stage->setAngleDelta(M_PI / 12);
      stage->setMonitoredStage(Initial_pointer);
      stage->setRotationAxis(
          Eigen::Vector3d::UnitZ()); // default is z , but here explicitly
                                     // definied for readablity.
      stage->properties().configureInitFrom(Stage::PARENT);
      stage->setPreGraspPose("open");

      // TODO(DONE): use the quternion to clen the setIKFrame
      Eigen::Isometry3d ik_transformation;
      Eigen::Quaterniond q =
          Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
          Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());
      ik_transformation.linear() = q.matrix();
      ik_transformation.translation().z() = 0.1;
      ik_transformation.translation().x() = -0.05;
      ik_transformation.translation().y() = 0.05;

      auto ik_wrapper =
          std::make_unique<stages::ComputeIK>("Compute IK", std::move(stage));
      ik_wrapper->properties().configureInitFrom(Stage::PARENT,
                                                 {"group", "eef"});
      ik_wrapper->setMaxIKSolutions(8);
      ik_wrapper->setIKFrame(ik_transformation, "panda_link8");
      ik_wrapper->properties().configureInitFrom(Stage::INTERFACE,
                                                 {"target_pose"});
      pick_container->insert(std::move(ik_wrapper));
    }

    // ----------(ENABLE COLLISON)----------------------
    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>(
          "Allow Collison (Hand, Object)");
      stage->allowCollisions("wine_glass",
                             t.getRobotModel()
                                 ->getJointModelGroup("hand")
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             true);
      pick_container->insert(std::move(stage));
    }

    // ----------(CLOSE GRIPPER)----------------------
    {
      auto stage =
          std::make_unique<stages::MoveTo>("Close Gripper", pipeline_planner);
      stage->setGroup("hand");
      stage->setGoal({{"panda_finger_joint1", 0.013},
                      {"panda_finger_joint2",
                       0.013}}); // used separate joint control , to overcome
                                 // the failure of full state completion.
      pick_container->insert(std::move(stage));
    }

    // ----------(ATTACH OBJECT)----------------------
    {
      auto stage =
          std::make_unique<stages::ModifyPlanningScene>("Attach Object");
      stage->attachObject("wine_glass", "panda_link8");
      pick_container->insert(std::move(stage));
    }

    // ----------(LIFT)----------------------
    {
      auto stage =
          std::make_unique<stages::MoveRelative>("Lift", cartesian_planner);
      stage->properties().configureInitFrom(Stage::PARENT, {"group"});
      geometry_msgs::msg::Vector3Stamped direction;
      direction.header.frame_id = "world";
      direction.vector.z = 0.3;
      stage->setDirection(direction);
      pick_container->insert(std::move(stage));
    }
    pick_stage_ptr = pick_container.get();
    t.add(std::move(pick_container));
  } // end of pick_container

  moveit_msgs::msg::OrientationConstraint ocm;
  ocm.link_name = "panda_link8";
  ocm.header.frame_id = "world";

  tf2::Quaternion q;
  q.setRPY(M_PI / 2, M_PI / 4, 0.0);

  ocm.orientation = tf2::toMsg(q);
  ocm.absolute_x_axis_tolerance = M_PI;
  ocm.absolute_y_axis_tolerance = M_PI;
  ocm.absolute_z_axis_tolerance = M_PI;

  ocm.weight = 1.0;
  moveit_msgs::msg::Constraints constraints;
  constraints.orientation_constraints.push_back(ocm);
  // ----------(CONNECT - MOVE TO PLACE)----------------------
  {
    auto stage = std::make_unique<stages::Connect>(
        "Move to Place - Pipeline",
        stages::Connect::GroupPlannerVector{{"panda_arm", pipeline_planner}});
    stage->setTimeout(5.0);
    stage->setPathConstraints(constraints);
    t.add(std::move(stage));
  }

  // ----------(PLACE CONTAINER)----------------------
  {
    auto place_container = std::make_unique<SerialContainer>("Place Container");
    t.properties().exposeTo(place_container->properties(),
                            {"ik_frame", "eef", "hand", "group"});
    place_container->properties().configureInitFrom(
        Stage::PARENT, {"ik_frame", "eef", "hand", "group"});

    // ----------(LOWER OBJECT)----------------------
    {
      auto stage = std::make_unique<stages::MoveRelative>("Lower Object",
                                                          cartesian_planner);
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = -0.3;
      stage->setDirection(vec);
      stage->properties().configureInitFrom(Stage::PARENT, {"group"});
      place_container->insert(std::move(stage));
    }

    // ------(GENERATEPLACEPOSE WITH COMPUTEIK WRAPPER)-------------
    {
      auto stage = std::make_unique<stages::GeneratePlacePose>("Place Pose");
      stage->setMonitoredStage(pick_stage_ptr);
      stage->properties().configureInitFrom(Stage::PARENT, {"ik_frame"});
      stage->setObject("wine_glass");

      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "world";
      pose.pose.position.x = 0.45;
      pose.pose.position.y = 0.4;
      pose.pose.position.z = 0.1;
      stage->setPose(pose);
      // TODO(DONE) : replace the ik with Eigen
      Eigen::Isometry3d place_transformation;

      Eigen::Quaterniond q =
          Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
          Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY());
      place_transformation.linear() = q.matrix();

      auto ik_wrapper =
          std::make_unique<stages::ComputeIK>("ComputeIK", std::move(stage));
      ik_wrapper->setMaxIKSolutions(2);
      ik_wrapper->setIKFrame(place_transformation, "panda_link8");
      ik_wrapper->properties().configureInitFrom(Stage::PARENT,
                                                 {"group", "eef"});
      ik_wrapper->properties().configureInitFrom(Stage::INTERFACE,
                                                 {"target_pose"});
      place_container->insert(std::move(ik_wrapper));

      // ----------(OPEN GRIPPER)----------------------
      {
        auto stage =
            std::make_unique<stages::MoveTo>("Open Gripper", pipeline_planner);
        stage->setGroup("hand");
        stage->setGoal("open");
        place_container->insert(std::move(stage));
      }
    }

    // ----------(FORBID COLLISON)----------------------
    {
      auto stage =
          std::make_unique<stages::ModifyPlanningScene>("forbid collison");
      stage->allowCollisions(
          "wine_glass", *t.getRobotModel()->getJointModelGroup("hand"), false);
      place_container->insert(std::move(stage));
    }

    // ----------(DETACH OBJECT)----------------------
    {
      auto stage =
          std::make_unique<stages::ModifyPlanningScene>("detach object");
      stage->detachObject("wine_glass", "panda_link8");
      place_container->insert(std::move(stage));
    }

    // ----------(RETREAT BACK)----------------------
    {
      auto stage = std::make_unique<stages::MoveRelative>("retreat after place",
                                                          cartesian_planner);
      stage->setGroup("panda_arm");
      stage->setMinMaxDistance(0.12, 0.25);
      stage->properties().configureInitFrom(Stage::PARENT, {"group"});
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "panda_link8";
      vec.vector.z = -0.5;
      stage->setDirection(vec);
      place_container->insert(std::move(stage));
    }

    t.add(std::move(place_container));
  } // end of place container

  // ----------(MOVE HOME)----------------------
  {
    auto stage =
        std::make_unique<stages::MoveTo>("move home", pipeline_planner);
    stage->properties().configureInitFrom(Stage::PARENT, {"group"});
    stage->setGoal("ready");
    stage->restrictDirection(stages::MoveTo::FORWARD);
    t.add(std::move(stage));
  }

  // ----------(PLAN & EXECUTE)----------------------
  try {
    t.init();
    t.plan(10);
    t.introspection().publishSolution(*t.solutions().front());
    t.execute(*t.solutions().front());
  } catch (const InitStageException &e) {
    std::cerr << "Initialization error: " << e << std::endl;
  }

  spinner.join();

  return 0;
}
