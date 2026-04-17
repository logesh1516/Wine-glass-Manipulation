#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/task.h>
#include <rclcpp/rclcpp.hpp>
#include <utility>

using namespace moveit::task_constructor;

int main(int argc, char *argv[]) {
  // ----------(ROS2 NODE)----------------------
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "MTC_node",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
          true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor] { executor.spin(); });

  // ----------(COLLISION OBJECT)----------------------
  moveit_msgs::msg::CollisionObject cylinder;
  cylinder.header.frame_id = "world";
  cylinder.id = "cylinder";

  shape_msgs::msg::SolidPrimitive primitives;
  primitives.type = primitives.CYLINDER;
  primitives.dimensions.resize(2);
  primitives.dimensions[primitives.CYLINDER_RADIUS] = 0.02;
  primitives.dimensions[primitives.CYLINDER_HEIGHT] = 0.2;

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.45;
  pose.position.y = 0.15;
  pose.position.z = 0.1;

  cylinder.primitives.push_back(primitives);
  cylinder.primitive_poses.push_back(pose);
  cylinder.operation = cylinder.ADD;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(cylinder);

  // ----------(TASK)----------------------
  Task t;
  t.setName("mtc_demo");
  t.loadRobotModel(node);
  // ----------(TASK PROPERTIES)----------------------
  t.setProperty("group", "panda_arm");
  t.setProperty("eef", "hand");
  t.setProperty("hand", "hand");
  t.setProperty("ik_frame", "panda_link8");

  auto pipeline_planner = std::make_shared<solvers::PipelinePlanner>(node);
  auto cartesian_planner = std::make_shared<solvers::CartesianPath>();

  Stage *Initial_pointer = nullptr;
  {
    auto stage = std::make_unique<stages::CurrentState>("Initial");
    t.add(std::move(stage));
  }

  {
    auto stage =
        std::make_unique<stages::MoveTo>("Gripper Open", pipeline_planner);
    stage->setGroup("hand");
    stage->setGoal("open");
    Initial_pointer = stage.get();
    t.add(std::move(stage));
  }

  {
    auto connect = std::make_unique<stages::Connect>(
        "connect",
        stages::Connect::GroupPlannerVector{{"panda_arm", pipeline_planner}});
    t.add(std::move(connect));
  }
  Stage *pick_stage_ptr = nullptr;
  {
    auto pick_container = std::make_unique<SerialContainer>("Pick");
    {
      auto stage = std::make_unique<stages::MoveRelative>("Approach Object",
                                                          cartesian_planner);
      stage->setGroup("panda_arm");
      geometry_msgs::msg::Vector3Stamped direction;
      direction.header.frame_id = "panda_link8";
      direction.vector.z = 0.1;
      stage->setDirection(direction);
      pick_container->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<stages::GenerateGraspPose>("Grasp Pose");
      stage->setObject("cylinder");
      stage->setAngleDelta(M_PI / 12);
      stage->setMonitoredStage(Initial_pointer);
      stage->setEndEffector("hand");
      stage->setPreGraspPose("open");

      auto ik_wrapper =
          std::make_unique<stages::ComputeIK>("Compute IK", std::move(stage));
      ik_wrapper->setGroup("panda_arm");
      ik_wrapper->setMaxIKSolutions(8);
      ik_wrapper->setEndEffector("hand");
      ik_wrapper->setIKFrame(
          Eigen::Isometry3d(Eigen::Translation3d(0, 0, 0.1) *
                            Eigen::AngleAxisd(1.571, Eigen::Vector3d::UnitX()) *
                            Eigen::AngleAxisd(0.785, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(1.571, Eigen::Vector3d::UnitZ())),
          "panda_link8");
      ik_wrapper->properties().configureInitFrom(Stage::INTERFACE,
                                                 {"target_pose"});
      pick_container->insert(std::move(ik_wrapper));
    }
    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>(
          "allow collison (hand,object)");
      stage->allowCollisions("cylinder",
                             t.getRobotModel()
                                 ->getJointModelGroup("hand")
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             true);
      pick_container->insert(std::move(stage));
    }

    {
      auto stage =
          std::make_unique<stages::MoveTo>("Close Gripper", pipeline_planner);
      stage->setGroup("hand");
      stage->setGoal("close");
      pick_container->insert(std::move(stage));
    }
    {
      auto stage =
          std::make_unique<stages::ModifyPlanningScene>("Attach Object");
      stage->attachObject("cylinder", "panda_link8");
      pick_container->insert(std::move(stage));
    }
    {
      auto stage =
          std::make_unique<stages::MoveRelative>("Lift", cartesian_planner);
      stage->setGroup("panda_arm");
      geometry_msgs::msg::Vector3Stamped direction;
      direction.header.frame_id = "world";
      direction.vector.z = 0.3;
      stage->setDirection(direction);
      pick_container->insert(std::move(stage));
    }
    pick_stage_ptr = pick_container.get();
    t.add(std::move(pick_container));
  }

  {
    auto place_container = std::make_unique<SerialContainer>("Place Container");
    {
      auto connect = std::make_unique<stages::Connect>(
          "connect",
          stages::Connect::GroupPlannerVector{{"panda_arm", pipeline_planner}});
      place_container->insert(std::move(connect));
    }

    {
      auto stage = std::make_unique<stages::MoveRelative>("lower object",
                                                          cartesian_planner);
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = -0.3;
      stage->setDirection(vec);
      stage->setGroup("panda_arm");
      place_container->insert(std::move(stage));
    }
    {
      auto stage =
          std::make_unique<stages::GeneratePlacePose>("generate_place_pose");
      stage->setMonitoredStage(pick_stage_ptr);
      stage->properties().configureInitFrom(Stage::PARENT, {"ik_frame"});
      stage->setObject("cylinder");
      stage->properties().set("marker_ns", "place_pose");
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "world";
      pose.pose.position.x = 0.45;
      pose.pose.position.y = 0.4;
      pose.pose.position.z = 0.1;
      stage->setPose(pose);

      auto ik_wrapper =
          std::make_unique<stages::ComputeIK>("ComputeIK", std::move(stage));
      ik_wrapper->setMaxIKSolutions(2);
      ik_wrapper->setIKFrame(
          Eigen::Isometry3d(Eigen::AngleAxisd(1.571, Eigen::Vector3d::UnitX()) *
                            Eigen::AngleAxisd(0.785, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(1.571, Eigen::Vector3d::UnitZ())),
          "panda_link8");
      ik_wrapper->setGroup("panda_arm");
      ik_wrapper->setEndEffector("hand");
      ik_wrapper->properties().configureInitFrom(Stage::INTERFACE,
                                                 {"target_pose"});
      place_container->insert(std::move(ik_wrapper));
      {
        auto stage =
            std::make_unique<stages::MoveTo>("Open Gripper", pipeline_planner);
        stage->setGroup("hand");
        stage->setGoal("open");
        place_container->insert(std::move(stage));
      }
    }
    {
      auto stage =
          std::make_unique<stages::ModifyPlanningScene>("forbid collison");
      stage->allowCollisions(
          "cylinder", *t.getRobotModel()->getJointModelGroup("hand"), false);
      place_container->insert(std::move(stage));
    }

    {
      auto stage =
          std::make_unique<stages::ModifyPlanningScene>("detach object");
      stage->detachObject("cylinder", "panda_link8");
      place_container->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<stages::MoveRelative>("retreat after place",
                                                          cartesian_planner);
      stage->setGroup("panda_arm");
      stage->setMinMaxDistance(.12, .25);
      stage->setIKFrame("panda_link8");
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "panda_link8";
      vec.vector.z = -1.0;
      stage->setDirection(vec);
      place_container->insert(std::move(stage));
    }

    t.add(std::move(place_container));
  }
  {
    auto stage =
        std::make_unique<stages::MoveTo>("move home", pipeline_planner);
    stage->setGroup("panda_arm");
    stage->setGoal("ready");
    stage->restrictDirection(stages::MoveTo::FORWARD);
    t.add(std::move(stage));
  }

  try {
    t.init();
    t.plan(10);
    t.introspection().publishSolution(*t.solutions().front());
  } catch (const InitStageException &e) {
    std::cerr << "Initialization error: " << e << std::endl;
  }

  spinner.join();

  return 0;
}
