#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/task.h>
#include <rclcpp/rclcpp.hpp>

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

  auto scene =
      std::make_shared<planning_scene::PlanningScene>(t.getRobotModel());
  auto &robot_state = scene->getCurrentStateNonConst();
  robot_state.setToDefaultValues();
  robot_state.setToDefaultValues(robot_state.getJointModelGroup("panda_arm"),
                                 "ready");

  auto pipeline_planner = std::make_shared<solvers::PipelinePlanner>(node);
  auto cartesian_planner = std::make_shared<solvers::CartesianPath>();

  Stage *Initial_pointer = nullptr;
  {
    auto stage = std::make_unique<stages::CurrentState>("Initial");
    t.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<stages::MoveTo>("MoveTo", pipeline_planner);
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

  {
    auto stage = std::make_unique<stages::GenerateGraspPose>("Grasp Pose");
    stage->setObject("cylinder");
    stage->setAngleDelta(M_PI / 4);
    stage->setMonitoredStage(Initial_pointer);
    stage->setEndEffector("hand");
    stage->setPreGraspPose("open");

    auto ik_wrapper =
        std::make_unique<stages::ComputeIK>("compute_ik", std::move(stage));
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
    t.add(std::move(ik_wrapper));
  }

  try {
    t.init();
    if (t.plan() && !t.solutions().empty()) {
      t.introspection().publishSolution(*t.solutions().front());
    } else {
      RCLCPP_ERROR(node->get_logger(),
                   "The planning Failed or no solutions are found");
    }
  } catch (InitStageException &e) {
    std::cerr << "Error" << e << std::endl;
  }

  spinner.join();
  rclcpp::shutdown();

  return 0;
}
