from launch_ros.actions import Node
from moveit_configs_utils.moveit_configs_builder import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from os.path import join


def generate_launch_description():

    rviz_config_file = join(get_package_share_directory(
        "isaacsim_winebot"), "rviz", "mtc.rviz")
    ros2_control_file = join(get_package_share_directory(
        "moveit_resources_panda_moveit_config"), "config", "ros2_controllers.yaml")

    moveit_builder_config = (
        MoveItConfigsBuilder
        ("moveit_resources_panda")
        .robot_description(
            file_path="config/panda.urdf.xacro",
        )
        .planning_pipelines(pipelines=["ompl"])
        .trajectory_execution(
            file_path="config/gripper_moveit_controllers.yaml")
        .to_moveit_configs()
    )

    mtc_capabilites = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
                moveit_builder_config.to_dict(),
                mtc_capabilites
        ]
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_builder_config.robot_description,
            moveit_builder_config.planning_pipelines,
            moveit_builder_config.robot_description_semantic,
            moveit_builder_config.robot_description_kinematics,

        ]
    )

    static_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "world", "panda_link0"],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_builder_config.robot_description,
        ]
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            moveit_builder_config.robot_description, ros2_control_file,
        ]
    )

    load_controllers = []

    for controllers in [
        "joint_state_broadcaster",
        "panda_arm_controller",
        "panda_hand_controller",
    ]:

        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controllers)], shell=True, output="screen")
        ]

    return LaunchDescription(
        [
            rviz2,
            static_transform,
            move_group_node,
            robot_state_publisher,
            ros2_control_node,
            *load_controllers
        ]
    )
