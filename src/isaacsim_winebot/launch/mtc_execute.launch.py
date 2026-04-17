from launch_ros.actions import Node
from moveit_configs_utils.moveit_configs_builder import MoveItConfigsBuilder
from launch import LaunchDescription


def generate_launch_description():

    moveit_builder_config = (
        MoveItConfigsBuilder
        ("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .to_moveit_configs()
    )
    node = Node(
        package="isaacsim_winebot",
        executable="mtc_winebot",
        output="screen",
        parameters=[
            moveit_builder_config.to_dict()
        ]
    )
    return LaunchDescription([node])
