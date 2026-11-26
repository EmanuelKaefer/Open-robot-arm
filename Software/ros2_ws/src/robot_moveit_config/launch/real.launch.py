import os

from launch import LaunchDescription, LaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import (
    generate_move_group_launch,
    generate_moveit_rviz_launch,
    generate_rsp_launch,
    generate_spawn_controllers_launch,
    generate_static_virtual_joint_tfs_launch,
)


def generate_launch_description():
    port = os.environ.get("ARM_PORT", "/dev/ttyACM0")
    moveit_config = (
        MoveItConfigsBuilder("robot_armV2SLDASM", package_name="robot_moveit_config")
        .robot_description(mappings={"use_mock_hardware": "false", "port": port})
        .to_moveit_configs()
    )

    # Every part gets THIS moveit_config. generate_demo_launch() would include the
    # package's own rsp.launch.py etc., which build a config without mappings (= mock).
    def include(ld):
        return IncludeLaunchDescription(LaunchDescriptionSource(ld))

    return LaunchDescription([
        include(generate_static_virtual_joint_tfs_launch(moveit_config)),
        include(generate_rsp_launch(moveit_config)),   # publishes /robot_description
        include(generate_move_group_launch(moveit_config)),
        include(generate_moveit_rviz_launch(moveit_config)),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[str(moveit_config.package_path / "config/ros2_controllers.yaml")],
            remappings=[("/controller_manager/robot_description", "/robot_description")],
            output="screen",
        ),
        include(generate_spawn_controllers_launch(moveit_config)),
    ])