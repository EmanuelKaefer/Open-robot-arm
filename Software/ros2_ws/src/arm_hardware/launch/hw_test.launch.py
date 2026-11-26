# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (c) 2026 Emanuel Käfer, Matteo Gassler, and Alexander Dick and contributors
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# This program is distributed WITHOUT ANY EXPRESS OR IMPLIED WARRANTY.
# See LICENSE-SOFTWARE in the project root for details.

"""Test ArmSystem without MoveIt: ros2_control_node + joint_state_broadcaster + arm_controller.

Usage (simulator):  ros2 launch arm_hardware hw_test.launch.py
       (hardware):  ros2 launch arm_hardware hw_test.launch.py port:=/dev/ttyACM0
Check:              ros2 topic echo /joint_states --once
Drive:              ros2 run arm_hardware test_trajectory.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    port = LaunchConfiguration('port')
    share = FindPackageShare('arm_hardware')

    robot_description = ParameterValue(
        Command([
            'xacro ', PathJoinSubstitution([share, 'bringup', 'arm_test.urdf.xacro']),
            ' port:=', port,
        ]),
        value_type=str,
    )
    controllers = PathJoinSubstitution([share, 'bringup', 'controllers.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'port', default_value='/tmp/ttySTM',
            description='Serial port: /tmp/ttySTM (stm_sim.py) or /dev/ttyACM0 (board)'),

        # Publishes robot_description (ros2_control_node subscribes to it) and TF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[controllers],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', 'arm_controller'],
            output='screen',
        ),
    ])
