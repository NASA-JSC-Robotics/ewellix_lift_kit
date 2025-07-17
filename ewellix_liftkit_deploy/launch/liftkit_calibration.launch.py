#!/usr/bin/env python3
#
# Copyright (c) 2025, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
#
# All rights reserved.
#
# This software is licensed under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "com_port",
            default_value="/dev/ttyUSB0",
            description="com port for the ewellix",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "calibration_direction",
            default_value="down",
            description="com port for the ewellix",
            choices=["up", "down"],
        )
    )

    com_port = LaunchConfiguration("com_port")
    calibration_direction = LaunchConfiguration("calibration_direction")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ewellix_liftkit_description"), "urdf", "ewellix_lift.urdf.xacro"]),
            " ",
            "name:=",
            "ewellix_liftkit_calibration",
            " ",
            "com_port:=",
            com_port,
            " ",
            "calibration_direction:=",
            calibration_direction,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    controller_params_file = os.path.join(
        get_package_share_directory("ewellix_liftkit_deploy"), "config", "liftkit_controllers.yaml"
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_params_file],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "-c",
            "controller_manager",
            "--controller-manager-timeout",
            "100",
        ],
    )

    nodes = [robot_state_publisher, controller_manager, joint_state_broadcaster_spawner]

    return LaunchDescription(declared_arguments + nodes)
