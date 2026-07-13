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
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare


def load_parameters():
    """Load parameters from central YAML file"""
    pkg_path = get_package_share_directory("ewellix_liftkit_deploy")
    yaml_path = os.path.join(pkg_path, "config", "ewellix_liftkit_parameters.yaml")
    
    with open(yaml_path, 'r') as f:
        params = yaml.safe_load(f)
    
    return params['ewellix_liftkit']


def generate_launch_description():
    # Load parameters from central YAML file
    params = load_parameters()
    
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_name",
            default_value="ewellix_liftkit",
            description="name of the robot",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="Prefix of the joint names, useful for multi-robot setup.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value=str(params['use_fake_hardware']).lower(),
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "com_port_top",
            default_value=params['com_port_top'],
            description="Serial port for top Elmo motor",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "com_port_bottom",
            default_value=params['com_port_bottom'],
            description="Serial port for bottom Elmo motor",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "height_limit",
            default_value=str(params['height_limit']),
            description="Maximum height in meters for the lift",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "is_700",
            default_value=str(params['is_700']).lower(),
            description="Set to true to use the 700mm stroke liftkit configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "is_500",
            default_value=str(params['is_500']).lower(),
            description="Set to true to use the 500mm stroke liftkit configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "max_ticks_mot_1",
            default_value=str(params['max_ticks_mot_1']),
            description="Max ticks for motor 1",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "max_ticks_mot_2",
            default_value=str(params['max_ticks_mot_2']),
            description="Max ticks for motor 2",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "min_height_m",
            default_value=str(params['min_height_m']),
            description="Minimum height in meters",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "max_height_m",
            default_value=str(params['max_height_m']),
            description="Maximum height in meters",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "homing_current_a",
            default_value=str(params['homing_current_a']),
            description="Homing current in amps",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "stall_velocity_thresh",
            default_value=str(params['stall_velocity_thresh']),
            description="Stall velocity threshold",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "stall_time_ms",
            default_value=str(params['stall_time_ms']),
            description="Stall time in milliseconds",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "homing_timeout_ms",
            default_value=str(params['homing_timeout_ms']),
            description="Homing timeout in milliseconds",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "poll_ms",
            default_value=str(params['poll_ms']),
            description="Poll time in milliseconds",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "backoff_counts",
            default_value=str(params['backoff_counts']),
            description="Backoff counts",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "top_home_direction",
            default_value=str(params['top_home_direction']),
            description="Top home direction",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "bottom_home_direction",
            default_value=str(params['bottom_home_direction']),
            description="Bottom home direction",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "calibration_direction",
            default_value=params['calibration_direction'],
            description="Calibration direction",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_value",
            default_value=str(params['initial_value']),
            description="Initial value",
        )
    )
    # === ADD MOTOR CONTROL PARAMETERS ===
    declared_arguments.append(
        DeclareLaunchArgument(
            "motor_acceleration",
            default_value=str(params['motor_acceleration']),
            description="Motor acceleration (AC parameter)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "motor_deceleration",
            default_value=str(params['motor_deceleration']),
            description="Motor deceleration (DC parameter)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "motor_stop_decel",
            default_value=str(params['motor_stop_decel']),
            description="Motor stop deceleration (SD parameter)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "motor_speed_profile",
            default_value=str(params['motor_speed_profile']),
            description="Motor speed profile (SP parameter)",
        )
    )
    # =====================================
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="launch rviz",
        )
    )

    robot_name              = LaunchConfiguration("robot_name")
    tf_prefix               = LaunchConfiguration("tf_prefix")
    use_fake_hardware       = LaunchConfiguration("use_fake_hardware")
    rviz                    = LaunchConfiguration("rviz")
    com_port_top            = LaunchConfiguration("com_port_top")
    com_port_bottom         = LaunchConfiguration("com_port_bottom")
    height_limit            = LaunchConfiguration("height_limit")
    is_500                  = LaunchConfiguration("is_500")
    is_700                  = LaunchConfiguration("is_700")
    max_ticks_mot_1         = LaunchConfiguration("max_ticks_mot_1")
    max_ticks_mot_2         = LaunchConfiguration("max_ticks_mot_2")
    min_height_m            = LaunchConfiguration("min_height_m")
    max_height_m            = LaunchConfiguration("max_height_m")
    homing_current_a        = LaunchConfiguration("homing_current_a")
    stall_velocity_thresh   = LaunchConfiguration("stall_velocity_thresh")
    stall_time_ms           = LaunchConfiguration("stall_time_ms")
    homing_timeout_ms       = LaunchConfiguration("homing_timeout_ms")
    poll_ms                 = LaunchConfiguration("poll_ms")
    backoff_counts          = LaunchConfiguration("backoff_counts")
    top_home_direction      = LaunchConfiguration("top_home_direction")
    bottom_home_direction   = LaunchConfiguration("bottom_home_direction")
    calibration_direction   = LaunchConfiguration("calibration_direction")
    initial_value           = LaunchConfiguration("initial_value")
    # === ADD MOTOR CONTROL PARAMETERS ===
    motor_acceleration      = LaunchConfiguration("motor_acceleration")
    motor_deceleration      = LaunchConfiguration("motor_deceleration")
    motor_stop_decel        = LaunchConfiguration("motor_stop_decel")
    motor_speed_profile     = LaunchConfiguration("motor_speed_profile")
    # =====================================

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ewellix_liftkit_description"), "urdf", "ewellix_lift.urdf.xacro"]),
            " ",
            "name:=",                 robot_name,
            " ",
            "tf_prefix:=",            tf_prefix,
            " ",
            "use_fake_hardware:=",    use_fake_hardware,
            " ",
            "com_port_top:=",         com_port_top,
            " ",
            "com_port_bottom:=",      com_port_bottom,
            " ",
            "height_limit:=",         height_limit,
            " ",
            "is_500:=",               is_500,
            " ",
            "is_700:=",               is_700,
            " ",
            "max_ticks_mot_1:=",      max_ticks_mot_1,
            " ",
            "max_ticks_mot_2:=",      max_ticks_mot_2,
            " ",
            "min_height_m:=",         min_height_m,
            " ",
            "max_height_m:=",         max_height_m,
            " ",
            "homing_current_a:=",     homing_current_a,
            " ",
            "stall_velocity_thresh:=", stall_velocity_thresh,
            " ",
            "stall_time_ms:=",        stall_time_ms,
            " ",
            "homing_timeout_ms:=",    homing_timeout_ms,
            " ",
            "poll_ms:=",              poll_ms,
            " ",
            "backoff_counts:=",       backoff_counts,
            " ",
            "top_home_direction:=",   top_home_direction,
            " ",
            "bottom_home_direction:=", bottom_home_direction,
            " ",
            "calibration_direction:=", calibration_direction,
            " ",
            "initial_value:=",        initial_value,
            " ",
            "motor_acceleration:=",   motor_acceleration,
            " ",
            "motor_deceleration:=",   motor_deceleration,
            " ",
            "motor_stop_decel:=",     motor_stop_decel,
            " ",
            "motor_speed_profile:=",  motor_speed_profile,
            # =====================================
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="log",
        parameters=[robot_description],
    )

    controller_common_params = ParameterFile(
        PathJoinSubstitution([FindPackageShare("ewellix_liftkit_deploy"), "config", "controllers_common.yaml"]),
        allow_substs=True,
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            controller_common_params,
        ],
    )

    rviz_config_file = PathJoinSubstitution([FindPackageShare("ewellix_liftkit_deploy"), "rviz", "view_robot.rviz"])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(rviz),
    )

    nodes = [robot_state_publisher, controller_manager, rviz_node]

    spawn_controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
          os.path.join(get_package_share_directory("ewellix_liftkit_deploy"), "launch", "spawn_controllers.launch.py")
        ),
        launch_arguments={
            "use_fake_hardware": use_fake_hardware,
        }.items(),
    )

    return LaunchDescription(declared_arguments + nodes + [spawn_controllers_launch])
