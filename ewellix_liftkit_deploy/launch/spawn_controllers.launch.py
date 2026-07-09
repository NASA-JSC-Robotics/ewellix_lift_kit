#!/usr/bin/env python3

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


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
            "namespace",
            default_value="",
            description="Namespace for the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value=str(params['use_fake_hardware']).lower(),
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    
    namespace = LaunchConfiguration("namespace")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "100",
            "-c",
            "controller_manager",
        ],
    )

    # Position controller spawner
    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=[
            "lift_position_controller",
            "--controller-manager-timeout",
            "100",
            "-c",
            "controller_manager",
        ],
    )

    return LaunchDescription(
        declared_arguments + [
            joint_state_broadcaster_spawner,
            position_controller_spawner,
        ]
    )