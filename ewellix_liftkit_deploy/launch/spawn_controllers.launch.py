#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

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
            default_value="false",
        )
    )
    namespace = LaunchConfiguration("namespace")

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

    return LaunchDescription(declared_arguments + [position_controller_spawner])