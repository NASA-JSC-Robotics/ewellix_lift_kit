from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():

    declared_arguments = []

    position_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "lift_position_trajectory_controller",
            "--controller-manager-timeout",
            "100",
            "-c",
            "controller_manager",
            "-t",
            "joint_trajectory_controller/JointTrajectoryController",
        ],
    )

    nodes = [position_trajectory_controller_spawner]

    return LaunchDescription(declared_arguments + nodes)
