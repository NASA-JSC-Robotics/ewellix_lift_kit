from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

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

    return LaunchDescription([position_trajectory_controller_spawner])
