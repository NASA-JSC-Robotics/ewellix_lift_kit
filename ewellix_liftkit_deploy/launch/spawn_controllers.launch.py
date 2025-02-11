from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
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
    ewellix_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "ewellix_state_broadcaster",
            "--controller-manager-timeout",
            "100",
            "-c",
            "controller_manager",
            "-t",
            "joint_state_broadcaster/JointStateBroadcaster",
        ],
    )

    nodes = [position_trajectory_controller_spawner, ewellix_state_broadcaster]

    return LaunchDescription(declared_arguments + nodes)
