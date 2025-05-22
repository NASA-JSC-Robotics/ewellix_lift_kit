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
