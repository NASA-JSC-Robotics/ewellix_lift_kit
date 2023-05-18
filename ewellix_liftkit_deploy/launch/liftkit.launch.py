import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue

import xacro

from launch_ros.actions import Node
def generate_launch_description():
    declared_arguments = []
    # xacro args
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
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "com_port",
            default_value="/dev/ttyUSB0",
            description="com port for the ewellix",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "height_limit",
            default_value="0.53",
            description="Maximium height in meters for the lift",
        )
    )

    # other args
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz",
            default_value='true',
            description="launch rviz",
        )
    )
    robot_name = LaunchConfiguration("robot_name")
    tf_prefix = LaunchConfiguration("tf_prefix")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    com_port = LaunchConfiguration("com_port")
    rviz = LaunchConfiguration("rviz")
    height_limit = LaunchConfiguration("height_limit")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ewellix_liftkit_description"), "urdf", "ewellix_lift_700mm.urdf.xacro"]),
            " ", 
            "name:=",
            robot_name,
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "com_port:=",
            com_port,
            " ",
            "height_limit:=",
            height_limit,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    controller_params_file = os.path.join(get_package_share_directory('ewellix_liftkit_deploy'),'config','liftkit_controllers.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description,
                    controller_params_file]
    )

    
    position_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_trajectory_controller", "--controller-manager-timeout",
                "100",],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager-timeout",
                "100",],
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ewellix_liftkit_deploy"), "rviz", "view_robot.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(rviz)
    )

    nodes = [
        robot_state_publisher,
        controller_manager,
        position_trajectory_controller_spawner,
        joint_state_broadcaster_spawner, 
        rviz_node
    ]

    return LaunchDescription(declared_arguments + nodes)
