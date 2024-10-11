from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition


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
            "max_height",
            default_value="500",
            description="Maximum height of the ewellix lifting column \
                         at full extension. Please enter value in mm.",
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
            default_value="0.7",
            description="Maximum height in meters for the lift",
        )
    )

    # other args
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="launch rviz",
        )
    )
    robot_name = LaunchConfiguration("robot_name")
    tf_prefix = LaunchConfiguration("tf_prefix")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    max_height = LaunchConfiguration("max_height")
    com_port = LaunchConfiguration("com_port")
    rviz = LaunchConfiguration("rviz")
    height_limit = LaunchConfiguration("height_limit")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ewellix_liftkit_description"), "urdf", "ewellix_lift_700mm.urdf.xacro"]
            ),
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
            "max_height:=",
            max_height,
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

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ewellix_liftkit_description"), "rviz", "view_robot.rviz"]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(rviz),
    )

    nodes_to_start = [
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
