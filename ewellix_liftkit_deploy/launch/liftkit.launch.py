import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
    declared_arguments.append(
        DeclareLaunchArgument(
            "is_700",
            default_value="true",
            description="Set to true to use the 500mm stroke liftkit configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "is_500",
            default_value="false",
            description="Set to true to use the 500mm stroke liftkit configuration.",
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
    declared_arguments.append(
        DeclareLaunchArgument(
            "world_file",
            default_value="empty.sdf",
            description="Gazebo world file (absolute path or filename from the gazebosim worlds collection) containing a custom world.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_gazebo",
            default_value="false",
            description="Launches gazebo",
        )
    )
    robot_name = LaunchConfiguration("robot_name")
    tf_prefix = LaunchConfiguration("tf_prefix")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    rviz = LaunchConfiguration("rviz")
    com_port = LaunchConfiguration("com_port")
    height_limit = LaunchConfiguration("height_limit")
    is_500 = LaunchConfiguration("is_500")
    is_700 = LaunchConfiguration("is_700")
    world_file = LaunchConfiguration("world_file")
    use_gazebo = LaunchConfiguration("use_gazebo")


    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ewellix_liftkit_description"), "urdf", "ewellix_lift.urdf.xacro"]),
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
            "is_500:=",
            is_500,
            " ",
            "is_700:=",
            is_700,
            " ",
            "use_gazebo:=",
            use_gazebo,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description,
            {"use_sim_time": use_gazebo}],
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
            # "-c",
            # "controller_manager",
            "--controller-manager-timeout",
            "100",
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
        parameters=[
            {"use_sim_time": use_gazebo}]
    )
    # GZ nodes
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            '-topic', 'robot_description',
            "-name",
            "ewellix_liftkit",
            "-allow_renaming",
            "true",
        ],
    )

    gz_launch_description_with_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": [" -r -v 4 ", world_file]}.items(),
        condition=IfCondition(use_gazebo),
    )

    gz_launch_description_without_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": [" -s -r -v 4 ", world_file]}.items(),
        condition=UnlessCondition(use_gazebo),
    )

   
    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
    )

    nodes = [robot_state_publisher, joint_state_broadcaster_spawner, rviz_node, gz_spawn_entity, gz_launch_description_with_gui, gz_launch_description_without_gui, gz_sim_bridge]

    spawn_controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ewellix_liftkit_deploy"), "launch", "spawn_controllers.launch.py")
        ),
        launch_arguments={
            "use_fake_hardware": use_fake_hardware,
        }.items(),
    )

    return LaunchDescription(declared_arguments + nodes + [spawn_controllers_launch])
