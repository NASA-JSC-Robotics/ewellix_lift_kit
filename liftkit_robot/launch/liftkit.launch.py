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
    # Get URDF via xacro
    package_name='liftkit' #<--- CHANGE ME

    
    ip_addr = '192.168.7.2'
    port = '8000'

    # xacro_file = os.path.join(get_package_share_directory(package_name),'description','rail_e.urdf.xacro')
    # robot_description_config = xacro.process_file(xacro_file)

    # # Create a robot_state_publisher node
    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','liftkit_controllers.yaml')
    # params = {'robot_description': robot_description_config.toxml(), 'ip_addr': ip_addr, 'port' : port}

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(package_name), "description", 'ewellix_lift_700mm.urdf.xacro']),
            " ",
            # "ip_addr:=",
            # ip_addr,
            # " ",
            # "port:=",
            # port,
            # " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

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

    # io_and_status_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["io_and_status_controller", "--controller-manager-timeout",
    #             "100",],
    # )

    # robot_status_publisher = Node(
    #      package='liftkit_hardware_interface',
    #      executable='robot_status_publisher', 
    #      name='robot_status_publisher' 
    #    )
    
    # controller_stopper = Node(
    #      package='liftkit_hardware_interface',
    #      executable='controller_stopper_node', 
    #      name='controller_stopper_node',
    #      parameters=[
    #         {
    #             "consistent_controllers": [
    #                 "io_and_status_controller",
    #                 "joint_state_broadcaster",
    #             ]
    #         },
    #     ],
    #    )


    nodes = [
        rsp,
        controller_manager,
        position_trajectory_controller_spawner,
        joint_state_broadcaster_spawner
        # io_and_status_controller_spawner,
        # controller_stopper
    ]

    return LaunchDescription(nodes)
