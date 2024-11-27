from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution

from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node

def generate_launch_description():

    # package must include all launch files
    package_name = 'base_package'

    # Declare Arguments

    # Initialized Arguments

    # Robot State Publisher
    args_robot_state_publisher = {'use_sim_time': 'false', 'use_ros2_control': 'true'}
    node_robot_state_publisher = get_launch_file('rsp.launch.py', args_robot_state_publisher)

    # ros2_control Controller Manager
    args_controller_manager = {}
    node_controller_manager = get_launch_file('controller_manager.launch.py', args_controller_manager)

    # pre-packaged Forward Position Controller. Delay until Controller Manager launches.
    args_controller = {}
    node_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["forward_position_controller"],
    )
    node_delay_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=node_controller_manager,
            on_start=[mode_controller],
        )
    )

    # Joint Broadcast
    node_joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["my_joint_state_broadcaster"],
    )
    node_delay_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=node_controller_manager,
            on_start=[node_joint_state_broadcaster_spawner],
        )
    )

    # RViz (uses config file)
    node_rviz = get_launch_file('rviz.launch.py', {})
    node_delay_rviz = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=node_delay_joint_state_broadcaster_spawner,
            on_start=[node_rviz],
        )
    )

    # Launch all
    return LaunchDescription([
        node_robot_state_publisher,
        node_controller_manager,
        node_delay_controller,
        node_delay_joint_state_broadcaster_spawner,
        node_delay_rviz
    ])

def get_launch_file(file_name, launch_args):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(package_name),
                'launch',
                file_name
            ])
        ]),
        launch_arguments=launch_args.items()
    )