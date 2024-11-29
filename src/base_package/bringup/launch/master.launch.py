from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution

from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit

from launch_ros.actions import Node

def generate_launch_description():

    # package must include all launch files
    package_name = 'base_package'

    # ________________ Declare Arguments ________________
    
    # ________________ Initialized Arguments ________________

    # ________________ Robot State Publisher ________________
    args_robot_state_publisher = {'use_sim_time': 'false', 'use_ros2_control': 'true'}
    node_robot_state_publisher = get_launch_file('rsp.launch.py', args_robot_state_publisher)

    # ________________ ros2_control Controller Manager ________________
    args_controller_manager = {}
    node_controller_manager = get_launch_file('controller_manager.launch.py', args_controller_manager)

    # ________________ pre-packaged Forward Position Controller. ________________
    # Delay until Controller Manager launches. 
    config_controller_yaml = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            "config",
            "controller.yaml",
        ]
    )
    args_controller = {}
    node_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["my_forward_position_controller", "--param-file", config_controller_yaml],
    )

    # external_cm_nodes = node_controller_manager.get_sub_entities()[0].describe_sub_entities()
    # external_cm_node = next(node for node in external_cm_nodes if node.node_package == 'controller_manager')
    # node_delay_controller = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=external_cm_node,
    #         on_exit=[node_controller],
    #     )
    # )

    # ________________ Joint Broadcast ________________
    # Joint State Broadcast different that Joint State Publisher?
    node_joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["my_joint_state_broadcaster"],
    )

    # node_joint_state_broadcaster_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=[
    #         "joint_state_broadcaster",
    #         "--controller-manager",
    #         "/controller_manager",
    #     ],
    # )

    # node_delay_joint_state_broadcaster_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=node_controller_manager,
    #         on_exit=[node_joint_state_broadcaster_spawner],
    #     )
    # )

    # ________________ RViz ________________
    config_rviz = PathJoinSubstitution(
        [FindPackageShare("base_package"), "config", "view_bot.rviz"]
    )
    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", config_rviz]
    )

    # node_delay_rviz = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=node_delay_joint_state_broadcaster_spawner,
    #         on_start=[node_rviz],
    #     )
    # )

    # ________________ Launch all ________________
    return LaunchDescription([
        node_robot_state_publisher,
        node_controller_manager,
        node_controller,
        # node_delay_controller,
        # node_delay_joint_state_broadcaster_spawner,
        # node_delay_rviz
        node_rviz,
        node_joint_state_broadcaster_spawner
    ])

def get_launch_file(file_name, launch_args):
    package_name = 'base_package'
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