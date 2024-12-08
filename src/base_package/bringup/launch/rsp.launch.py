# FROM Articulated Robot Series. Launch file for rviz.
# https://github.com/joshnewans/articubot_one/blob/d5aa5e9bc9039073c0d8fd7fe426e170be79c087/launch/rsp.launch.py

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro

def generate_launch_description():

    # Get launch argument values
    mock_mode       = LaunchConfiguration('mock_mode')
    sim_mode        = LaunchConfiguration('sim_mode')

    # Declare Launch Args to be passed from parent launch file or command line
    # These defaults are set twice, once here and once in robot.urdf.xacro
    mock_mode_launch_arg = DeclareLaunchArgument(
        'mock_mode',
        default_value='false',
        description='Use the mock hardware interface (mock_components/GenericSystem), instead of Gazebo or the real robot'
    )
    sim_mode_launch_arg = DeclareLaunchArgument(
        'sim_mode',
        default_value='false',
        description='Use the Gazebo simulator and sim_time. Note, need to manually change use_sim_time in controller YAML config file'
    )

    # Generate URDF file from XACRO file
    pkg_path = os.path.join(get_package_share_directory('base_package'))
    xacro_file = os.path.join(pkg_path,'models','robot.urdf.xacro')
    # robot_description_config = xacro.process_file(xacro_file).toxml()
    robot_description_config = Command(['xacro ', xacro_file, ' use_mode_mock_hardware:=', mock_mode, ' use_mode_gazebo:=', sim_mode])

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': sim_mode}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen', # output='both'
        parameters=[params]
    )

    # joint_state_publisher_gui_node = Node(
    #     package = 'joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui'
    #     # condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    # )

    # Launch!
    return LaunchDescription([
        mock_mode_launch_arg,
        sim_mode_launch_arg,
        node_robot_state_publisher
    ])