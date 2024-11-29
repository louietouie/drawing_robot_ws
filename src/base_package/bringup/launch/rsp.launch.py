# FROM Articulated Robot Series. Launch file for rviz.
# https://github.com/joshnewans/articubot_one/blob/d5aa5e9bc9039073c0d8fd7fe426e170be79c087/launch/rsp.launch.py

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro

def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Generate URDF file from XACRO file
    pkg_path = os.path.join(get_package_share_directory('base_package'))
    xacro_file = os.path.join(pkg_path,'models','robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
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
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        node_robot_state_publisher
    ])