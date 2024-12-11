
import os
import xacro
from ament_index_python.packages import get_package_share_directory



from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    package_name = 'base_package'

    pkg_path = os.path.join(get_package_share_directory('base_package'))
    xacro_file = os.path.join(pkg_path,'models','robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml()}

    controller_config_yaml = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            "config",
            # "controller.yaml"
            "ik_controller.yaml",
        ]
    )

    node_controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[controller_config_yaml, params]
    )

    return LaunchDescription([node_controller_manager])