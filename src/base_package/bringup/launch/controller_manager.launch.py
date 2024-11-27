from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    package_name = 'base_package'

    controller_config_yaml = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            "config",
            "controller.yaml",
        ]
    )

    node_controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config_yaml]
    )

    return LaunchDescription([node_controller_manager])