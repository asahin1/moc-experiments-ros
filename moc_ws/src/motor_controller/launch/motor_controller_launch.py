from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("motor_controller"), "config", "default.yaml"
    )
    config_file = DeclareLaunchArgument("config_file", default_value=default_config)
    config = LaunchConfiguration("config_file")
    node = Node(
        package="motor_controller",
        executable="motor_controller",
        parameters=[config],
    )
    return LaunchDescription(
        [
            config_file,
            node,
        ]
    )
