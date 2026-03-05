from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("robot_visualizer"), "config", "default.yaml"
    )
    config_file = DeclareLaunchArgument("config_file", default_value=default_config)
    config = LaunchConfiguration("config_file")
    robot_config_file = DeclareLaunchArgument("robot_config_file")
    robot_config = LaunchConfiguration("robot_config_file")
    node = Node(
        package="robot_visualizer",
        executable="robot_visualizer",
        parameters=[config, robot_config],
    )
    return LaunchDescription(
        [
            config_file,
            robot_config_file,
            node,
        ]
    )
