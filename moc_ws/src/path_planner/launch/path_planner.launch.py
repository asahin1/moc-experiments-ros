from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os
import json


def create_node(context):
    robot_names_list = LaunchConfiguration("robot_names").perform(context)
    config = LaunchConfiguration("config_file")
    try:
        loaded_list = json.loads(robot_names_list)
    except Exception:
        loaded_list = []
    node = Node(
        package="path_planner",
        executable="path_planner_node",
        parameters=[config, {"robot_names": loaded_list}],
    )
    return [node]


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("path_planner"), "config", "default.yaml"
    )
    config_file = DeclareLaunchArgument("config_file", default_value=default_config)
    robot_names_arg = DeclareLaunchArgument("robot_names")
    return LaunchDescription(
        [
            config_file,
            robot_names_arg,
            OpaqueFunction(function=create_node),
        ]
    )
