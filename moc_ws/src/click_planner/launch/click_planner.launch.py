from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import json


def create_planner_node(context):
    robot_names_list = LaunchConfiguration("robot_names").perform(context)
    config = LaunchConfiguration("config_file")
    print(robot_names_list)
    try:
        loaded_list = json.loads(robot_names_list)
    except Exception:
        loaded_list = []
    print(loaded_list)
    planner_node = Node(
        package="click_planner",
        executable="click_planner_node",
        parameters=[config, {"robot_names": loaded_list}],
    )
    return [planner_node]


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("click_planner"), "config", "default.yaml"
    )
    config_file = DeclareLaunchArgument("config_file", default_value=default_config)
    robot_names_arg = DeclareLaunchArgument("robot_names")
    return LaunchDescription(
        [
            config_file,
            robot_names_arg,
            OpaqueFunction(function=create_planner_node),
        ]
    )
