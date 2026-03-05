from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os
import json


def create_node(context):
    first_end_robot_names_list = LaunchConfiguration("first_end_robot_names").perform(
        context
    )
    last_end_robot_names_list = LaunchConfiguration("last_end_robot_names").perform(
        context
    )
    config = LaunchConfiguration("config_file")
    try:
        first_loaded_list = json.loads(first_end_robot_names_list)
        last_loaded_list = json.loads(last_end_robot_names_list)
    except Exception:
        first_loaded_list = []
        last_loaded_list = []
    node = Node(
        package="cable_action_planner",
        executable="cable_action_planner_node",
        parameters=[
            config,
            {
                "first_end_robot_names": first_loaded_list,
                "last_end_robot_names": last_loaded_list,
            },
        ],
    )
    return [node]


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("cable_action_planner"), "config", "default.yaml"
    )
    config_file = DeclareLaunchArgument("config_file", default_value=default_config)
    first_end_robot_names_arg = DeclareLaunchArgument("first_end_robot_names")
    last_end_robot_names_arg = DeclareLaunchArgument("last_end_robot_names")
    return LaunchDescription(
        [
            config_file,
            first_end_robot_names_arg,
            last_end_robot_names_arg,
            OpaqueFunction(function=create_node),
        ]
    )
