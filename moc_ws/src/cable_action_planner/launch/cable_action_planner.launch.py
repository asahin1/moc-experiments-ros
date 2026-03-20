from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os
import json


def create_node(context):
    rear_end_robot_names_list = LaunchConfiguration("rear_end_robot_names").perform(
        context
    )
    front_end_robot_names_list = LaunchConfiguration("front_end_robot_names").perform(
        context
    )
    config = LaunchConfiguration("config_file")
    try:
        rear_loaded_list = json.loads(rear_end_robot_names_list)
        front_loaded_list = json.loads(front_end_robot_names_list)
    except Exception:
        rear_loaded_list = []
        front_loaded_list = []
    node = Node(
        package="cable_action_planner",
        executable="cable_action_planner_node",
        parameters=[
            config,
            {
                "rear_end_robot_names": rear_loaded_list,
                "front_end_robot_names": front_loaded_list,
            },
        ],
    )
    return [node]


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("cable_action_planner"), "config", "default.yaml"
    )
    config_file = DeclareLaunchArgument("config_file", default_value=default_config)
    rear_end_robot_names_arg = DeclareLaunchArgument("rear_end_robot_names")
    front_end_robot_names_arg = DeclareLaunchArgument("front_end_robot_names")
    return LaunchDescription(
        [
            config_file,
            rear_end_robot_names_arg,
            front_end_robot_names_arg,
            OpaqueFunction(function=create_node),
        ]
    )
