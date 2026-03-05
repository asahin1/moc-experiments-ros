from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import json


def create_mocap_node(context):
    robot_names_list = LaunchConfiguration("robot_names").perform(context)
    config = LaunchConfiguration("config_file")
    try:
        loaded_list = json.loads(robot_names_list)
    except Exception:
        loaded_list = []
    mocap_node = Node(
        package="mocap_sim_tf2_cpp",
        executable="mocap_sim_tf2_cpp_node",
        parameters=[config, {"robot_names": loaded_list}],
    )
    return [mocap_node]


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("mocap_sim_tf2_cpp"), "config", "default.yaml"
    )
    config_file = DeclareLaunchArgument("config_file", default_value=default_config)
    return LaunchDescription(
        [
            config_file,
            OpaqueFunction(function=create_mocap_node),
        ]
    )
