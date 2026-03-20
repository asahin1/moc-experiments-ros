from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

import yaml


def create_static_tf_node(context):
    # This resolves the LaunchConfiguration into an actual string path
    config_path = LaunchConfiguration("config_file").perform(context)

    with open(config_path, "r") as f:
        params = yaml.safe_load(f)["static_tf_node"]["ros__parameters"]

    node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            str(params["x"]),
            "--y",
            str(params["y"]),
            "--z",
            str(params["z"]),
            "--yaw",
            str(params["yaw"]),
            "--pitch",
            str(params["pitch"]),
            "--roll",
            str(params["roll"]),
            "--frame-id",
            params["frame_id"],
            "--child-frame-id",
            params["child_frame_id"],
        ],
    )
    return [node]


def generate_launch_description():
    rviz_file_arg = DeclareLaunchArgument("rviz_file")
    rviz_file = LaunchConfiguration("rviz_file")
    config_file = DeclareLaunchArgument("config_file")
    config = LaunchConfiguration("config_file")

    map_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        parameters=[config],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_file],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"autostart": True},
            {"node_names": ["map_server"]},  # List all nodes to manage here
        ],
    )
    return LaunchDescription(
        [
            rviz_file_arg,
            config_file,
            OpaqueFunction(function=create_static_tf_node),
            rviz_node,
            RegisterEventHandler(
                OnProcessStart(
                    target_action=rviz_node,
                    on_start=[map_node],
                )
            ),
            lifecycle_manager,
        ]
    )
