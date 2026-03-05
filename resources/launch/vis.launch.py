from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node


def generate_launch_description():
    rviz_file_arg = DeclareLaunchArgument("rviz_file")
    rviz_file = LaunchConfiguration("rviz_file")
    config_file = DeclareLaunchArgument("config_file")
    config = LaunchConfiguration("config_file")
    map_world_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "-3.6",
            "--y",
            "-2.1",
            "--z",
            "0",
            "--yaw",
            "0",
            "--pitch",
            "0",
            "--roll",
            "0",
            "--frame-id",
            "world",
            "--child-frame-id",
            "map",
        ],
    )
    map_node = Node(
        package="nav2_map_server",
        executable="map_server",
        parameters=[config],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_file],
    )
    lifecycle_bringup = Node(
        package="nav2_util",
        executable="lifecycle_bringup",
        arguments=["map_server"],
    )
    return LaunchDescription(
        [
            rviz_file_arg,
            config_file,
            map_world_tf_node,
            rviz_node,
            RegisterEventHandler(
                OnProcessStart(
                    target_action=rviz_node,
                    on_start=[map_node],
                )
            ),
            RegisterEventHandler(
                OnProcessStart(
                    target_action=map_node,
                    on_start=[lifecycle_bringup],
                )
            ),
        ]
    )
