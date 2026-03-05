import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('mocap4r2_tf2'),
        'config',
        'tf2_params.yaml'
        )
        
    node=Node(
        package='mocap4r2_tf2',
        executable='tf2_broadcaster',
        name='tf2_broadcaster',
        parameters = [config]
    )
    ld.add_action(node)
    return ld
