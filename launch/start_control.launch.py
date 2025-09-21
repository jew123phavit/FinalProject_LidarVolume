import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # หาตำแหน่งของไฟล์ config
    config_file = os.path.join(
        get_package_share_directory('closed_loop_control'),
        'config',
        'motor_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='closed_loop_control',
            executable='encoder_node',
            name='encoder_node',
            parameters=[config_file]
        ),
        Node(
            package='closed_loop_control',
            executable='controller_node',
            name='controller_node',
            parameters=[config_file]
        ),
    ])
