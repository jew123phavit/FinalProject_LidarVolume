import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg = 'closed_loop_control'
    cfg = os.path.join(get_package_share_directory(pkg), 'config', 'motor_config.yaml')

    return LaunchDescription([
        Node(
            package=pkg, executable='encoder_node', name='encoder_node',
            parameters=[cfg], output='screen',
        ),
        Node(
            package=pkg, executable='controller_node', name='controller_node',
            parameters=[cfg], output='screen',
        ),
    ])