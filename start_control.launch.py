import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # --- Launch Arguments ---
    pkg_name = 'closed_loop_control'

    default_config = os.path.join(
        get_package_share_directory(pkg_name), 'config', 'motor_config.yaml'
    )

    config_arg = DeclareLaunchArgument(
        'config_file', default_value=default_config,
        description='Path to motor_config.yaml'
    )

    # Controller tuning / options
    direction_mode_arg = DeclareLaunchArgument(
        'direction_mode', default_value='auto_shortest',
        description='Motion policy: auto_shortest | force_cw | force_ccw'
    )
    kp_arg = DeclareLaunchArgument('kp', default_value='2.0', description='P gain (deg/s per deg)')
    max_speed_arg = DeclareLaunchArgument('max_speed_dps', default_value='720.0', description='Max speed (deg/s)')
    min_speed_arg = DeclareLaunchArgument('min_speed_dps', default_value='20.0', description='Min speed (deg/s)')

    # Angle offsets (deg)
    offset_ctrl_arg = DeclareLaunchArgument('offset_deg_ctrl', default_value='0.0', description='Controller offset (deg)')
    offset_enc_arg = DeclareLaunchArgument('offset_deg_enc', default_value='0.0', description='Encoder fine offset (deg)')

    # Optional logging level
    log_level_arg = DeclareLaunchArgument('log_level', default_value='info', description='ROS log level')

    # --- Nodes ---
    config = LaunchConfiguration('config_file')

    encoder_node = Node(
        package=pkg_name,
        executable='encoder_node',
        name='encoder_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            config,
            {
                # Fine offset (post-sensor compensation)
                'offset_deg': ParameterValue(LaunchConfiguration('offset_deg_enc'), value_type=float),
            },
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    controller_node = Node(
        package=pkg_name,
        executable='controller_node',
        name='controller_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            config,
            {
                'direction_mode': LaunchConfiguration('direction_mode'),
                'kp': ParameterValue(LaunchConfiguration('kp'), value_type=float),
                'max_speed_dps': ParameterValue(LaunchConfiguration('max_speed_dps'), value_type=float),
                'min_speed_dps': ParameterValue(LaunchConfiguration('min_speed_dps'), value_type=float),
                'offset_deg': ParameterValue(LaunchConfiguration('offset_deg_ctrl'), value_type=float),
            },
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    return LaunchDescription([
        config_arg,
        direction_mode_arg,
        kp_arg,
        max_speed_arg,
        min_speed_arg,
        offset_ctrl_arg,
        offset_enc_arg,
        log_level_arg,
        encoder_node,
        controller_node,
    ])
