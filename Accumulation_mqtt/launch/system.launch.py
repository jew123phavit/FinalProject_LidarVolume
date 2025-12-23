import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    # --- 1. SLLIDAR Launch (Lidar Driver) ---
    # เรียกใช้ Launch file ของ sllidar_ros2 ที่มีอยู่แล้ว
    sllidar_dir = get_package_share_directory('sllidar_ros2')
    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sllidar_dir, 'launch', 'sllidar_a1_launch.py')
        ),
        # ถ้าต้องการแก้พอร์ตหรือเฟรม สามารถใส่ launch_arguments ได้ที่นี่
        # launch_arguments={'serial_port': '/dev/ttyUSB0', 'frame_id': 'laser'}.items()
    )

    # --- 2. Closed Loop Control Nodes (Motor & Encoder) ---
    # Node อ่านค่า Encoder (I2C)
    encoder_node = Node(
        package='closed_loop_control',
        executable='encoder_node', # ชื่อที่ตั้งใน setup.py (entry_points)
        name='encoder_node',
        output='screen'
    )

    # Node ควบคุมมอเตอร์ (GPIO + PID)
    controller_node = Node(
        package='closed_loop_control',
        executable='controller_node',
        name='controller_node',
        output='screen'
    )

    # Node สั่งสแกนอัตโนมัติ (State Machine)
    # ใส่ TimerAction เพื่อหน่วงเวลาให้ controller พร้อมก่อน 2 วินาที
    auto_scanner_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='closed_loop_control',
                executable='auto_scanner_node',
                name='auto_scanner_node',
                output='screen'
            )
        ]
    )

    # --- 3. Accumulation MQTT (Main Process) ---
    # Node คำนวณพิกัดและส่ง MQTT
    accumulation_node = Node(
        package='Accumulation_mqtt',
        executable='3D_Accumulation_mqtt', # ชื่อที่ตั้งใน setup.py
        name='point_publisher_node',
        output='screen'
    )

    # --- Return Launch Description ---
    return LaunchDescription([
        sllidar_launch,
        encoder_node,
        controller_node,
        auto_scanner_node,
        accumulation_node
    ])