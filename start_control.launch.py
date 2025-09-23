import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    ฟังก์ชันนี้จะถูกเรียกโดย `ros2 launch` เพื่อสร้างคำอธิบายของสิ่งที่จะรัน
    """
    
    # --- 1. กำหนดชื่อแพ็คเกจของเรา ---
    # เราควรเก็บชื่อแพ็คเกจไว้ในตัวแปรเพื่อให้แก้ไขง่ายในอนาคต
    package_name = 'closed_loop_control'

    # --- 2. สร้าง Path ไปยังไฟล์ Configuration (YAML) ---
    # get_package_share_directory(package_name) จะคืนค่า Path ไปยังโฟลเดอร์ 'install/closed_loop_control/share/closed_loop_control'
    # จากนั้นเราใช้ os.path.join เพื่อสร้าง Path ที่สมบูรณ์ไปยังไฟล์ YAML ของเรา
    config_file_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'motor_config.yaml'
    )

    # --- 3. สร้าง LaunchDescription ซึ่งเป็นเหมือนรายการของสิ่งที่จะรัน ---
    return LaunchDescription([
        
        # --- 4. นิยาม Node ตัวที่หนึ่ง: Encoder Node ---
        Node(
            package=package_name,          # ชื่อแพ็คเกจที่ Node นี้อยู่
            executable='encoder_node',     # ชื่อ executable ที่กำหนดใน setup.py
            name='encoder_node',           # ชื่อของ Node ตอนที่รันในระบบ ROS2
            parameters=[config_file_path], # !! ส่วนสำคัญ: บอกให้ Node นี้โหลดพารามิเตอร์จากไฟล์ที่เราหาไว้
            output='screen',               # แสดง log ของ Node นี้บนหน้าจอ Terminal
        ),

        # --- 5. นิยาม Node ตัวที่สอง: Controller Node ---
        Node(
            package=package_name,
            executable='controller_node',
            name='controller_node',
            parameters=[config_file_path], # Node นี้ก็ใช้ไฟล์ config เดียวกัน
            output='screen',
        ),
    ])
