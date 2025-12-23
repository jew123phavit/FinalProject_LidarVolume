import rclpy
from rclpy.node import Node
from rclpy.time import Time
import paho.mqtt.client as mqtt
import json
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String

# --- Config ---
MQTT_BROKER = 'localhost' 
MQTT_PORT = 1883
MQTT_TOPIC_POINTS = 'mapping/points_3d'
MQTT_TOPIC_CMD = 'control/command' # Topic สำหรับรับคำสั่งจาก PC

class PointPublisherNode(Node):

    def __init__(self):
        super().__init__('point_publisher_node')
        
        # State Variables
        self.current_angle = 0.0
        self.last_angle_time = self.get_clock().now() # เก็บเวลาล่าสุดที่มุมอัปเดต
        self.is_lidar_enabled = True # สถานะเปิด/ปิดการส่งข้อมูล
        self.current_scan_points = []

        # --- MQTT ---
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
        self.mqtt_client.loop_start()

        # --- ROS2 Subs/Pubs ---
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Float32, 'current_angle', self._on_current, 10)
        
        # Publisher สำหรับคุม Motor (ส่งต่อคำสั่งไป controller/auto_scanner)
        self.pub_target_dir = self.create_publisher(String, '/target_dir', 10) 

        # Timer ส่งข้อมูล (10 Hz เพื่อความ Real-time)
        self.create_timer(0.1, self.publish_points_callback)
        
        self.get_logger().info('3D Accumulation Node Ready (Timestamp Verified + ROI)')

    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("MQTT Connected.")
            client.subscribe(MQTT_TOPIC_CMD)

    def on_mqtt_message(self, client, userdata, msg):
        """รับคำสั่ง Start/Stop จาก PC"""
        try:
            cmd = json.loads(msg.payload.decode())
            target = cmd.get('target')
            action = cmd.get('action')
            
            if target == 'motor':
                # สั่งหยุด/เดิน มอเตอร์ผ่าน ROS2 Topic
                msg_str = String()
                if action == 'stop':
                    msg_str.data = 'STOP' # ต้องไปแก้ controller ให้รองรับ หรือสั่ง speed 0
                    self.get_logger().info("Command: STOP MOTOR")
                    # หมายเหตุ: ใน auto_scanner อาจต้องเพิ่ม Logic หยุด
                elif action == 'start':
                    msg_str.data = 'AUTO'
                    self.get_logger().info("Command: START MOTOR")
                self.pub_target_dir.publish(msg_str)

            elif target == 'lidar':
                # สั่งหยุด/เริ่ม การประมวลผลข้อมูล
                if action == 'stop':
                    self.is_lidar_enabled = False
                    self.get_logger().info("Command: PAUSE LIDAR DATA")
                elif action == 'start':
                    self.is_lidar_enabled = True
                    self.get_logger().info("Command: RESUME LIDAR DATA")
                    
            elif target == 'system' and action == 'reset':
                 self.current_scan_points = []
                 self.get_logger().info("Command: RESET DATA")

        except Exception as e:
            self.get_logger().error(f"MQTT Cmd Error: {e}")

    def _on_current(self, msg: Float32):
        self.current_angle = msg.data
        self.last_angle_time = self.get_clock().now() # อัปเดตเวลาล่าสุดที่ได้รับมุม

    def scan_callback(self, msg: LaserScan):
        if not self.is_lidar_enabled:
            return

        # --- 2. Timestamp Verification ---
        # เช็คว่าข้อมูลมุม (Encoder) เก่าเกินไปหรือไม่ (เช่น เกิน 0.1 วินาที)
        # ถ้าเก่าเกินไป แสดงว่ามุมอาจจะไม่ซิงค์กับเลเซอร์ -> ทิ้งข้อมูลรอบนี้
        now = self.get_clock().now()
        time_diff = (now - self.last_angle_time).nanoseconds / 1e9
        
        if time_diff > 0.15: # ยอมรับ delay ได้ไม่เกิน 150ms
            # self.get_logger().warn(f"Synchronization Lag: {time_diff:.4f}s (Data Dropped)")
            return 

        # --- 3. ROI Filtering (Servo Angle) ---
        # กรองเฉพาะช่วงมุมของมอเตอร์ที่ต้องการ (90-180 องศา)
        # สมมติว่า 0 คือนอน, 90 คือตั้งฉาก, 180 คืออีกฝั่ง
        if not (90 <= self.current_angle <= 180):
             return

        points_this_scan = []
        # คำนวณ Sin/Cos ไว้ก่อนเพื่อความเร็ว
        cos_motor = np.cos(np.radians(self.current_angle))
        sin_motor = np.sin(np.radians(self.current_angle))

        for i, distance in enumerate(msg.ranges):
            if not (msg.range_min < distance < msg.range_max):
                continue

            # กรองระยะ (ROI Distance) ตัดสิ่งที่ไกลเกินถังออก (เช่น > 2 เมตร)
            if distance > 2.0: 
                continue

            angle_lidar = msg.angle_min + i * msg.angle_increment
            
            # คำนวณพิกัด (ปรับตามการติดตั้งจริง)
            # สมมติ: แกนหมุนคือ Y, LiDAR กวาดในระนาบ XZ
            x = distance * np.sin(angle_lidar) 
            z_lidar = distance * np.cos(angle_lidar)
            
            # Rotate ตามมุมมอเตอร์
            # x_final = x
            # y_final = z_lidar * sin_motor
            # z_final = z_lidar * cos_motor
            
            # สูตรของคุณ (ตรวจสอบทิศทางอีกทีนะครับ)
            x_m = distance * np.sin(angle_lidar) * cos_motor
            y_m = distance * np.sin(angle_lidar) * sin_motor
            z_m = distance * np.cos(angle_lidar)

            points_this_scan.append([x_m, y_m, z_m])

        self.current_scan_points = points_this_scan

    def publish_points_callback(self):
        if not self.current_scan_points:
            return

        payload = {'points_m': self.current_scan_points}
        try:
            # ใช้ qos=0 เพื่อความเร็วสูงสุด
            self.mqtt_client.publish(MQTT_TOPIC_POINTS, json.dumps(payload), qos=0)
        except Exception as e:
            pass
        
        self.current_scan_points = []

def main(args=None):
    rclpy.init(args=args)
    node = PointPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()