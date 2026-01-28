# ไฟล์: 3D_Accumulation_mqtt.py
import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
import json
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String

# --- Config ---
MQTT_BROKER = 'localhost' 
MQTT_PORT = 1883
MQTT_TOPIC_POINTS = 'mapping/points_3d'
MQTT_TOPIC_CMD = 'control/command'

class PointPublisherNode(Node):
    def __init__(self):
        super().__init__('point_publisher_node')
        
        # State Variables
        self.current_angle = 0.0
        self.last_angle_time = self.get_clock().now()
        self.is_lidar_enabled = True
        self.current_scan_points = []
        self.last_log_time = self.get_clock().now()

        # --- MQTT ---
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        try:
            self.mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.mqtt_client.loop_start()
            self.get_logger().info("MQTT Connected.")
        except Exception as e:
            self.get_logger().error(f"MQTT Connection Error: {e}")

        # --- ROS2 Subs ---
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Float32, 'current_angle', self._on_current, 10)
        self.pub_target_dir = self.create_publisher(String, '/target_dir', 10) 

        # Timer ส่งข้อมูล (20 Hz)
        self.create_timer(0.05, self.publish_points_callback)
        self.get_logger().info('3D Accumulation Node Ready')

    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            client.subscribe(MQTT_TOPIC_CMD)

    def on_mqtt_message(self, client, userdata, msg):
        try:
            cmd = json.loads(msg.payload.decode())
            target = cmd.get('target')
            action = cmd.get('action')
            
            if target == 'motor':
                msg_str = String()
                msg_str.data = 'STOP' if action == 'stop' else 'AUTO'
                self.pub_target_dir.publish(msg_str)
            elif target == 'lidar':
                self.is_lidar_enabled = (action == 'start')
            elif target == 'system' and action == 'reset':
                 self.current_scan_points = []
                 self.get_logger().info("Reset Data Buffer")
        except Exception as e:
            self.get_logger().error(f"MQTT Cmd Error: {e}")

    def _on_current(self, msg: Float32):
        self.current_angle = msg.data
        self.last_angle_time = self.get_clock().now()

    def scan_callback(self, msg: LaserScan):
        if not self.is_lidar_enabled: return

        # 1. เช็คเวลา (ยอมรับ delay ได้มากขึ้นเป็น 0.5s)
        now = self.get_clock().now()
        time_diff = (now - self.last_angle_time).nanoseconds / 1e9
        if time_diff > 0.5:
            if (now - self.last_log_time).nanoseconds / 1e9 > 2.0:
                self.get_logger().warn(f"Waiting for Encoder... (Lag: {time_diff:.2f}s)")
                self.last_log_time = now
            return 

        # 2. คำนวณ 3D Points (XYZ)
        points_this_scan = []
        angle_rad = np.radians(self.current_angle)
        cos_motor = np.cos(angle_rad)
        sin_motor = np.sin(angle_rad)

        for i, distance in enumerate(msg.ranges):
            # กรองข้อมูลระยะทางที่ผิดพลาด
            if not (msg.range_min < distance < msg.range_max): continue
            if distance > 4.0: continue # ตัดข้อมูลไกลเกิน 4 เมตร

            angle_lidar = msg.angle_min + i * msg.angle_increment
            
            # คำนวณพิกัด XYZ (SI Unit: เมตร)
            # x, y คือระนาบพื้น, z คือความสูงจากพื้น
            r_projected = distance * np.sin(angle_lidar) # ระยะราบ
            x_m = r_projected * cos_motor
            y_m = r_projected * sin_motor
            z_m = distance * np.cos(angle_lidar)

            points_this_scan.append([x_m, y_m, z_m])

        self.current_scan_points = points_this_scan

    def publish_points_callback(self):
        if not self.current_scan_points: return
        
        # ส่งข้อมูลเป็น JSON array ของ [x,y,z]
        payload = {'points_m': self.current_scan_points}
        try:
            self.mqtt_client.publish(MQTT_TOPIC_POINTS, json.dumps(payload), qos=0)
        except Exception as e:
            self.get_logger().error(f"MQTT Publish Error: {e}")
        self.current_scan_points = []

def main(args=None):
    rclpy.init(args=args)
    node = PointPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()