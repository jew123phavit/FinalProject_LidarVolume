import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
import json
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String # สำหรับรับคำสั่ง Reset

# --- การตั้งค่า MQTT ---
MQTT_BROKER = '100.117.126.91' # IP ของ RPi (Tailscale)
MQTT_PORT = 1883
MQTT_TOPIC_POINTS = 'mapping/points_2d' # Topic ใหม่สำหรับส่งข้อมูลพิกัด
MQTT_TOPIC_RESET = 'mapping/reset'      # Topic สำหรับรับคำสั่ง Reset จาก GUI

class PointPublisherNode(Node):

    def __init__(self):
        super().__init__('point_publisher_node')
        self.get_logger().info('Point Publisher Node has been started.')

        # 1. ไม่สร้างแผนที่เป็น Grid แต่จะสะสมพิกัด (x,y) ในหน่วยเมตร
        self.accumulated_points = []

        # 2. ตั้งค่า MQTT Client
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message # เพิ่ม callback เพื่อรับ message
        self.mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
        self.mqtt_client.loop_start()

        # 3. ตั้งค่า ROS2 Subscription สำหรับ LiDAR
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # 4. ตั้งเวลาส่งข้อมูลทุกๆ 1 วินาที
        self.create_timer(1.0, self.publish_points_callback)
        self.get_logger().info(f"Publishing 2D points to MQTT topic '{MQTT_TOPIC_POINTS}'")

    def on_mqtt_connect(self, client, userdata, flags, rc):
        """Callback เมื่อเชื่อมต่อ MQTT สำเร็จ"""
        if rc == 0:
            self.get_logger().info("Connected to MQTT Broker.")
            # ดักฟังคำสั่ง Reset ที่ส่งมาจาก GUI
            client.subscribe(MQTT_TOPIC_RESET)
            self.get_logger().info(f"Subscribed to MQTT topic '{MQTT_TOPIC_RESET}'")
        else:
            self.get_logger().error(f"Failed to connect to MQTT Broker, return code {rc}")

    def on_mqtt_message(self, client, userdata, msg):
        """Callback เมื่อได้รับ Message จาก Broker (เช่นคำสั่ง Reset)"""
        if msg.topic == MQTT_TOPIC_RESET:
            self.get_logger().info("<<<<< Reset command received from GUI. Clearing accumulated points. >>>>>")
            self.accumulated_points = []

    def scan_callback(self, msg: LaserScan):
        """รับข้อมูลดิบจาก LiDAR แล้วแปลงเป็นพิกัด (x, y) ในหน่วยเมตร"""
        points_this_scan = []
        for i, distance in enumerate(msg.ranges):
            # กรองข้อมูลระยะทางที่ผิดพลาดออก
            if not (msg.range_min < distance < msg.range_max):
                continue

            angle = msg.angle_min + i * msg.angle_increment

            # แปลง Polar (angle, distance) เป็น Cartesian (x, y) ในหน่วยเมตร (SI units)
            # ตามมาตรฐาน ROS: แกน X คือด้านหน้า, แกน Y คือด้านซ้าย
            x_m = distance * np.cos(angle)
            y_m = distance * np.sin(angle)
            points_this_scan.append([x_m, y_m])

        # เพิ่มพิกัดที่ได้ใหม่เข้าไปใน list ที่สะสมไว้
        self.accumulated_points.extend(points_this_scan)

    def publish_points_callback(self):
        """ส่งข้อมูลพิกัดทั้งหมดที่สะสมไว้ออกไปทาง MQTT"""
        if not self.accumulated_points:
            return

        # Payload ตอนนี้จะมีแค่รายการของพิกัดในหน่วยเมตร
        payload = {
            'points_m': self.accumulated_points
        }
        
        try:
            payload_json = json.dumps(payload)
            self.mqtt_client.publish(MQTT_TOPIC_POINTS, payload_json)
        except Exception as e:
            self.get_logger().error(f"Failed to publish points: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PointPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()