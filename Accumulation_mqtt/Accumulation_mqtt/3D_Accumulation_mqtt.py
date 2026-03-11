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
        if not self.is_lidar_enabled:
            return

        # 1. เช็คเวลา (Timestamp Verification)
        now = self.get_clock().now()
        time_diff = (now - self.last_angle_time).nanoseconds / 1e9
        
        if time_diff > 0.5: 
            if (now - self.last_log_time).nanoseconds / 1e9 > 2.0:
                self.get_logger().warn(f"Waiting for Encoder... (Lag: {time_diff:.2f}s)")
                self.last_log_time = now
            return 

        # ===================================================
        # --- 3D Cartesian Mapping (Matrix Transformation) ---
        # ===================================================
        
        ranges = np.array(msg.ranges)

        # สร้าง Array มุมของ LiDAR ทุกจุด
        angles_raw = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        # Normalize มุมให้อยู่ช่วง 0 - 2π (ป้องกันค่าติดลบ)
        angles_norm = angles_raw % (2 * np.pi)

        # --- กรองมุม LiDAR: เก็บเฉพาะ 90° - 270° เท่านั้น ---
        # (ตัด Noise ด้านบนถังและบริเวณนอกถังทิ้ง)
        angle_mask = (angles_norm >= np.radians(90)) & (angles_norm <= np.radians(270))

        # --- กรองระยะ: ต้องอยู่ในช่วงที่ valid และไม่เกิน 1.0 เมตร ---
        # (ถังมี r=22cm เท่านั้น ไม่จำเป็นต้องมองไกลกว่านี้)
        distance_mask = (ranges > msg.range_min) & (ranges < msg.range_max) & (ranges <= 1.0)

        # รวม Mask ทั้งสองเข้าด้วยกัน
        valid_mask = angle_mask & distance_mask
        
        if not np.any(valid_mask):
            return
            
        d = ranges[valid_mask]
        indices = np.where(valid_mask)[0]
        
        # ---------------------------------------------------
        # แกน Y (Pitch): แนวการหมุนของ LiDAR
        # ---------------------------------------------------
        # lidar_offset = 180° เพื่อให้จุด 0° ชี้ลงพื้นพอดี (คงค่าเดิมของคุณ)
        lidar_offset = np.radians(180.0) 
        theta_l = msg.angle_min + indices * msg.angle_increment + lidar_offset

        # สร้างพิกัด Local ของ Lidar (เมื่อหมุนรอบแกน Y ระนาบที่ได้คือระนาบ XZ)
        x_local = d * np.sin(theta_l)    # กวาดไปตามแนวแกน X
        y_local = np.zeros_like(d)       # แกน Y เป็น 0 เพราะเป็นจุดหมุนของ Lidar
        z_local = -d * np.cos(theta_l)   # ติดลบเพื่อให้ชี้ลงพื้น (Z เป็นความสูง)

        P_local = np.vstack((x_local, y_local, z_local))

        # ---------------------------------------------------
        # แกน X (Roll): แนวการหมุนของ Stepper Motor
        # ---------------------------------------------------
        # ให้ 180 องศาคือจุดศูนย์กลาง (ชี้ลงพื้นตรงๆ) จึงต้องลบ 180 ออก
        theta_m = np.radians(-(self.current_angle - 180.0))
        
        cos_m = np.cos(theta_m)
        sin_m = np.sin(theta_m)

        # Matrix สำหรับหมุนรอบแกน X (Roll Matrix)
        R_motor_x = np.array([
            [1,     0,      0],
            [0, cos_m, -sin_m],
            [0, sin_m,  cos_m]
        ])

        # ---------------------------------------------------
        # คำนวณพิกัด 3D จริง (Matrix Multiplication)
        # ---------------------------------------------------
        # นำ Matrix มอเตอร์ คูณกับ พิกัด Local ของ Lidar
        P_global = R_motor_x @ P_local

        # ---------------------------------------------------
        # (Optional) ชดเชยจุดหมุน (Translation Offset)
        # ถ้าแกนมอเตอร์ กับ หัว Lidar มีระยะห่างกัน (เยื้องกัน) 
        # ให้ใส่ระยะห่าง (หน่วยเมตร) ที่นี่ เพื่อลบความโค้งที่เหลืออยู่ให้แบนสนิท
        # T_offset = np.array([[0.0], [0.0], [0.05]]) # เช่น Lidar ต่ำกว่าแกนมอเตอร์ 5 cm
        # P_global = P_global + T_offset
        # ---------------------------------------------------

        # แปลงข้อมูลกลับเป็น List เพื่อส่งผ่าน MQTT
        self.current_scan_points = P_global.T.tolist()

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