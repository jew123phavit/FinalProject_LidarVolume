import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import paho.mqtt.client as mqtt
import json
import time

# --- MQTT Configuration ---
MQTT_BROKER = 'localhost' 
MQTT_PORT = 1883
MQTT_TOPIC_CMD = 'control/command'

class AutoScannerNode(Node):
    """
    Auto Scanner Node (Rev 3.0 - Advisor Fix)
    - เพิ่ม Home Position (Parking) แยกจาก Start Position
    - Start: Home -> Start -> Scan Loop
    - Stop: Scan Loop -> Home
    """
    def __init__(self):
        super().__init__('auto_scanner_node')
        
        # --- Parameters (ตั้งค่าตามที่คุณวัดมา) ---
        # 1. Home (Parking): 81.0
        # 2. Start (0 deg phys): 167.0
        # 3. End (180 deg phys): 352.0
        self.declare_parameter('home_angle_deg', 90.0)
        self.declare_parameter('start_angle_deg', 270.0)
        self.declare_parameter('end_angle_deg', 180.0)
        self.declare_parameter('scan_speed_dps', 200.0)
        self.declare_parameter('pause_duration_s', 0.5)

        self.home_angle = self.get_parameter('home_angle_deg').value
        self.start_angle = self.get_parameter('start_angle_deg').value
        self.end_angle = self.get_parameter('end_angle_deg').value
        self.scan_speed = self.get_parameter('scan_speed_dps').value
        self.pause_duration = self.get_parameter('pause_duration_s').value

        # --- State Machine Variables ---
        # States: "IDLE", "MOVING_TO_START", "SCAN_CW", "PAUSE_CW", "SCAN_CCW", "PAUSE_CCW", "PARKING"
        self.state = "IDLE"
        self.is_scanning_active = False 

        # --- MQTT Setup ---
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        try:
            self.mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.mqtt_client.loop_start()
            self.get_logger().info("Connected to MQTT Broker.")
        except Exception as e:
            self.get_logger().error(f"MQTT Connection Failed: {e}")

        # --- ROS2 Communication ---
        self.pub_target_angle = self.create_publisher(Float32, '/target_angle', 10)
        self.pub_target_dir = self.create_publisher(String, '/target_dir', 10)
        self.pub_target_speed = self.create_publisher(Float32, '/target_speed', 10)

        self.sub_motion_reached = self.create_subscription(
            Float32,
            '/motion_reached',
            self._motion_reached_callback,
            10)

        self.get_logger().info(f"Auto Scanner Rev 3.0 Ready.")
        self.get_logger().info(f"Config: Home={self.home_angle}, Start={self.start_angle}, End={self.end_angle}")

    # ---------------------------------------------------------
    # MQTT Handlers
    # ---------------------------------------------------------
    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            client.subscribe(MQTT_TOPIC_CMD)

    def on_mqtt_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            target = payload.get('target')
            action = payload.get('action')

            if target == 'motor':
                if action == 'start':
                    self.get_logger().info("MQTT CMD: START SEQUENCE")
                    self.start_sequence()
                elif action == 'stop':
                    self.get_logger().info("MQTT CMD: STOP & PARK")
                    self.stop_sequence()
                elif action == 'home':
                    self.get_logger().info("MQTT CMD: GO HOME ONLY")
                    self.go_home_only()
                    
        except Exception as e:
            self.get_logger().error(f"MQTT Payload Error: {e}")

    # ---------------------------------------------------------
    # Command Logic
    # ---------------------------------------------------------
    def start_sequence(self):
        """เริ่มทำงาน: ไปที่จุด Start (167) ก่อนเริ่ม Scan"""
        if self.state == "IDLE" or self.state == "PARKING":
            self.is_scanning_active = True
            self.state = "MOVING_TO_START"
            self.get_logger().info(f">>> Moving to Start Position ({self.start_angle}°)...")
            self._send_movement_cmd(self.start_angle, speed=self.scan_speed)

    def stop_sequence(self):
        """หยุดการทำงาน: เลิก loop แล้วกลับบ้าน (Home)"""
        self.is_scanning_active = False
        # หมายเหตุ: เราจะไม่เปลี่ยน state ทันทีตรงนี้ 
        # แต่จะให้ Callback (_motion_reached หรือ Timer) เป็นตัวเช็ค flag 'is_scanning_active' แล้วพาไป State PARKING เอง

    def go_home_only(self):
        """สั่งกลับบ้าน (81)"""
        self.is_scanning_active = False
        self.state = "PARKING"
        self._send_movement_cmd(self.home_angle, speed=self.scan_speed)

    # ---------------------------------------------------------
    # Movement Helpers
    # ---------------------------------------------------------
    def _clamp_angle(self, angle):
        """
        ป้องกันการส่งค่ามุมเกินขอบเขต
        อาจารย์แก้ไข: ต้องยอมให้ไปที่ Home (81) ได้ แม้จะอยู่นอกช่วง Start-End (167-352)
        """
        # ถ้าเป้าหมายคือ Home ให้ผ่านได้เลย
        if abs(angle - self.home_angle) < 0.1:
            return angle
            
        # ถ้าไม่ใช่ Home ให้ Clamp อยู่ในช่วง Start-End
        # หมายเหตุ: ใช้ min/max สลับกันเพื่อให้รองรับกรณี Start > End ได้ด้วย (เผื่ออนาคต)
        lower = min(self.start_angle, self.end_angle)
        upper = max(self.start_angle, self.end_angle)
        
        clamped = max(lower, min(angle, upper))
        
        if clamped != angle:
            self.get_logger().warn(f"Angle {angle} out of bounds [{lower},{upper}]! Clamped to {clamped}")
        return clamped

    def _send_movement_cmd(self, angle, speed=None, direction='AUTO'):
        safe_angle = self._clamp_angle(angle)
        
        if speed is not None:
            v_msg = Float32()
            v_msg.data = float(speed)
            self.pub_target_speed.publish(v_msg)

        d_msg = String()
        d_msg.data = direction
        self.pub_target_dir.publish(d_msg)

        msg = Float32()
        msg.data = float(safe_angle)
        self.pub_target_angle.publish(msg)
        self.get_logger().info(f"==> Move to {safe_angle:.2f}° (State: {self.state})")

    # ---------------------------------------------------------
    # State Machine Callback
    # ---------------------------------------------------------
    def _motion_reached_callback(self, msg):
        reached_angle = msg.data
        self.get_logger().info(f"[ACK] Reached {reached_angle:.2f}°")

        # กรณีสั่ง Stop กลางคัน ให้กลับบ้าน (ถ้ายังไม่ได้อยู่ที่บ้าน)
        if not self.is_scanning_active:
            if self.state != "PARKING" and self.state != "IDLE":
                self.get_logger().info("Stop requested. Returning to HOME...")
                self.state = "PARKING"
                self._send_movement_cmd(self.home_angle)
                return
            elif self.state == "PARKING":
                self.get_logger().info("Parked at Home. System IDLE.")
                self.state = "IDLE"
                return

        # State Machine Transition (Normal Operation)
        if self.state == "MOVING_TO_START":
            # ถึงจุด Start (167) แล้ว -> เริ่มสแกนไปที่ End (352)
            self.get_logger().info("At Start Position. Begin Scanning CW...")
            self.state = "SCAN_CW"
            self._send_movement_cmd(self.end_angle)

        elif self.state == "SCAN_CW":
            # ถึง End (352) -> หยุดรอ -> เตรียมกลับ
            self.state = "PAUSE_CW"
            self.get_logger().info(f"Scan CW Done. Pausing {self.pause_duration}s")
            self.create_timer(self.pause_duration, self._timer_next_step)

        elif self.state == "SCAN_CCW":
            # ถึง Start (167) -> หยุดรอ -> เตรียมไปใหม่
            self.state = "PAUSE_CCW"
            self.get_logger().info(f"Scan CCW Done. Pausing {self.pause_duration}s")
            self.create_timer(self.pause_duration, self._timer_next_step)
            
        elif self.state == "PARKING":
            # ถึง Home (81) -> จบ
            self.get_logger().info("Parked at Home. System IDLE.")
            self.state = "IDLE"

    def _timer_next_step(self):
        """Timer สำหรับการหยุดรอ (Pause)"""
        if hasattr(self, '_timer') and self._timer:
            self.destroy_timer(self._timer)
        
        # เช็ค Stop flag ระหว่างรอ Timer
        if not self.is_scanning_active:
            self.state = "PARKING"
            self._send_movement_cmd(self.home_angle)
            return

        if self.state == "PAUSE_CW":
            # พักเสร็จ -> วิ่งกลับไป Start (167) (CCW)
            self.state = "SCAN_CCW"
            self._send_movement_cmd(self.start_angle)

        elif self.state == "PAUSE_CCW":
            # พักเสร็จ -> วิ่งไป End (352) (CW)
            self.state = "SCAN_CW"
            self._send_movement_cmd(self.end_angle)

def main(args=None):
    rclpy.init(args=args)
    node = AutoScannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()