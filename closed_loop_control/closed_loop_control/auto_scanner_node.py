import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import paho.mqtt.client as mqtt
import json
import time

# --- MQTT Configuration ---
MQTT_BROKER = 'localhost' 
MQTT_PORT = 1883
MQTT_TOPIC_CMD = 'control/command'  # Topic สำหรับรับคำสั่งจาก GUI

class AutoScannerNode(Node):
    """
    Auto Scanner Node (Rev 2.0)
    - รองรับ MQTT Command (Start/Stop/Home)
    - มีระบบ Homing อัตโนมัติก่อนเริ่มงาน
    - ป้องกันการส่งมุมเกินขอบเขต (Safety Clamp)
    """
    def __init__(self):
        super().__init__('auto_scanner_node')
        
        # --- Parameters ---
        self.declare_parameter('start_angle_deg', 0.0)
        self.declare_parameter('end_angle_deg', 180.0)
        self.declare_parameter('scan_speed_dps', 200.0)
        self.declare_parameter('pause_duration_s', 0.5)

        self.start_angle = self.get_parameter('start_angle_deg').value
        self.end_angle = self.get_parameter('end_angle_deg').value
        self.scan_speed = self.get_parameter('scan_speed_dps').value
        self.pause_duration = self.get_parameter('pause_duration_s').value

        # --- State Machine Variables ---
        # States: "IDLE", "HOMING", "SCAN_CW", "PAUSE_CW", "SCAN_CCW", "PAUSE_CCW"
        self.state = "IDLE"
        self.is_scanning_active = False # ตัวแปรคุมว่าเราอยู่ในโหมด Auto Run หรือไม่

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

        # รอรับสัญญาณว่ามอเตอร์หมุนถึงที่แล้ว
        self.sub_motion_reached = self.create_subscription(
            Float32,
            '/motion_reached',
            self._motion_reached_callback,
            10)

        self.get_logger().info(f"Auto Scanner Ready. Range: [{self.start_angle}, {self.end_angle}]")

    # ---------------------------------------------------------
    # MQTT Handlers
    # ---------------------------------------------------------
    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            client.subscribe(MQTT_TOPIC_CMD)

    def on_mqtt_message(self, client, userdata, msg):
        """รับคำสั่ง JSON: {'target': 'motor', 'action': 'start'}"""
        try:
            payload = json.loads(msg.payload.decode())
            target = payload.get('target')
            action = payload.get('action')

            if target == 'motor':
                if action == 'start':
                    self.get_logger().info("MQTT CMD: START SEQUENCE")
                    self.start_sequence()
                elif action == 'stop':
                    self.get_logger().info("MQTT CMD: STOP")
                    self.stop_sequence()
                elif action == 'home':
                    self.get_logger().info("MQTT CMD: GO HOME")
                    self.go_home_only()
                    
        except Exception as e:
            self.get_logger().error(f"MQTT Payload Error: {e}")

    # ---------------------------------------------------------
    # Command Logic
    # ---------------------------------------------------------
    def start_sequence(self):
        """เริ่มทำงาน: ต้อง Homing ก่อนเสมอ"""
        if self.state == "IDLE":
            self.is_scanning_active = True
            self.state = "HOMING"
            self.get_logger().info(">>> Sequence Started. Homing first...")
            
            # ส่งคำสั่งให้วิ่งไปที่จุดเริ่มต้น (0 องศา)
            self._send_movement_cmd(self.start_angle, speed=self.scan_speed)

    def stop_sequence(self):
        """หยุดการทำงานทันที"""
        self.is_scanning_active = False
        self.state = "IDLE"
        # สามารถเพิ่มคำสั่งหยุดมอเตอร์ฉุกเฉินได้ถ้า controller_node รองรับ
        # self._send_angle_command(current_angle) # หยุดที่จุดปัจจุบัน (Optional)

    def go_home_only(self):
        """สั่งกลับบ้านเฉยๆ ไม่รันต่อ"""
        self.is_scanning_active = False # ไม่เข้า Loop Scan
        self.state = "HOMING"
        self._send_movement_cmd(self.start_angle, speed=self.scan_speed)

    # ---------------------------------------------------------
    # Movement Helpers (Safety)
    # ---------------------------------------------------------
    def _clamp_angle(self, angle):
        """(ข้อ 1) ป้องกันการส่งค่ามุมเกินขอบเขต"""
        clamped = max(self.start_angle, min(angle, self.end_angle))
        if clamped != angle:
            self.get_logger().warn(f"Angle {angle} out of bounds! Clamped to {clamped}")
        return clamped

    def _send_movement_cmd(self, angle, speed=None, direction='AUTO'):
        # 1. Safety Clamp
        safe_angle = self._clamp_angle(angle)
        
        # 2. Set Speed
        if speed is not None:
            v_msg = Float32()
            v_msg.data = float(speed)
            self.pub_target_speed.publish(v_msg)

        # 3. Set Direction (Optional if 'AUTO' works well)
        d_msg = String()
        d_msg.data = direction
        self.pub_target_dir.publish(d_msg)

        # 4. Send Target Angle
        msg = Float32()
        msg.data = float(safe_angle)
        self.pub_target_angle.publish(msg)
        self.get_logger().info(f"==> Move to {safe_angle:.2f}° ({self.state})")

    # ---------------------------------------------------------
    # State Machine Callback
    # ---------------------------------------------------------
    def _motion_reached_callback(self, msg):
        """
        ทำงานเมื่อมอเตอร์แจ้งว่า 'ถึงเป้าหมายแล้ว'
        """
        reached_angle = msg.data
        self.get_logger().info(f"[ACK] Reached {reached_angle:.2f}°")

        # ถ้า user สั่ง stop ไปแล้ว ให้จบการทำงาน
        if self.state == "IDLE":
            return

        # State Machine Transition
        if self.state == "HOMING":
            if self.is_scanning_active:
                # Homing เสร็จ -> เริ่ม Scan รอบแรก (ไปที่ End)
                self.get_logger().info("Homing Complete. Starting Scan Loop...")
                self.state = "SCAN_CW"
                self._send_movement_cmd(self.end_angle)
            else:
                # Homing อย่างเดียว -> จบ
                self.get_logger().info("Homing Complete. System Idle.")
                self.state = "IDLE"

        elif self.state == "SCAN_CW":
            # ถึงปลายทาง (180) -> หยุดรอ -> กลับ
            self.state = "PAUSE_CW"
            self.get_logger().info(f"Scan CW Done. Pausing {self.pause_duration}s")
            self.create_timer(self.pause_duration, self._timer_next_step)

        elif self.state == "SCAN_CCW":
            # ถึงต้นทาง (0) -> หยุดรอ -> ไปใหม่
            self.state = "PAUSE_CCW"
            self.get_logger().info(f"Scan CCW Done. Pausing {self.pause_duration}s")
            self.create_timer(self.pause_duration, self._timer_next_step)

    def _timer_next_step(self):
        """Timer สำหรับการหยุดรอ (Pause)"""
        # เคลียร์ Timer ทิ้งก่อน (One-shot)
        if hasattr(self, '_timer') and self._timer:
            self.destroy_timer(self._timer)
        
        # เช็คอีกทีว่ายัง Active อยู่ไหม
        if not self.is_scanning_active:
            self.state = "IDLE"
            return

        if self.state == "PAUSE_CW":
            # พักเสร็จแล้ว -> วิ่งกลับ (CCW)
            self.state = "SCAN_CCW"
            self._send_movement_cmd(self.start_angle)

        elif self.state == "PAUSE_CCW":
            # พักเสร็จแล้ว -> วิ่งไป (CW)
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