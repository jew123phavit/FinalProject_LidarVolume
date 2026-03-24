# ไฟล์: 3D_Accumulation_mqtt.py
import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
import json
import numpy as np
from collections import deque
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String

# --- Config ---
MQTT_BROKER = 'localhost'
MQTT_PORT = 1883
MQTT_TOPIC_POINTS = 'mapping/points_3d'
MQTT_TOPIC_CMD    = 'control/command'

# ── Point Buffer Limits ──────────────────────────────────────────────────────
PARKING_MAX_POINTS = 2000
SCAN_MAX_POINTS    = 150000

# ════════════════════════════════════════════════════════════════════════════
# 🔧 CALIBRATION PARAMETERS — ปรับค่าตรงนี้เพื่อแก้ปัญหา
# ════════════════════════════════════════════════════════════════════════════

# [1] แก้ปัญหาจุดไม่ต่อเนื่องระหว่าง CW/CCW (Discontinuity)
#     ถ้า CW อยู่สูงกว่า CCW → เพิ่มค่า (บวก)
#     ถ้า CCW อยู่สูงกว่า CW → ลดค่า (ลบ)
#     ปรับทีละ 0.5 องศา แล้วสังเกตผล
MOTOR_ANGLE_OFFSET_DEG = -3.0

# [2] เก็บข้อมูลเฉพาะขา CW เพื่อป้องกัน Double Layer
#     True  = เก็บแค่ SCAN_CW (ช้าลงครึ่ง แต่ไม่มี Double Layer)
#     False = เก็บทั้ง CW+CCW (เร็วขึ้น แต่อาจมี Double Layer)
UNIDIRECTIONAL_SCAN = True

# [3] แก้ปัญหาทรงกรวย (Cone Shape)
#     วัดระยะจากหัว LiDAR ถึงแกนมอเตอร์ แล้วใส่ค่านี้ (หน่วยเมตร)
#     เช่น หัว LiDAR ต่ำกว่าแกนมอเตอร์ 3 cm → T_OFFSET_Z = 0.03
#     เช่น หัว LiDAR หน้า/หลังแกนมอเตอร์ 2 cm → T_OFFSET_Y = ±0.02
T_OFFSET_X = 0.0     # เยื้องซ้าย-ขวา (เมตร)
T_OFFSET_Y = 0.0     # เยื้องหน้า-หลัง (เมตร)
T_OFFSET_Z = 0.0175 # เยื้องบน-ล่าง (เมตร)
APPLY_T_OFFSET = True  # เปิด True เมื่อวัดค่าออฟเซ็ตได้แล้ว

# ════════════════════════════════════════════════════════════════════════════

# ── States ───────────────────────────────────────────────────────────────────
STATES_LIDAR_ALLOWED = {'IDLE', 'PARKING', 'SCAN_CW', 'SCAN_CCW', 'PAUSE_CW', 'PAUSE_CCW'}
STATES_SCAN_ACTIVE   = {'SCAN_CW', 'SCAN_CCW', 'PAUSE_CW', 'PAUSE_CCW'}
STATES_PARKING_VIEW  = {'IDLE', 'PARKING'}

# เลือก state ที่จะเก็บข้อมูลขณะ scan ตาม UNIDIRECTIONAL_SCAN flag
if UNIDIRECTIONAL_SCAN:
    STATES_COLLECT_SCAN = {'SCAN_CW'}          # เก็บเฉพาะขา CW เท่านั้น
else:
    STATES_COLLECT_SCAN = {'SCAN_CW', 'SCAN_CCW'}   # เก็บทั้ง 2 ขา


class PointPublisherNode(Node):
    def __init__(self):
        super().__init__('point_publisher_node')

        # ── Motor State ──────────────────────────────────────────────────────
        self.motor_state = 'IDLE'

        # ── Point Buffers ────────────────────────────────────────────────────
        self.parking_buffer: deque = deque(maxlen=PARKING_MAX_POINTS)
        self.scan_buffer: list = []
        self.scan_complete = False
        self._last_sent_scan_idx = 0   # index จุดสุดท้ายที่ส่งไปแล้ว (delta publish)

        # ── LiDAR / Encoder State ────────────────────────────────────────────
        self.current_angle  = 0.0
        self.last_angle_time = self.get_clock().now()
        self.is_lidar_enabled = True
        self.last_log_time   = self.get_clock().now()

        # ── Log calibration settings on startup ─────────────────────────────
        self.get_logger().info("=" * 55)
        self.get_logger().info(f"  MOTOR_ANGLE_OFFSET = {MOTOR_ANGLE_OFFSET_DEG:.2f}°")
        self.get_logger().info(f"  UNIDIRECTIONAL     = {UNIDIRECTIONAL_SCAN}")
        self.get_logger().info(f"  COLLECT STATES     = {STATES_COLLECT_SCAN}")
        self.get_logger().info(f"  T_OFFSET (xyz)     = [{T_OFFSET_X}, {T_OFFSET_Y}, {T_OFFSET_Z}] m")
        self.get_logger().info("=" * 55)

        # ── MQTT ─────────────────────────────────────────────────────────────
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        try:
            self.mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.mqtt_client.loop_start()
            self.get_logger().info("MQTT Connected.")
        except Exception as e:
            self.get_logger().error(f"MQTT Connection Error: {e}")

        # ── ROS2 Subscriptions ───────────────────────────────────────────────
        self.create_subscription(LaserScan, '/scan',          self.scan_callback,     10)
        self.create_subscription(Float32,   'current_angle',  self._on_current,       10)
        self.create_subscription(String,    '/scanner/state', self._on_scanner_state, 10)
        self.pub_target_dir = self.create_publisher(String, '/target_dir', 10)

        # ── Timer ส่งข้อมูล (20 Hz) ──────────────────────────────────────────
        self.create_timer(0.05, self.publish_points_callback)
        self.get_logger().info('3D Accumulation Node Ready')

    # ════════════════════════════════════════════════════════════════════════
    # STATE CALLBACKS
    # ════════════════════════════════════════════════════════════════════════

    def _on_scanner_state(self, msg: String):
        new_state = msg.data.upper()
        if new_state == self.motor_state:
            return

        prev_state = self.motor_state
        self.motor_state = new_state
        self.get_logger().info(f"[State] {prev_state} → {new_state}")

        # ── HOMING: เริ่ม sequence ใหม่ → ล้างทุก buffer ──────────────────
        if new_state == 'HOMING':
            self.scan_buffer.clear()
            self.parking_buffer.clear()
            self.scan_complete = False
            self._last_sent_scan_idx = 0
            self.get_logger().info(
                "[Buffer] HOMING → All buffers cleared, LiDAR BLOCKED"
            )

        # ── TO_START: กำลังเคลื่อนที่ 180°→90° → ไม่รับข้อมูลเด็ดขาด ────────
        elif new_state == 'TO_START':
            self.scan_buffer.clear()
            self.scan_complete = False
            self._last_sent_scan_idx = 0
            self.get_logger().info(
                "[Buffer] TO_START → Scan buffer cleared, LiDAR BLOCKED"
            )

        # ── PARKING/IDLE: กลับมาจอด → เริ่ม live preview ───────────────────
        elif new_state in STATES_PARKING_VIEW:
            self.get_logger().info(
                f"[Buffer] {new_state} → Parking FIFO active (max {PARKING_MAX_POINTS:,} pts)"
            )

        elif new_state in STATES_COLLECT_SCAN:
            self.get_logger().info(
                f"[Buffer] {new_state} → COLLECTING scan data"
            )
        elif new_state in STATES_SCAN_ACTIVE:
            self.get_logger().info(
                f"[Buffer] {new_state} → HOLDING (unidirectional mode, not collecting)"
            )

    def _on_current(self, msg: Float32):
        self.current_angle = msg.data
        self.last_angle_time = self.get_clock().now()

    # ════════════════════════════════════════════════════════════════════════
    # MQTT
    # ════════════════════════════════════════════════════════════════════════

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
                self.scan_buffer.clear()
                self.parking_buffer.clear()
                self.scan_complete = False
                self.get_logger().info("Reset: All buffers cleared")
        except Exception as e:
            self.get_logger().error(f"MQTT Cmd Error: {e}")

    def _send_stop_and_park(self):
        try:
            payload = json.dumps({'target': 'motor', 'action': 'stop'})
            self.mqtt_client.publish(MQTT_TOPIC_CMD, payload, qos=1)
            self.get_logger().warn(
                f"[Scan Complete] {SCAN_MAX_POINTS:,} pts reached → STOP & PARK"
            )
        except Exception as e:
            self.get_logger().error(f"MQTT Stop Error: {e}")

    # ════════════════════════════════════════════════════════════════════════
    # SCAN CALLBACK — CORE LOGIC
    # ════════════════════════════════════════════════════════════════════════

    def scan_callback(self, msg: LaserScan):

        # ── Gate 1: LiDAR enabled ────────────────────────────────────────────
        if not self.is_lidar_enabled:
            return

        # ── Gate 2: State whitelist ──────────────────────────────────────────
        # HOMING (180° → 180° homing)  → บล็อก
        # TO_START (180° → 90°)        → บล็อก
        # IDLE                         → บล็อก (ไม่มี motor movement)
        if self.motor_state not in STATES_LIDAR_ALLOWED:
            return

        # ── Gate 3: Encoder freshness ────────────────────────────────────────
        now = self.get_clock().now()
        time_diff = (now - self.last_angle_time).nanoseconds / 1e9
        if time_diff > 0.5:
            if (now - self.last_log_time).nanoseconds / 1e9 > 2.0:
                self.get_logger().warn(f"Waiting for Encoder... (Lag: {time_diff:.2f}s)")
                self.last_log_time = now
            return

        # ── Gate 4: Unidirectional mode — skip non-collect scan states ───────
        if self.motor_state in STATES_SCAN_ACTIVE:
            if self.motor_state not in STATES_COLLECT_SCAN:
                return   # เช่น SCAN_CCW ถ้า UNIDIRECTIONAL_SCAN=True
            if self.scan_complete:
                return   # scan buffer เต็มแล้ว

        # ════════════════════════════════════════════════════════════════════
        # 3D Cartesian Mapping (Matrix Transformation)
        # ════════════════════════════════════════════════════════════════════
        ranges = np.array(msg.ranges)

        # สร้าง Array มุมของ LiDAR ทุกจุด + Normalize
        angles_raw  = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        angles_norm = angles_raw % (2 * np.pi)

        # กรองมุม LiDAR: เก็บเฉพาะ 90° - 270°
        angle_mask    = (angles_norm >= np.radians(90.0)) & (angles_norm <= np.radians(270.0))
        # กรองระยะ: valid + ไม่เกิน 1.0 m (ถัง r=22cm)
        distance_mask = (ranges > msg.range_min) & (ranges < msg.range_max) & (ranges <= 1.0)

        valid_mask = angle_mask & distance_mask
        if not np.any(valid_mask):
            return

        d       = ranges[valid_mask]
        indices = np.where(valid_mask)[0]

        # ── Local LiDAR Coordinates ──────────────────────────────────────────
        lidar_offset = np.radians(180.0)
        theta_l      = msg.angle_min + indices * msg.angle_increment + lidar_offset

        x_local = d * np.sin(theta_l)
        y_local = np.zeros_like(d)
        z_local = -d * np.cos(theta_l)
        P_local = np.vstack((x_local, y_local, z_local))

        # ── Motor Rotation Matrix (Roll — X axis) ────────────────────────────
        # 🔧 ใส่ MOTOR_ANGLE_OFFSET_DEG เพื่อแก้ Discontinuity
        theta_m = np.radians(-(self.current_angle - 180.0 + MOTOR_ANGLE_OFFSET_DEG))
        cos_m, sin_m = np.cos(theta_m), np.sin(theta_m)

        R_motor_x = np.array([
            [1,     0,      0],
            [0, cos_m, -sin_m],
            [0, sin_m,  cos_m]
        ])

        P_global = R_motor_x @ P_local

        # ── Translation Offset (Cone Shape Fix) ──────────────────────────────
        # 🔧 เปิด APPLY_T_OFFSET = True เมื่อวัดค่าออฟเซ็ตได้แล้ว
        if APPLY_T_OFFSET:
            T_offset = np.array([[T_OFFSET_X], [T_OFFSET_Y], [T_OFFSET_Z]])
            P_global = P_global + T_offset

        new_points: list = P_global.T.tolist()

        # ════════════════════════════════════════════════════════════════════
        # BUFFER ROUTING
        # ════════════════════════════════════════════════════════════════════

        if self.motor_state in STATES_COLLECT_SCAN:
            # ── SCAN MODE: สะสมจนครบ 150,000 จุด ────────────────────────────
            remaining = SCAN_MAX_POINTS - len(self.scan_buffer)
            if remaining <= 0:
                return
            self.scan_buffer.extend(new_points[:remaining])
            if len(self.scan_buffer) >= SCAN_MAX_POINTS and not self.scan_complete:
                self.scan_complete = True
                self._send_stop_and_park()

        else:
            # ── PARKING MODE: FIFO deque ──────────────────────────────────────
            self.parking_buffer.extend(new_points)

    # ════════════════════════════════════════════════════════════════════════
    # PUBLISH CALLBACK (20 Hz)
    # ════════════════════════════════════════════════════════════════════════

    def publish_points_callback(self):
        # ── ส่งเฉพาะจุดใหม่ที่ยังไม่เคยส่ง (delta) ─────────────────────────
        # ป้องกัน packet ใหญ่เกิน 1MB ที่ MQTT broker drop
        # GUI จะ accumulate จุดเองฝั่ง client
        if self.motor_state in STATES_SCAN_ACTIVE:
            total = len(self.scan_buffer)
            new_pts = list(self.scan_buffer)[self._last_sent_scan_idx:total]
            if new_pts:
                self._last_sent_scan_idx = total
                points_to_send = new_pts
            else:
                return
        else:
            # parking: ส่งทั้งหมด (แค่ 2000 จุด ไม่หนัก)
            points_to_send = list(self.parking_buffer)
            if not points_to_send:
                return

        payload = {'points_m': points_to_send}
        try:
            self.mqtt_client.publish(MQTT_TOPIC_POINTS, json.dumps(payload), qos=0)
        except Exception as e:
            self.get_logger().error(f"MQTT Publish Error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = PointPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()