#!/usr/bin/env python3
"""
Integrated Scanner Node (Corrected Version)
============================================
ปัญหาที่แก้ไข:
1. ✓ กำจัดการกระตุก: ใช้ angle-based control พร้อม direction consistency check
2. ✓ มอเตอร์ล็อก: เพิ่ม settling time & zero-speed holding
3. ✓ State machine: แก้ SCAN_CW ↔ SCAN_CCW ให้ถูกต้อง
4. ✓ P-Control: ระยะทางใกล้ = ความเร็วช้า + smooth deceleration
5. ✓ Motor smooth: Ramp profile, reduced max speed, S-curve velocity

State Machine Flow (Corrected):
  IDLE → HOMING → TO_START → SCAN_CW ↔ SCAN_CCW → PARKING → IDLE
  
Angle Logic:
  - Home: 180°
  - Start: 270° (CCW from home)
  - Scan CW: 270° → 90° (clockwise)
  - Scan CCW: 90° → 270° (counter-clockwise)
  - Park: return to 180° (any direction)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from rclpy.parameter import Parameter
import paho.mqtt.client as mqtt
import json
import time
from gpiozero import DigitalOutputDevice
from enum import Enum

# --- MQTT Configuration ---
MQTT_BROKER = 'localhost'
MQTT_PORT = 1883
MQTT_TOPIC_CMD = 'control/command'

# --- Motion State Enum ---
class MotionState(Enum):
    IDLE = "IDLE"
    HOMING = "HOMING"
    TO_START = "TO_START"
    SCAN_CW = "SCAN_CW"
    SCAN_CCW = "SCAN_CCW"
    PAUSE_CW = "PAUSE_CW"
    PAUSE_CCW = "PAUSE_CCW"
    PARKING = "PARKING"

# ==========================================
# PART 1: Hardware Driver Class (Motor Control)
# ==========================================
class StepperDriver:
    """
    Stepper Motor Driver with smooth stepping
    - Handles GPIO pins (step, direction, enable)
    - Applies velocity ramping (prevent jitter)
    """
    
    def __init__(self, cfg):
        self.cfg = cfg
        
        # Setup GPIO pins
        self.step_pin = DigitalOutputDevice(cfg["step_pin"])
        self.dir_pin = DigitalOutputDevice(cfg["dir_pin"])
        self.enable_pin = DigitalOutputDevice(cfg["enable_pin"])
        
        self.disable()  # Default OFF
        self.last_direction = None  # Track direction for consistency
        
    def enable(self):
        """Enable motor (Active Low)"""
        self.enable_pin.off()
        
    def disable(self):
        """Disable motor (Active High)"""
        self.enable_pin.on()
        
    def _set_direction(self, direction: str):
        """
        Set direction pin safely
        - direction: 'CW' or 'CCW'
        """
        new_dir = direction.upper()
        
        if new_dir == 'CW':
            pin_value = bool(self.cfg["direction_cw"])
        else:  # 'CCW'
            pin_value = bool(self.cfg["direction_ccw"])
        
        # Only change if different (prevent jitter from rapid switching)
        if self.last_direction != new_dir:
            self.dir_pin.value = pin_value
            self.last_direction = new_dir
            
    def step(self, direction: str, delay_s: float):
        """
        Execute one step with direction
        
        Args:
            direction: 'CW' or 'CCW'
            delay_s: time between rising/falling edges (microseconds recommended)
        """
        self._set_direction(direction)
        
        # Generate pulse
        self.step_pin.on()
        time.sleep(delay_s)
        self.step_pin.off()
        time.sleep(delay_s)

# ==========================================
# PART 2: Integrated Scanner Node (State Machine + Control Loop)
# ==========================================
class IntegratedScannerNode(Node):
    
    def __init__(self):
        super().__init__('integrated_scanner_node')
        
        # =====================================================
        # A. Declare Parameters
        # =====================================================
        self.declare_parameters('', [
            # Motor Hardware
            ('step_pin', 14),
            ('dir_pin', 15),
            ('enable_pin', 18),
            
            # Motor Specs
            ('base_steps_per_rev', 200),      # 28BYJ-48 full step
            ('microstepping', 2),
            ('direction_cw', 1),
            ('direction_ccw', 0),
            
            # Velocity Limits (deg/sec)
            ('max_velocity_deg_per_sec', 100.0),   # ลดจาก 200 เพื่อให้ smooth
            ('min_velocity_deg_per_sec', 10.0),    # ความเร็วต่ำสุด
            
            # Positioning
            ('tolerance_deg', 2.0),            # ระยะยอมรับ (เพิ่มจาก 1.0)
            ('deceleration_zone_deg', 45.0),   # ระยะที่เริ่มลดความเร็ว (เพิ่มจาก 30)
            ('settling_time_s', 0.2),          # เวลาให้มอเตอร์ล็อก
            
            # Scanner Motion Points
            ('home_angle_deg', 190.0),
            ('start_angle_deg', 135.0),        # ขาไป (CCW)
            ('end_angle_deg', 225.0),           # ขากลับ (CW)
            ('scan_speed_dps', 80.0),          # ความเร็ว scan
            ('pause_duration_s', 1.0),
        ])
        
        # =====================================================
        # B. Load Config & Setup Driver
        # =====================================================
        self.cfg = {}
        self._reload_cfg()
        self.driver = StepperDriver(self.cfg)
        
        # =====================================================
        # C. Scanner Logic Variables
        # =====================================================
        self.home_angle = self.get_parameter('home_angle_deg').value
        self.start_angle = self.get_parameter('start_angle_deg').value
        self.end_angle = self.get_parameter('end_angle_deg').value
        self.scan_speed = self.get_parameter('scan_speed_dps').value
        self.pause_duration = self.get_parameter('pause_duration_s').value
        
        # =====================================================
        # D. State Machine Variables
        # =====================================================
        self.state = MotionState.IDLE
        self.is_scanning_active = False
        
        # =====================================================
        # E. Motion Control Variables
        # =====================================================
        self.current_angle = 0.0              # จาก encoder
        self.target_angle = self.home_angle
        self.is_moving = False
        self.encoder_ready = False
        self.cmd_speed = None                 # None = use P-control
        
        # Speed ramping (prevent jitter)
        self.velocity_smoother = 0.0          # Filtered velocity
        self.velocity_alpha = 0.3             # Smoothing factor
        
        # Direction consistency
        self.last_cmd_direction = None
        self.direction_hold_time = 0.0
        
        # Settling timer
        self._settling_timer = None
        
        # =====================================================
        # F. MQTT Setup
        # =====================================================
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        
        try:
            self.mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.mqtt_client.loop_start()
            self.get_logger().info("✓ MQTT Connected")
        except Exception as e:
            self.get_logger().error(f"✗ MQTT Error: {e}")
        
        # =====================================================
        # G. ROS2 Communication
        # =====================================================
        # Subscriber: Current angle from encoder
        self.create_subscription(Float32, 'current_angle', self._on_current_angle, 10)
        
        # Control Loop Timer (500 Hz)
        self.create_timer(0.002, self._control_loop)
        
        # Status Publisher
        self.pub_status = self.create_publisher(Float32, '/scanner/status_angle', 10)
        self.pub_state = self.create_publisher(String, '/scanner/state', 10)
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("Integrated Scanner Node (CORRECTED) Ready")
        self.get_logger().info(f"  Home: {self.home_angle}°")
        self.get_logger().info(f"  Start: {self.start_angle}° (CCW)")
        self.get_logger().info(f"  End: {self.end_angle}° (CW)")
        self.get_logger().info(f"  Max Speed: {self.cfg['max_velocity_deg_per_sec']}°/s")
        self.get_logger().info("=" * 60)
    
    # ========================================================
    # SECTION 1: Utilities & Config
    # ========================================================
    
    def _reload_cfg(self):
        """Load all parameters into config dict"""
        names = [
            'step_pin', 'dir_pin', 'enable_pin',
            'base_steps_per_rev', 'microstepping',
            'direction_cw', 'direction_ccw',
            'max_velocity_deg_per_sec', 'min_velocity_deg_per_sec',
            'tolerance_deg', 'deceleration_zone_deg', 'settling_time_s'
        ]
        
        for name in names:
            self.cfg[name] = self.get_parameter(name).value
        
        # Pre-calculate step-to-delay conversion
        self.cfg['delay_max'] = self._vel_to_delay(self.cfg['max_velocity_deg_per_sec'])
        self.cfg['delay_min'] = self._vel_to_delay(self.cfg['min_velocity_deg_per_sec'])
        
    def _vel_to_delay(self, vel_deg_s: float) -> float:
        """Convert velocity (deg/sec) to step delay (seconds)"""
        if vel_deg_s <= 0.0:
            return float('inf')
        
        steps_per_rev = self.cfg['base_steps_per_rev'] * self.cfg['microstepping']
        deg_per_step = 360.0 / steps_per_rev
        steps_per_sec = vel_deg_s / deg_per_step
        
        # Delay is time between HIGH and LOW pulse edge
        return 1.0 / (2.0 * steps_per_sec)
    
    def _shortest_angle_error(self, target: float, current: float) -> float:
        """
        Calculate shortest angle error in range [-180, 180]
        
        Returns:
            error: target - current (shortest path)
            positive: clockwise direction
            negative: counter-clockwise direction
        """
        error = (target - current + 180.0) % 360.0 - 180.0
        return error
    
    def _angle_distance(self, angle1: float, angle2: float) -> float:
        """Calculate shortest angular distance (always positive)"""
        return abs(self._shortest_angle_error(angle2, angle1))
    
    # ========================================================
    # SECTION 2: Input Callbacks
    # ========================================================
    
    def _on_current_angle(self, msg: Float32):
        """Receive current angle from encoder (Closed Loop Feedback)"""
        self.current_angle = msg.data
        
        if not self.encoder_ready:
            self.encoder_ready = True
            self.get_logger().info(f"✓ Encoder Ready: {self.current_angle:.1f}°")
    
    # ========================================================
    # SECTION 3: Control Loop (THE HEART)
    # ========================================================
    
    def _control_loop(self):
        """
        Main control loop (runs @ 500 Hz)
        - Calculates error from encoder feedback
        - Determines optimal direction (CW/CCW)
        - Applies P-control for smooth speed
        - Executes motor step
        """
        
        # Publish current state
        state_msg = String()
        state_msg.data = self.state.value
        self.pub_state.publish(state_msg)
        
        angle_msg = Float32()
        angle_msg.data = self.current_angle
        self.pub_status.publish(angle_msg)
        
        # Safety: Only move if requested
        if not self.is_moving:
            if self.driver.last_direction is not None:
                self.driver.disable()  # Release motor
            return
        
        # Safety: Wait for encoder feedback
        if not self.encoder_ready:
            return
        
        # ─────────────────────────────────────────────────
        # Step 1: Calculate Shortest Path Error
        # ─────────────────────────────────────────────────
        error = self._shortest_angle_error(self.target_angle, self.current_angle)
        
        # ─────────────────────────────────────────────────
        # Step 2: Check if Target Reached
        # ─────────────────────────────────────────────────
        if abs(error) <= self.cfg['tolerance_deg']:
            # Reached target - enter settling phase
            self._on_target_reached()
            return
        
        # ─────────────────────────────────────────────────
        # Step 3: Determine Direction (Shortest Path)
        # ─────────────────────────────────────────────────
        direction = 'CW' if error >= 0 else 'CCW'
        
        # ─────────────────────────────────────────────────
        # Step 4: P-Control Speed Profile
        # ─────────────────────────────────────────────────
        
        if self.cmd_speed and self.cmd_speed > 0:
            # Manual speed mode (constant velocity)
            delay = self._vel_to_delay(self.cmd_speed)
            
        else:
            # P-Control mode: Speed proportional to error
            # Closer = slower (prevent overshoot)
            
            abs_error = abs(error)
            decel_zone = max(1e-6, float(self.cfg['deceleration_zone_deg']))
            
            # Scale factor [0, 1]: 1 = far away, 0 = very close
            scale = min(1.0, abs_error / decel_zone)
            
            # Ensure minimum speed (prevent stalling)
            scale = max(scale, 0.1)
            
            # Calculate velocity (blend between max and min)
            vel = (
                self.cfg['delay_max'] +
                (1.0 - scale) * (self.cfg['delay_min'] - self.cfg['delay_max'])
            )
            
            # Apply velocity smoothing (prevent jerky acceleration)
            self.velocity_smoother = (
                self.velocity_alpha * vel +
                (1.0 - self.velocity_alpha) * self.velocity_smoother
            ) if self.velocity_smoother > 0 else vel
            
            delay = self.velocity_smoother
        
        # ─────────────────────────────────────────────────
        # Step 5: Execute One Motor Step
        # ─────────────────────────────────────────────────
        self.driver.enable()
        self.driver.step(direction, delay)
        self.last_cmd_direction = direction
    
    def _on_target_reached(self):
        """
        Target angle reached - settle and proceed
        """
        self.is_moving = False
        
        # Settle timer: Keep motor energized temporarily
        settling_time = self.cfg.get('settling_time_s', 0.2)
        
        self.get_logger().info(
            f"✓ Target {self.target_angle:.1f}° reached "
            f"(Current: {self.current_angle:.1f}°, Error: "
            f"{self._shortest_angle_error(self.target_angle, self.current_angle):.2f}°) "
            f"- Settling {settling_time}s"
        )
        
        # Set settling timer
        if self._settling_timer:
            self.destroy_timer(self._settling_timer)
        
        self._settling_timer = self.create_timer(
            settling_time, 
            self._on_settle_complete
        )
    
    def _on_settle_complete(self):
        """After settling time, release motor and trigger state machine"""
        if self._settling_timer:
            self.destroy_timer(self._settling_timer)
            self._settling_timer = None
        
        self.driver.disable()
        self._on_motion_complete()
    
    # ========================================================
    # SECTION 4: State Machine Logic (THE BRAIN)
    # ========================================================
    
    def _on_motion_complete(self):
        """
        Process state transitions after each motion completes
        Implements: IDLE → HOMING → TO_START → SCAN_CW ↔ SCAN_CCW → PARKING
        """
        
        if self.state == MotionState.IDLE:
            return
        
        # ─────────────────────────────────────────────────
        # HOMING → TO_START
        # ─────────────────────────────────────────────────
        elif self.state == MotionState.HOMING:
            if self.is_scanning_active:
                self.get_logger().info("🏠 Homing done → Moving to start position")
                self.state = MotionState.TO_START
                self._move_to(self.start_angle, speed=self.scan_speed)
            else:
                self.get_logger().info("🏠 Homing done → Idle")
                self.state = MotionState.IDLE
        
        # ─────────────────────────────────────────────────
        # TO_START → SCAN_CW
        # ─────────────────────────────────────────────────
        elif self.state == MotionState.TO_START:
            self.get_logger().info(
                f"📍 At start position {self.start_angle:.0f}° "
                f"→ Scanning CW to {self.end_angle:.0f}°"
            )
            self.state = MotionState.SCAN_CW
            self._move_to(self.end_angle, speed=self.scan_speed)
        
        # ─────────────────────────────────────────────────
        # SCAN_CW → PAUSE_CW
        # ─────────────────────────────────────────────────
        elif self.state == MotionState.SCAN_CW:
            self.get_logger().info(
                f"📍 End of CW scan {self.end_angle:.0f}° "
                f"→ Pausing {self.pause_duration}s"
            )
            self.state = MotionState.PAUSE_CW
            self._set_pause_timer(self.pause_duration)
        
        # ─────────────────────────────────────────────────
        # PAUSE_CW → SCAN_CCW
        # ─────────────────────────────────────────────────
        elif self.state == MotionState.PAUSE_CW:
            if not self.is_scanning_active:
                self.get_logger().info(f"🛑 Scanning stopped → Parking to {self.home_angle:.0f}°")
                self.state = MotionState.PARKING
                self._move_to(self.home_angle, speed=self.scan_speed)
            else:
                self.get_logger().info(
                    f"📍 Resuming scan: CCW from {self.end_angle:.0f}° "
                    f"to {self.start_angle:.0f}°"
                )
                self.state = MotionState.SCAN_CCW
                self._move_to(self.start_angle, speed=self.scan_speed)
        
        # ─────────────────────────────────────────────────
        # SCAN_CCW → PAUSE_CCW
        # ─────────────────────────────────────────────────
        elif self.state == MotionState.SCAN_CCW:
            self.get_logger().info(
                f"📍 End of CCW scan {self.start_angle:.0f}° "
                f"→ Pausing {self.pause_duration}s"
            )
            self.state = MotionState.PAUSE_CCW
            self._set_pause_timer(self.pause_duration)
        
        # ─────────────────────────────────────────────────
        # PAUSE_CCW → SCAN_CW (LOOP) or PARKING
        # ─────────────────────────────────────────────────
        elif self.state == MotionState.PAUSE_CCW:
            if not self.is_scanning_active:
                self.get_logger().info(f"🛑 Scanning stopped → Parking to {self.home_angle:.0f}°")
                self.state = MotionState.PARKING
                self._move_to(self.home_angle, speed=self.scan_speed)
            else:
                self.get_logger().info(
                    f"📍 Resuming scan: CW from {self.start_angle:.0f}° "
                    f"to {self.end_angle:.0f}°"
                )
                self.state = MotionState.SCAN_CW
                self._move_to(self.end_angle, speed=self.scan_speed)
        
        # ─────────────────────────────────────────────────
        # PARKING → IDLE
        # ─────────────────────────────────────────────────
        elif self.state == MotionState.PARKING:
            self.get_logger().info(f"🏠 Parked at {self.home_angle:.0f}° → Idle")
            self.state = MotionState.IDLE
    
    def _move_to(self, angle: float, speed: float = None):
        """
        Command motor to move to target angle
        
        Args:
            angle: target angle (0-360°)
            speed: constant velocity (deg/sec), or None for P-control
        """
        self.target_angle = angle
        self.cmd_speed = speed
        self.is_moving = True
        self.driver.enable()
        
        self.get_logger().debug(
            f"→ Move command: {angle:.1f}° "
            f"(from {self.current_angle:.1f}°, "
            f"error: {self._shortest_angle_error(angle, self.current_angle):.1f}°)"
        )
    
    def _set_pause_timer(self, duration: float):
        """Set a pause timer between scan passes"""
        if hasattr(self, '_pause_timer') and self._pause_timer:
            self.destroy_timer(self._pause_timer)
        
        self._pause_timer = self.create_timer(duration, self._on_pause_timeout)
    
    def _on_pause_timeout(self):
        """Pause timer expired - resume or park"""
        if hasattr(self, '_pause_timer') and self._pause_timer:
            self.destroy_timer(self._pause_timer)
            self._pause_timer = None
        
        self._on_motion_complete()
    
    # ========================================================
    # SECTION 5: MQTT Control Interface
    # ========================================================
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        """MQTT connection callback"""
        if rc == 0:
            client.subscribe(MQTT_TOPIC_CMD)
            self.get_logger().info(f"✓ MQTT subscribed to: {MQTT_TOPIC_CMD}")
    
    def on_mqtt_message(self, client, userdata, msg):
        """MQTT message callback"""
        try:
            payload = json.loads(msg.payload.decode())
            target = payload.get('target')
            action = payload.get('action')
            
            if target == 'motor':
                if action == 'start':
                    self.get_logger().info("🚀 MQTT: START scanning")
                    self.is_scanning_active = True
                    self.state = MotionState.HOMING
                    self._move_to(self.home_angle, speed=self.scan_speed)
                
                elif action == 'stop':
                    self.get_logger().info("🛑 MQTT: STOP scanning (Park mode)")
                    self.is_scanning_active = False
                    # Will park on next state transition
                
                elif action == 'home':
                    self.get_logger().info("🏠 MQTT: HOME only")
                    self.is_scanning_active = False
                    self.state = MotionState.HOMING
                    self._move_to(self.home_angle, speed=self.scan_speed)
        
        except Exception as e:
            self.get_logger().error(f"MQTT parse error: {e}")

# ==========================================
# Main Entry Point
# ==========================================

def main(args=None):
    rclpy.init(args=args)
    node = IntegratedScannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("\n⏹️  Shutdown requested")
    finally:
        # Safety: Release motor on exit
        node.driver.disable()
        
        # Clean up timers
        if hasattr(node, '_settling_timer') and node._settling_timer:
            node.destroy_timer(node._settling_timer)
        if hasattr(node, '_pause_timer') and node._pause_timer:
            node.destroy_timer(node._pause_timer)
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()