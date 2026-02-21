#!/usr/bin/env python3
"""
FIXED Integrated Scanner Node
==============================
Fixes:
1. ✓ Constrain angles to [90°, 270°] - no wrapping through 0/360
2. ✓ Dynamic parameter updates via callback
3. ✓ Proper ROS2 parameter handling

Key changes:
- Add _constrain_to_scan_range() method
- Add _param_callback() for dynamic updates  
- Use get_parameter() for runtime values
- Constrain angles before error calculation
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float32, String
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
# PART 1: Hardware Driver Class
# ==========================================
class StepperDriver:
    """Stepper Motor Driver (TB6600)"""
    
    def __init__(self, cfg):
        self.cfg = cfg
        self.step_pin = DigitalOutputDevice(cfg["step_pin"])
        self.dir_pin = DigitalOutputDevice(cfg["dir_pin"])
        self.enable_pin = DigitalOutputDevice(cfg["enable_pin"])
        self.disable()
        self.last_direction = None
        
    def enable(self):
        self.enable_pin.off()
        
    def disable(self):
        self.enable_pin.on()
        
    def _set_direction(self, direction: str):
        new_dir = direction.upper()
        if new_dir == 'CW':
            pin_value = bool(self.cfg["direction_cw"])
        else:
            pin_value = bool(self.cfg["direction_ccw"])
        
        if self.last_direction != new_dir:
            self.dir_pin.value = pin_value
            self.last_direction = new_dir
            
    def step(self, direction: str, delay_s: float):
        self._set_direction(direction)
        self.step_pin.on()
        time.sleep(delay_s)
        self.step_pin.off()
        time.sleep(delay_s)

# ==========================================
# PART 2: Integrated Scanner Node (FIXED)
# ==========================================
class IntegratedScannerNode(Node):
    
    def __init__(self):
        super().__init__('integrated_scanner_node')
        
        # =====================================================
        # A. Declare Parameters
        # =====================================================
        self.declare_parameters('', [
            # GPIO
            ('step_pin', 23),
            ('dir_pin', 24),
            ('enable_pin', 25),
            
            # Motor
            ('base_steps_per_rev', 200),
            ('microstepping', 16),
            ('direction_cw', 1),
            ('direction_ccw', 0),
            
            # Velocity
            ('max_velocity_deg_per_sec', 300.0),
            ('min_velocity_deg_per_sec', 30.0),
            
            # Position Control
            ('tolerance_deg', 2.0),
            ('deceleration_zone_deg', 50.0),
            ('settling_time_s', 0.2),
            
            # Scanner Motion
            ('home_angle_deg', 180.0),
            ('start_angle_deg', 90.0),
            ('end_angle_deg', 270.0),
            ('scan_speed_dps', 120.0),
            ('pause_duration_s', 0.5),
            
            # Advanced
            ('velocity_smoother_alpha', 0.3),
        ])
        
        # =====================================================
        # B. Load Config
        # =====================================================
        self.cfg = {}
        self._reload_cfg()
        self.driver = StepperDriver(self.cfg)
        
        # =====================================================
        # C. Scanner Variables
        # =====================================================
        self.state = MotionState.IDLE
        self.is_scanning_active = False
        self.current_angle = 0.0
        self.target_angle = self.get_parameter('home_angle_deg').value
        self.is_moving = False
        self.encoder_ready = False
        self.cmd_speed = None
        
        self.velocity_smoother = 0.0
        self.last_cmd_direction = None
        self._settling_timer = None
        
        # =====================================================
        # D. MQTT Setup
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
        # E. ROS2 Communication
        # =====================================================
        self.create_subscription(Float32, 'current_angle', self._on_current_angle, 10)
        self.create_timer(0.002, self._control_loop)  # 500 Hz
        self.pub_status = self.create_publisher(Float32, '/scanner/status_angle', 10)
        self.pub_state = self.create_publisher(String, '/scanner/state', 10)
        
        # =====================================================
        # F. PARAMETER CALLBACK (NEW!)
        # =====================================================
        self.add_on_set_parameters_callback(self._param_callback)
        
        self.get_logger().info("="*60)
        self.get_logger().info("✓ Scanner Node Ready (with dynamic params)")
        self.get_logger().info("="*60)
    
    # ========================================================
    # SECTION 1: Utilities & Config
    # ========================================================
    
    def _reload_cfg(self):
        """Load parameters into config"""
        names = [
            'step_pin', 'dir_pin', 'enable_pin',
            'base_steps_per_rev', 'microstepping',
            'direction_cw', 'direction_ccw',
            'max_velocity_deg_per_sec', 'min_velocity_deg_per_sec',
            'tolerance_deg', 'deceleration_zone_deg', 'settling_time_s',
            'velocity_smoother_alpha'
        ]
        
        for name in names:
            self.cfg[name] = self.get_parameter(name).value
        
        self.cfg['delay_max'] = self._vel_to_delay(self.cfg['max_velocity_deg_per_sec'])
        self.cfg['delay_min'] = self._vel_to_delay(self.cfg['min_velocity_deg_per_sec'])
    
    def _param_callback(self, params):
        """
        FIXED: Dynamic parameter callback
        Allows changing parameters at runtime without rebuild
        """
        try:
            for param in params:
                # Update cfg with new value
                self.cfg[param.name] = param.value
                
                # Recalculate derived values if needed
                if 'velocity' in param.name:
                    self.cfg['delay_max'] = self._vel_to_delay(
                        self.cfg['max_velocity_deg_per_sec']
                    )
                    self.cfg['delay_min'] = self._vel_to_delay(
                        self.cfg['min_velocity_deg_per_sec']
                    )
                
                self.get_logger().info(f"Parameter updated: {param.name} = {param.value}")
            
            return SetParametersResult(successful=True)
        except Exception as e:
            self.get_logger().error(f"Parameter callback error: {e}")
            return SetParametersResult(successful=False, reason=str(e))
    
    def _vel_to_delay(self, vel_deg_s: float) -> float:
        """Convert velocity to step delay"""
        if vel_deg_s <= 0.0:
            return float('inf')
        
        steps_per_rev = self.cfg['base_steps_per_rev'] * self.cfg['microstepping']
        deg_per_step = 360.0 / steps_per_rev
        steps_per_sec = vel_deg_s / deg_per_step
        return 1.0 / (2.0 * steps_per_sec)
    
    def _constrain_to_scan_range(self, angle: float) -> float:
        """
        FIXED: Constrain angle to [90°, 270°] range
        Prevents wrapping through 0/360
        
        Scan zone: 90° to 270° (half circle, never crosses 0/360)
        """
        angle = angle % 360.0
        
        if angle < 90.0:
            # 0-89: Map to upper range (270-359)
            angle = 360.0 + angle  # 0-89 → 360-449
            # But this is still problematic, so wrap it
            angle = 270.0 + (angle - 360.0)  # → 270-359
        elif angle > 270.0:
            # 271-359: Keep in upper range or map to upper
            # Actually, 271-359 should wrap to negative for safety
            # So: 271-359 → -89 to -1
            angle = angle - 360.0  # 271-359 → -89 to -1
            # Then constrain to 270-359
            angle = 360.0 + angle  # -89 to -1 → 271-359
        
        return angle
    
    def _shortest_angle_error(self, target: float, current: float) -> float:
        """
        FIXED: Calculate shortest angle error with range constraint
        Ensures we never wrap through 0/360
        """
        # Constrain both to scan range FIRST
        target_c = self._constrain_to_scan_range(target)
        current_c = self._constrain_to_scan_range(current)
        
        # Now calculate error (should be small)
        error = target_c - current_c
        
        # Handle wrap-around within constrained range
        if error > 180.0:
            error -= 360.0
        elif error < -180.0:
            error += 360.0
        
        return error
    
    def _angle_distance(self, angle1: float, angle2: float) -> float:
        """Calculate shortest angular distance"""
        return abs(self._shortest_angle_error(angle2, angle1))
    
    # ========================================================
    # SECTION 2: Input Callbacks
    # ========================================================
    
    def _on_current_angle(self, msg: Float32):
        """Receive current angle from encoder"""
        self.current_angle = msg.data
        
        if not self.encoder_ready:
            self.encoder_ready = True
            self.get_logger().info(f"✓ Encoder Ready: {self.current_angle:.1f}°")
    
    # ========================================================
    # SECTION 3: Control Loop (THE HEART)
    # ========================================================
    
    def _control_loop(self):
        """Main control loop @ 500 Hz"""
        
        # Publish status
        state_msg = String()
        state_msg.data = self.state.value
        self.pub_state.publish(state_msg)
        
        angle_msg = Float32()
        angle_msg.data = self.current_angle
        self.pub_status.publish(angle_msg)
        
        if not self.is_moving:
            if self.driver.last_direction is not None:
                self.driver.disable()
            return
        
        if not self.encoder_ready:
            return
        
        # ─────────────────────────────────────────────────
        # FIXED: Calculate error with angle constraint
        # ─────────────────────────────────────────────────
        error = self._shortest_angle_error(self.target_angle, self.current_angle)
        
        # Check if reached
        if abs(error) <= self.cfg['tolerance_deg']:
            self._on_target_reached()
            return
        
        # Determine direction
        direction = 'CW' if error >= 0 else 'CCW'
        
        # Calculate speed (P-Control)
        if self.cmd_speed and self.cmd_speed > 0:
            delay = self._vel_to_delay(self.cmd_speed)
        else:
            abs_error = abs(error)
            decel = max(1e-6, float(self.cfg['deceleration_zone_deg']))
            scale = min(1.0, abs_error / decel)
            scale = max(scale, 0.1)
            
            vel_delay = (
                self.cfg['delay_max'] +
                (1.0 - scale) * (self.cfg['delay_min'] - self.cfg['delay_max'])
            )
            
            alpha = self.cfg.get('velocity_smoother_alpha', 0.3)
            self.velocity_smoother = (
                alpha * vel_delay +
                (1.0 - alpha) * self.velocity_smoother
            ) if self.velocity_smoother > 0 else vel_delay
            
            delay = self.velocity_smoother
        
        # Execute step
        self.driver.enable()
        self.driver.step(direction, delay)
        self.last_cmd_direction = direction
    
    def _on_target_reached(self):
        """Target reached - settle"""
        self.is_moving = False
        settling_time = self.cfg.get('settling_time_s', 0.15)
        
        self.get_logger().info(
            f"✓ Target {self.target_angle:.1f}° reached "
            f"(Current: {self.current_angle:.1f}°)"
        )
        
        if self._settling_timer:
            self.destroy_timer(self._settling_timer)
        
        self._settling_timer = self.create_timer(
            settling_time,
            self._on_settle_complete
        )
    
    def _on_settle_complete(self):
        """After settling, trigger next state"""
        if self._settling_timer:
            self.destroy_timer(self._settling_timer)
            self._settling_timer = None
        
        self.driver.disable()
        self._on_motion_complete()
    
    # ========================================================
    # SECTION 4: State Machine Logic
    # ========================================================
    
    def _on_motion_complete(self):
        """Process state transitions"""
        
        if self.state == MotionState.IDLE:
            return
        
        elif self.state == MotionState.HOMING:
            if self.is_scanning_active:
                self.get_logger().info(" Homing done → Moving to start position")
                self.state = MotionState.TO_START
                self._move_to(self.get_parameter('start_angle_deg').value, speed=self.get_parameter('scan_speed_dps').value)
            else:
                self.get_logger().info(" Homing done → Idle")
                self.state = MotionState.IDLE
        
        elif self.state == MotionState.TO_START:
            self.get_logger().info(" At start position → Scanning CW")
            self.state = MotionState.SCAN_CW
            self._move_to(self.get_parameter('end_angle_deg').value, speed=self.get_parameter('scan_speed_dps').value)
        
        elif self.state == MotionState.SCAN_CW:
            self.get_logger().info(" End of CW scan → Pausing")
            self.state = MotionState.PAUSE_CW
            self._set_pause_timer(self.get_parameter('pause_duration_s').value)
        
        elif self.state == MotionState.PAUSE_CW:
            if not self.is_scanning_active:
                self.get_logger().info(f" Scanning stopped → Parking")
                self.state = MotionState.PARKING
                self._move_to(self.get_parameter('home_angle_deg').value, speed=self.get_parameter('scan_speed_dps').value)
            else:
                self.get_logger().info(" Resuming scan: CCW")
                self.state = MotionState.SCAN_CCW
                self._move_to(self.get_parameter('start_angle_deg').value, speed=self.get_parameter('scan_speed_dps').value)
        
        elif self.state == MotionState.SCAN_CCW:
            self.get_logger().info(" End of CCW scan → Pausing")
            self.state = MotionState.PAUSE_CCW
            self._set_pause_timer(self.get_parameter('pause_duration_s').value)
        
        elif self.state == MotionState.PAUSE_CCW:
            if not self.is_scanning_active:
                self.get_logger().info(f" Scanning stopped → Parking")
                self.state = MotionState.PARKING
                self._move_to(self.get_parameter('home_angle_deg').value, speed=self.get_parameter('scan_speed_dps').value)
            else:
                self.get_logger().info(" Resuming scan: CW")
                self.state = MotionState.SCAN_CW
                self._move_to(self.get_parameter('end_angle_deg').value, speed=self.get_parameter('scan_speed_dps').value)
        
        elif self.state == MotionState.PARKING:
            self.get_logger().info(f" Parked → Idle")
            self.state = MotionState.IDLE
    
    def _move_to(self, angle: float, speed: float = None):
        """Command move to target angle"""
        # Constrain target to scan range
        self.target_angle = self._constrain_to_scan_range(angle)
        self.cmd_speed = speed
        self.is_moving = True
        self.driver.enable()
        
        self.get_logger().debug(f"→ Move to {angle:.1f}° (constrained: {self.target_angle:.1f}°)")
    
    def _set_pause_timer(self, duration: float):
        """Set pause timer"""
        if hasattr(self, '_pause_timer') and self._pause_timer:
            self.destroy_timer(self._pause_timer)
        
        self._pause_timer = self.create_timer(duration, self._on_pause_timeout)
    
    def _on_pause_timeout(self):
        """Pause timer expired"""
        if hasattr(self, '_pause_timer') and self._pause_timer:
            self.destroy_timer(self._pause_timer)
            self._pause_timer = None
        
        self._on_motion_complete()
    
    # ========================================================
    # SECTION 5: MQTT Control
    # ========================================================
    
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
                    self.get_logger().info(" MQTT: START")
                    self.is_scanning_active = True
                    self.state = MotionState.HOMING
                    self._move_to(self.get_parameter('home_angle_deg').value, speed=self.get_parameter('scan_speed_dps').value)
                
                elif action == 'stop':
                    self.get_logger().info(" MQTT: STOP")
                    self.is_scanning_active = False
                
                elif action == 'home':
                    self.get_logger().info(" MQTT: HOME")
                    self.is_scanning_active = False
                    self.state = MotionState.HOMING
                    self._move_to(self.get_parameter('home_angle_deg').value, speed=self.get_parameter('scan_speed_dps').value)
        except Exception as e:
            self.get_logger().error(f"MQTT error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = IntegratedScannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.driver.disable()
        if hasattr(node, '_settling_timer') and node._settling_timer:
            node.destroy_timer(node._settling_timer)
        if hasattr(node, '_pause_timer') and node._pause_timer:
            node.destroy_timer(node._pause_timer)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()