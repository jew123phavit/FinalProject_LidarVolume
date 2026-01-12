#!/usr/bin/env python3
"""
Encoder Publisher Node (AS5600)
================================
- Reads 12-bit encoder angle from AS5600 via I2C
- Publishes angle (0-360°) to 'current_angle' topic
- Supports dynamic parameter updates (home_offset, i2c_bus)
- No smoothing here (smoothing in scanner node instead)

Connection:
  AS5600 (GY-512) ↔ Raspberry Pi I2C
  - VCC → 3.3V (or 5V with voltage divider on SDA/SCL)
  - GND → GND
  - SDA → GPIO 2 (I2C SDA)
  - SCL → GPIO 3 (I2C SCL)
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter
from std_msgs.msg import Float32
import smbus

# ==========================================
# AS5600 Encoder Hardware Driver
# ==========================================

class AS5600Encoder:
    """
    AS5600 Magnetic Encoder Interface (I2C)
    - 12-bit resolution (4096 positions = 0.088° per step)
    - No moving parts, highly reliable
    """
    
    DEVICE_ADDRESS = 0x36          # AS5600 default I2C address
    RAW_ANGLE_REGISTER = 0x0C      # 2-byte angle register
    
    def __init__(self, i2c_bus: int, home_offset: int):
        """
        Initialize encoder
        
        Args:
            i2c_bus: I2C bus number (usually 1 for RPi)
            home_offset: Offset to set 0° at home position (0-4095 raw counts)
        """
        self._open_bus(i2c_bus)
        self.offset = int(home_offset)
    
    def _open_bus(self, i2c_bus: int):
        """Open I2C bus (close existing if open)"""
        if hasattr(self, 'bus'):
            try:
                self.bus.close()
            except Exception:
                pass
        
        self.bus = smbus.SMBus(int(i2c_bus))
        self.i2c_bus = int(i2c_bus)
    
    def read_raw_angle(self) -> int:
        """
        Read raw 12-bit angle from AS5600
        
        Returns:
            raw_angle: 0-4095 (0° = 0x000, 90° = 0x400, 180° = 0x800, etc.)
        """
        data = self.bus.read_i2c_block_data(
            self.DEVICE_ADDRESS,
            self.RAW_ANGLE_REGISTER,
            2  # Read 2 bytes
        )
        
        # Combine MSB and LSB into 12-bit value
        raw = ((data[0] << 8) | data[1]) & 0x0FFF
        return raw
    
    def get_angle_degrees(self) -> float:
        """
        Get angle in degrees (0-360°) with home offset applied
        
        Returns:
            angle_deg: 0.0 to 360.0 degrees
            
        Logic:
            1. Read raw 12-bit value (0-4095)
            2. Subtract home_offset to shift 0° to desired position
            3. Convert to degrees: (0-4095) → (0-360°)
        """
        raw = self.read_raw_angle()
        
        # Apply home offset with wrapping
        corrected = (raw - self.offset + 4096) % 4096
        
        # Convert to degrees
        angle_deg = corrected * 360.0 / 4096.0
        
        return angle_deg

# ==========================================
# ROS2 Node
# ==========================================

class EncoderPublisherNode(Node):
    """
    ROS2 node that publishes encoder angle
    - Topic: 'current_angle' (Float32 msg)
    - Rate: 100 Hz
    """
    
    def __init__(self):
        super().__init__('encoder_node')
        
        # ─────────────────────────────────────────────────
        # Step 1: Declare Parameters
        # ─────────────────────────────────────────────────
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('home_offset', 819)  # ปรับค่าตามหอม encoder ของคุณ
        
        # ─────────────────────────────────────────────────
        # Step 2: Read Parameters & Initialize Hardware
        # ─────────────────────────────────────────────────
        i2c_bus = self.get_parameter('i2c_bus').value
        home_offset = self.get_parameter('home_offset').value
        
        try:
            self.encoder = AS5600Encoder(i2c_bus=i2c_bus, home_offset=home_offset)
            self.get_logger().info(
                f"✓ AS5600 Encoder initialized (Bus: {i2c_bus}, Offset: {home_offset})"
            )
        except Exception as e:
            self.get_logger().error(f"✗ Encoder init failed: {e}")
            raise
        
        # ─────────────────────────────────────────────────
        # Step 3: Create Publisher & Timer
        # ─────────────────────────────────────────────────
        self.pub = self.create_publisher(Float32, 'current_angle', 10)
        
        # Publish at 100 Hz
        self.timer = self.create_timer(0.01, self._tick)
        
        # ─────────────────────────────────────────────────
        # Step 4: Enable Dynamic Parameter Updates
        # ─────────────────────────────────────────────────
        self.add_on_set_parameters_callback(self._param_callback)
        
        self.get_logger().info("✓ EncoderPublisherNode ready (100 Hz)")
        
        # Internal tracking
        self._last_angle = None
        self._error_count = 0
    
    def _param_callback(self, params):
        """
        Callback for parameter changes (ROS2 std feature)
        - Validates new values
        - Updates hardware if needed
        """
        
        new_bus = self.encoder.i2c_bus
        new_offset = self.encoder.offset
        
        for param in params:
            # ─ home_offset parameter ─
            if param.name == 'home_offset':
                # Type check
                if param.type_ not in (Parameter.Type.INTEGER, Parameter.Type.DOUBLE):
                    return SetParametersResult(
                        successful=False,
                        reason="home_offset must be numeric"
                    )
                
                # Range check (0-4095 for 12-bit)
                value = int(param.value)
                if not (0 <= value <= 4095):
                    return SetParametersResult(
                        successful=False,
                        reason="home_offset must be 0..4095 (12-bit range)"
                    )
                
                new_offset = value
            
            # ─ i2c_bus parameter ─
            elif param.name == 'i2c_bus':
                # Type check
                if param.type_ not in (Parameter.Type.INTEGER, Parameter.Type.DOUBLE):
                    return SetParametersResult(
                        successful=False,
                        reason="i2c_bus must be integer"
                    )
                
                new_bus = int(param.value)
        
        # ─────────────────────────────────────────────────
        # Apply changes
        # ─────────────────────────────────────────────────
        
        if new_bus != self.encoder.i2c_bus:
            try:
                self.encoder._open_bus(new_bus)
                self.get_logger().info(f"✓ I2C bus changed: {new_bus}")
            except Exception as e:
                return SetParametersResult(
                    successful=False,
                    reason=f"Failed to switch I2C bus: {e}"
                )
        
        if new_offset != self.encoder.offset:
            self.encoder.offset = new_offset
            self.get_logger().info(f"✓ Home offset updated: {new_offset} (0° calibrated)")
        
        return SetParametersResult(successful=True)
    
    def _tick(self):
        """
        Timer callback: Read encoder and publish
        """
        try:
            angle = float(self.encoder.get_angle_degrees())
            
            # Publish to topic
            msg = Float32()
            msg.data = angle
            self.pub.publish(msg)
            
            # Log if significant change (for debugging)
            if (self._last_angle is None or
                abs(angle - self._last_angle) > 5.0):
                # self.get_logger().debug(f"Angle: {angle:.1f}°")
                self._last_angle = angle
            
            # Reset error counter on successful read
            self._error_count = 0
        
        except Exception as e:
            self._error_count += 1
            
            if self._error_count <= 1:  # Log only first error
                self.get_logger().error(f"Encoder read failed: {e}")
            elif self._error_count % 100 == 0:  # Log periodically
                self.get_logger().error(
                    f"Encoder errors: {self._error_count} consecutive"
                )

# ==========================================
# Main Entry Point
# ==========================================

def main(args=None):
    rclpy.init(args=args)
    node = EncoderPublisherNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
