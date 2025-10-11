#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter
from std_msgs.msg import Float32
import smbus

class AS5600Encoder:
    DEVICE_ADDRESS = 0x36
    RAW_ANGLE_REGISTER = 0x0C

    def __init__(self, i2c_bus: int, home_offset: int):
        self._open_bus(i2c_bus)
        self.offset = int(home_offset)

    def _open_bus(self, i2c_bus: int):
        if hasattr(self, 'bus'):
            try:
                self.bus.close()
            except Exception:
                pass
        self.bus = smbus.SMBus(int(i2c_bus))
        self.i2c_bus = int(i2c_bus)

    def read_raw_angle(self):
        data = self.bus.read_i2c_block_data(self.DEVICE_ADDRESS, self.RAW_ANGLE_REGISTER, 2)
        return ((data[0] << 8) | data[1]) & 0x0FFF  # 12-bit

    def get_angle_degrees(self) -> float:
        raw = self.read_raw_angle()
        corrected = (raw - self.offset + 4096) % 4096
        return corrected * 360.0 / 4096.0

class EncoderPublisherNode(Node):
    def __init__(self):
        super().__init__('encoder_node')
        # (1) ประกาศพารามิเตอร์ (มีค่าเริ่มต้น)
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('home_offset', 3072)  # ปรับให้ 0° ตรงจริง

        # (2) สร้างอุปกรณ์
        i2c_bus = self.get_parameter('i2c_bus').value
        home_offset = self.get_parameter('home_offset').value
        self.encoder = AS5600Encoder(i2c_bus=i2c_bus, home_offset=home_offset)

        # (3) ตั้ง publisher + timer
        self.pub = self.create_publisher(Float32, 'current_angle', 10)
        self.timer = self.create_timer(0.01, self._tick)  # 100 Hz

        # (4) เปิด parameter callback
        self.add_on_set_parameters_callback(self._param_cb)
        self.get_logger().info("encoder_node started (param-callback on).")

    def _param_cb(self, params):
        new_bus = self.encoder.i2c_bus
        new_offset = self.encoder.offset
        for p in params:
            if p.name == 'home_offset':
                if p.type_ not in (Parameter.Type.INTEGER, Parameter.Type.DOUBLE):
                    return SetParametersResult(successful=False, reason="home_offset must be number")
                v = int(p.value)
                if not (0 <= v <= 4095):
                    return SetParametersResult(successful=False, reason="home_offset must be 0..4095")
                new_offset = v
            elif p.name == 'i2c_bus':
                if p.type_ not in (Parameter.Type.INTEGER, Parameter.Type.DOUBLE):
                    return SetParametersResult(successful=False, reason="i2c_bus must be integer")
                new_bus = int(p.value)

        if new_bus != self.encoder.i2c_bus:
            self.encoder._open_bus(new_bus)
            self.get_logger().info(f"i2c_bus -> {new_bus}")
        if new_offset != self.encoder.offset:
            self.encoder.offset = new_offset
            self.get_logger().info(f"home_offset -> {new_offset}")

        return SetParametersResult(successful=True)

    def _tick(self):
        angle = float(self.encoder.get_angle_degrees())
        self.pub.publish(Float32(data=angle))

def main(args=None):
    rclpy.init(args=args)
    node = EncoderPublisherNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()