# encoder_node.py
# — แก้บั๊กอ่าน I2C, เพิ่ม offset_deg (fine trim), และ parameter callback —
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float32
import smbus

class AS5600Encoder:
    DEVICE_ADDRESS = 0x36
    RAW_ANGLE_REGISTER = 0x0C

    def __init__(self, config):
        self._i2c_bus_num = int(config['i2c_bus'])
        self.bus = smbus.SMBus(self._i2c_bus_num)
        self.offset_raw = int(config['home_offset'])
        print("AS5600 Encoder Initialized.")

    def reconfigure(self, config):
        new_bus = int(config['i2c_bus'])
        if new_bus != self._i2c_bus_num:
            try: self.bus.close()
            except Exception: pass
            self.bus = smbus.SMBus(new_bus)
            self._i2c_bus_num = new_bus
        self.offset_raw = int(config['home_offset'])

    def read_raw_angle(self):
        data = self.bus.read_i2c_block_data(self.DEVICE_ADDRESS, self.RAW_ANGLE_REGISTER, 2)
        raw = (data[0] << 8) | data[1]
        return raw & 0x0FFF  # 12-bit

    def get_angle_degrees(self):
        raw = self.read_raw_angle()
        corrected_raw = (raw - self.offset_raw + 4096) % 4096
        return corrected_raw * (360.0 / 4096.0)

class EncoderPublisherNode(Node):
    def __init__(self):
        super().__init__('encoder_node')
        self.get_logger().info("Encoder Publisher Node Started")

        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('home_offset', 3072)
        self.declare_parameter('offset_deg', 0.0)  # fine trim (optional)

        self._reload_config()
        self.encoder = AS5600Encoder(config=self.config)

        self.publisher_ = self.create_publisher(Float32, 'current_angle', 10)
        self.timer = self.create_timer(0.01, self.publish_angle)  # 100 Hz

        self.add_on_set_parameters_callback(self._on_param_update)

    def _reload_config(self):
        names = ['i2c_bus', 'home_offset', 'offset_deg']
        self.config = {n: self.get_parameter(n).value for n in names}

    def _on_param_update(self, params):
        updates = {p.name: p.value for p in params}
        if 'home_offset' in updates:
            v = int(updates['home_offset'])
            if not (0 <= v <= 4095):
                return SetParametersResult(successful=False, reason='home_offset must be in [0, 4095]')
        for p in params:
            self.set_parameters([Parameter(name=p.name, value=p.value)])
        self._reload_config()
        self.encoder.reconfigure(self.config)
        return SetParametersResult(successful=True)

    def publish_angle(self):
        angle = self.encoder.get_angle_degrees()
        angle = (angle + float(self.config.get('offset_deg', 0.0))) % 360.0
        msg = Float32()
        msg.data = float(angle)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = EncoderPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
