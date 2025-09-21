import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import smbus

class AS5600Encoder:
    # ... (คัดลอกคลาส AS5600Encoder ทั้งหมดจากโค้ดเดิมมาวางที่นี่) ...
    DEVICE_ADDRESS = 0x36
    RAW_ANGLE_REGISTER = 0x0C
    def __init__(self, config):
        self.bus = smbus.SMBus(config['i2c_bus'])
        self.offset = config['home_offset']
        print("AS5600 Encoder Initialized.")
    def read_raw_angle(self):
        read_bytes = self.bus.read_i2c_block_data(self.DEVICE_ADDRESS, self.RAW_ANGLE_REGISTER, 2)
        eturn (read_bytes[0] << 8) | read_bytes[1]
    def get_angle_degrees(self):
        raw_angle = self.read_raw_angle()
        corrected_raw_angle = (raw_angle - self.offset + 4096) % 4096
        return corrected_raw_angle * 360.0 / 4096.0

class EncoderPublisherNode(Node):
    def __init__(self):
        super().__init__('encoder_node')
        self.get_logger().info("Encoder Publisher Node Started")

        # ประกาศและดึงค่า Parameters จากไฟล์ YAML
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('home_offset', 3030)

        encoder_config = {
            'i2c_bus': self.get_parameter('i2c_bus').value,
            'home_offset': self.get_parameter('home_offset').value
        }

        self.encoder = AS5600Encoder(config=encoder_config)

        # สร้าง Publisher ที่จะส่งข้อมูลไปที่ Topic '/current_angle'
        self.publisher_ = self.create_publisher(Float32, 'current_angle', 10)

        # สร้าง Timer ให้ทำงานทุกๆ 0.01 วินาที (100 Hz)
        self.timer = self.create_timer(0.01, self.publish_angle)

    def publish_angle(self):
        angle = self.encoder.get_angle_degrees()
        msg = Float32()
        msg.data = angle
        self.publisher_.publish(msg)
        # self.get_logger().info(f'Publishing Angle: {angle:.2f}') # Un-comment for debugging

def main(args=None):
    rclpy.init(args=args)
    node = EncoderPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
