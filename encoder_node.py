import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import smbus
from rcl_interfaces.msg import SetParametersResult

class AS5600Encoder:
    DEVICE_ADDRESS = 0x36
    RAW_ANGLE_REGISTER = 0x0C
    
    def __init__(self, i2c_bus, home_offset):
        self.bus = smbus.SMBus(i2c_bus)
        self.offset = home_offset
        print(f"AS5600 Encoder Initialized with offset: {self.offset}")

    def read_raw_angle(self):
        read_bytes = self.bus.read_i2c_block_data(self.DEVICE_ADDRESS, self.RAW_ANGLE_REGISTER, 2)
        return (read_bytes[0] << 8) | read_bytes[1]

    def get_angle_degrees(self):
        raw_angle = self.read_raw_angle()
        # ใช้ self.offset ซึ่งสามารถเปลี่ยนแปลงได้
        corrected_raw_angle = (raw_angle - self.offset + 4096) % 4096
        return corrected_raw_angle * 360.0 / 4096.0

class EncoderPublisherNode(Node):
    def __init__(self):
        super().__init__('encoder_node')
        self.get_logger().info("Encoder Publisher Node Started (with Dynamic Parameters)")

        # 1. ประกาศ Parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('home_offset', 3072) # ใช้ค่าจาก YAML เป็นค่าเริ่มต้น

        # 2. ดึงค่าเริ่มต้นมาใช้งาน
        i2c_bus = self.get_parameter('i2c_bus').value
        initial_home_offset = self.get_parameter('home_offset').value
        
        self.encoder = AS5600Encoder(i2c_bus=i2c_bus, home_offset=initial_home_offset)

        # 3. สร้าง Publisher
        self.publisher_ = self.create_publisher(Float32, 'current_angle', 10)

        # 4. สร้าง Timer ให้ทำงาน
        self.timer = self.create_timer(0.01, self.publish_angle)
        
        # 5. !! เพิ่ม Parameter Callback !!
        self.add_on_set_parameters_callback(self.parameters_callback)

    def parameters_callback(self, params):
        """ฟังก์ชันที่จะถูกเรียกเมื่อมีการ `ros2 param set`"""
        for param in params:
            if param.name == 'home_offset':
                # อัปเดตค่า offset ใน object encoder โดยตรง
                new_offset = param.value
                self.encoder.offset = new_offset
                self.get_logger().info(f"Home offset dynamically updated to: {new_offset}")
        return SetParametersResult(successful=True)

    def publish_angle(self):
        angle = self.encoder.get_angle_degrees()
        msg = Float32()
        msg.data = angle
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EncoderPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
