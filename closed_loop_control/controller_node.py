import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from gpiozero import DigitalOutputDevice
import time



class StepperDriver:
    # ... (คัดลอกคลาส StepperDriver ทั้งหมดจากโค้ดเดิมมาวางที่นี่) ...
    def __init__(self, config):
        self.config = config
        self.step_pin = DigitalOutputDevice(config["step_pin"])
        self.dir_pin = DigitalOutputDevice(config["dir_pin"])
        self.enable_pin = DigitalOutputDevice(config["enable_pin"])
        print(f"Stepper Driver Initialized.")
        self.disable()
    def enable(self):
        self.enable_pin.off()
    def disable(self):
        self.enable_pin.on()
    def _set_direction(self, direction):
        if direction == 'CW':
            self.dir_pin.value = self.config["direction_cw"]
        elif direction == 'CCW':
            self.dir_pin.value = self.config["direction_ccw"]
    def step(self, direction, delay):
        self._set_direction(direction)
        self.step_pin.on()
        time.sleep(delay)
        self.step_pin.off()
        time.sleep(delay)

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.get_logger().info("Controller Node Started")

        # ประกาศและดึงค่า Parameters ทั้งหมด
        self.declare_parameter('step_pin', 14)
        self.declare_parameter('dir_pin', 15)
        self.declare_parameter('enable_pin', 18)
        self.declare_parameter('base_steps_per_rev', 200)
        self.declare_parameter('microstepping', 8)
        self.declare_parameter('direction_cw', 1)
        self.declare_parameter('direction_ccw', 0)
        self.declare_parameter('tolerance_deg', 1.0)
        self.declare_parameter('max_speed_delay', 0.0005)
        self.declare_parameter('min_speed_delay', 0.005)
        self.declare_parameter('deceleration_zone_deg', 30.0)

        self.config = {param.name: param.value for param in self.get_parameters(self._parameters.keys())}

        self.driver = StepperDriver(config=self.config)

        # ตัวแปรสถานะ
        self.current_angle = 0.0
        self.target_angle = 0.0
        self.is_moving = False

        # สร้าง Subscriber 2 ตัว
        self.current_angle_sub = self.create_subscription(
            Float32, 'current_angle', self.current_angle_callback, 10)
        self.target_angle_sub = self.create_subscription(
            Float32, 'target_angle', self.target_angle_callback, 10)

        # สร้าง Timer สำหรับ Control Loop (ทำงานที่ 200 Hz)
        self.control_timer = self.create_timer(0.005, self.control_loop)
        self.get_logger().info("Ready to receive target commands on /target_angle")

    def current_angle_callback(self, msg):
        self.current_angle = msg.data

    def target_angle_callback(self, msg):
        self.target_angle = msg.data
        self.is_moving = True
        self.get_logger().info(f"New target received: {self.target_angle:.2f}°")
        self.driver.enable()

    def control_loop(self):
        if not self.is_moving:
            return

        error = self.target_angle - self.current_angle
        error = (error + 180) % 360 - 180

        if abs(error) <= self.config['tolerance_deg']:
            self.get_logger().info(f"Target reached! Final error: {error:.2f}°")
            self.is_moving = False
            self.driver.disable()
            return

        # ตรรกะควบคุมความเร็ว
        if abs(error) > self.config['deceleration_zone_deg']:
            speed_factor = 1.0
        else:
            speed_factor = abs(error) / self.config['deceleration_zone_deg']

        delay = self.config['max_speed_delay'] + (1 - speed_factor) * (self.config['min_speed_delay'] - self.config['max_speed_delay'])

        if error > 0:
            self.driver.step('CW', delay)
        else:
            self.driver.step('CCW', delay)

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
