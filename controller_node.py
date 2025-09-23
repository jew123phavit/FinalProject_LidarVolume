import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from gpiozero import DigitalOutputDevice
import time
from rcl_interfaces.msg import SetParametersResult

class StepperDriver:
    # ... (คลาส StepperDriver เหมือนเดิมทุกประการ ไม่ต้องแก้ไข) ...
    def __init__(self, config):
        self.config = config
        self.step_pin = DigitalOutputDevice(config["step_pin"])
        self.dir_pin = DigitalOutputDevice(config["dir_pin"])
        self.enable_pin = DigitalOutputDevice(config["enable_pin"])
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
        self.get_logger().info("Controller Node Started with new features!")

        # 1. ประกาศ Parameters ทั้งหมดที่เราจะใช้
        self.declare_parameters(
            namespace='',
            parameters=[
                ('step_pin', 14),
                ('dir_pin', 15),
                ('enable_pin', 18),
                ('base_steps_per_rev', 200),
                ('microstepping', 8),
                ('direction_cw', 1),
                ('direction_ccw', 0),
                ('target_offset_deg', 0.0), # <-- ใหม่
                ('max_velocity_deg_per_sec', 90.0), # <-- ใหม่
                ('min_velocity_deg_per_sec', 10.0), # <-- ใหม่
                ('tolerance_deg', 0.5),
                ('deceleration_zone_deg', 20.0)
            ])

        # สร้าง Dictionary เก็บค่า Config
        self.config = {}
        self.update_config() # ดึงค่าล่าสุดมาใช้

        # สร้าง Stepper Driver
        self.driver = StepperDriver(self.config)

        # ตัวแปรสถานะ
        self.current_angle = 0.0
        self.target_angle = 0.0
        self.is_moving = False

        # 2. สร้าง Subscribers
        # Subscriber ตัวเดิม สำหรับรับค่าองศาจาก Encoder
        self.current_angle_sub = self.create_subscription(
            Float32, 'current_angle', self.current_angle_callback, 10)
        # Subscriber ตัวเดิม สำหรับรับเป้าหมายแบบ "องศาสัมบูรณ์" (Absolute)
        self.target_angle_sub = self.create_subscription(
            Float32, 'target_angle', self.target_angle_callback, 10)
        
        # !! Subscriber ตัวใหม่ !! สำหรับสั่ง "หมุนไปอีก X องศา" (Relative)
        # นี่คือวิธีที่เราจะ "กำหนดทิศทาง" การหมุนได้
        self.move_relative_sub = self.create_subscription(
            Float32, 'move_relative', self.move_relative_callback, 10)

        # 3. สร้าง Timer สำหรับ Control Loop
        self.control_timer = self.create_timer(0.005, self.control_loop)

        # 4. !! ส่วนสำคัญ !! เพิ่ม Parameter Callback
        # ทำให้เราสามารถเปลี่ยนค่า offset, velocity ด้วยคำสั่ง `ros2 param set` ได้ทันที
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        self.get_logger().info("Ready. Use '/target_angle' for absolute moves or '/move_relative' for relative moves.")
        self.get_logger().info(f"Initial Max Speed Delay: {self.config['max_speed_delay']:.6f} s")

    def velocity_to_delay(self, velocity_deg_s):
        """แปลงความเร็ว (deg/s) เป็นค่า delay (s) สำหรับ driver"""
        if velocity_deg_s <= 0:
            return float('inf') # ความเร็วเป็น 0 หรือติดลบ -> delay เป็นอนันต์ (หยุด)
        
        steps_per_rev = self.config['base_steps_per_rev'] * self.config['microstepping']
        deg_per_step = 360.0 / steps_per_rev
        steps_per_sec = velocity_deg_s / deg_per_step
        # 1 step cycle = step_pin on -> sleep(delay) -> step_pin off -> sleep(delay)
        # So, time_per_step = 2 * delay
        # delay = time_per_step / 2 = (1 / steps_per_sec) / 2
        delay = 1.0 / (2.0 * steps_per_sec)
        return delay

    def update_config(self):
        """ดึงค่าพารามิเตอร์ทั้งหมดและคำนวณค่าที่จำเป็น"""
        params = self.get_parameters([
            'step_pin', 'dir_pin', 'enable_pin', 'base_steps_per_rev', 'microstepping',
            'direction_cw', 'direction_ccw', 'target_offset_deg', 'max_velocity_deg_per_sec',
            'min_velocity_deg_per_sec', 'tolerance_deg', 'deceleration_zone_deg'
        ])
        for param in params:
            self.config[param.name] = param.value
        
        # คำนวณค่า delay จาก velocity ที่ตั้งไว้
        self.config['max_speed_delay'] = self.velocity_to_delay(self.config['max_velocity_deg_per_sec'])
        self.config['min_speed_delay'] = self.velocity_to_delay(self.config['min_velocity_deg_per_sec'])

    def parameters_callback(self, params):
        """ฟังก์ชันที่จะถูกเรียกเมื่อมีการ `ros2 param set`"""
        self.get_logger().info("Parameter change detected! Updating configuration...")
        # อัปเดต config ทั้งหมดแล้วคำนวณ delay ใหม่
        self.update_config()
        self.get_logger().info(f"New Max Speed Delay: {self.config['max_speed_delay']:.6f} s")
        self.get_logger().info(f"New Target Offset: {self.config['target_offset_deg']:.2f} deg")
        return SetParametersResult(successful=True)

    def current_angle_callback(self, msg):
        self.current_angle = msg.data

    def target_angle_callback(self, msg):
        """Callback สำหรับการเคลื่อนที่ไปยังองศาสัมบูรณ์"""
        # นำค่า offset มาคำนวณกับเป้าหมาย
        self.target_angle = (msg.data + self.config['target_offset_deg']) % 360.0
        self.is_moving = True
        self.get_logger().info(f"New ABSOLUTE target: {self.target_angle:.2f}° (raw: {msg.data:.2f}°, offset: {self.config['target_offset_deg']:.2f}°)")
        self.driver.enable()

    def move_relative_callback(self, msg):
        """Callback สำหรับการเคลื่อนที่ไปอีก X องศา (กำหนดทิศทางได้)"""
        relative_angle = msg.data
        # ค่าบวกคือ CW, ค่าลบคือ CCW
        # คำนวณเป้าหมายใหม่จากตำแหน่งปัจจุบัน + ค่าที่สั่ง
        self.target_angle = (self.current_angle + relative_angle + self.config['target_offset_deg']) % 360.0
        self.is_moving = True
        self.get_logger().info(f"New RELATIVE move: {relative_angle:.2f}°. New target: {self.target_angle:.2f}°")
        self.driver.enable()

    def control_loop(self):
        if not self.is_moving:
            return

        error = self.target_angle - self.current_angle
        # ทำให้ error อยู่ในช่วง -180 ถึง 180 (เลือกทางที่ใกล้ที่สุด)
        error = (error + 180) % 360 - 180

        if abs(error) <= self.config['tolerance_deg']:
            self.get_logger().info(f"Target reached! Final error: {error:.2f}°")
            self.is_moving = False
            self.driver.disable()
            return

        # P-Control Logic (เหมือนเดิม แต่ใช้ค่า delay ที่คำนวณจาก velocity)
        decel_zone = self.config['deceleration_zone_deg']
        if abs(error) > decel_zone:
            speed_factor = 1.0
        else:
            speed_factor = abs(error) / decel_zone

        # หน่วงความเร็วต่ำสุดให้มีความเร็วจริงๆ ไม่ใช่ 0
        min_speed_factor = 0.1 # ป้องกันการหยุดนิ่งเมื่อเข้าใกล้เป้าหมาย
        speed_factor = max(speed_factor, min_speed_factor)
        
        delay = self.config['max_speed_delay'] + (1 - speed_factor) * (self.config['min_speed_delay'] - self.config['max_speed_delay'])

        # กำหนดทิศทางจากเครื่องหมายของ error
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
