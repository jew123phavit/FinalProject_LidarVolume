# controller_node.py
# — เพิ่ม direction_mode, kp/min/max_speed_dps, offset_deg, และ parameter callback —
import math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float32
from gpiozero import DigitalOutputDevice
import time

class StepperDriver:
    """Simple step/dir driver wrapper for a stepper driver."""
    def __init__(self, config):
        self.config = config
        self._init_pins_from_config(config)
        self.disable()
        print("Stepper Driver Initialized.")

    def _init_pins_from_config(self, cfg):
        self.step_pin = DigitalOutputDevice(cfg["step_pin"])
        self.dir_pin = DigitalOutputDevice(cfg["dir_pin"])
        self.enable_pin = DigitalOutputDevice(cfg["enable_pin"])

    def apply_pin_update(self, cfg):
        self.disable()
        try:
            self.step_pin.close()
            self.dir_pin.close()
            self.enable_pin.close()
        except Exception:
            pass
        self._init_pins_from_config(cfg)

    def enable(self):
        self.enable_pin.off()   # EN low = enable (ทั่วไป)

    def disable(self):
        self.enable_pin.on()    # EN high = disable

    def _set_direction(self, direction):
        if direction == 'CW':
            self.dir_pin.value = self.config["direction_cw"]
        elif direction == 'CCW':
            self.dir_pin.value = self.config["direction_ccw"]
        else:
            raise ValueError("direction must be 'CW' or 'CCW'")

    def step(self, direction, delay):
        """One step; delay = half-period (s). Full step period = 2*delay."""
        self._set_direction(direction)
        self.step_pin.on()
        time.sleep(delay)
        self.step_pin.off()
        time.sleep(delay)

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.get_logger().info("Controller Node Started")

        # Hardware & motor
        self.declare_parameter('step_pin', 14)
        self.declare_parameter('dir_pin', 15)
        self.declare_parameter('enable_pin', 18)
        self.declare_parameter('base_steps_per_rev', 200)
        self.declare_parameter('microstepping', 8)
        self.declare_parameter('direction_cw', 1)
        self.declare_parameter('direction_ccw', 0)

        # Control
        self.declare_parameter('tolerance_deg', 0.5)
        self.declare_parameter('kp', 2.0)                # deg/s per deg
        self.declare_parameter('max_speed_dps', 720.0)   # cap speed
        self.declare_parameter('min_speed_dps', 20.0)    # floor speed
        self.declare_parameter('direction_mode', 'auto_shortest')  # auto_shortest|force_cw|force_ccw
        self.declare_parameter('offset_deg', 0.0)

        self._reload_config()
        self.driver = StepperDriver(config=self.config)

        self.current_angle = 0.0
        self.target_angle = 0.0
        self.is_moving = False

        self.create_subscription(Float32, 'current_angle', self.current_angle_callback, 10)
        self.create_subscription(Float32, 'target_angle', self.target_angle_callback, 10)

        self.control_timer = self.create_timer(0.005, self.control_loop)  # 200 Hz
        self.add_on_set_parameters_callback(self._on_param_update)
        self.get_logger().info("Ready to receive target commands on /target_angle")

    def _reload_config(self):
        names = [
            'step_pin','dir_pin','enable_pin','base_steps_per_rev','microstepping',
            'direction_cw','direction_ccw',
            'tolerance_deg','kp','max_speed_dps','min_speed_dps',
            'direction_mode','offset_deg'
        ]
        self.config = {n: self.get_parameter(n).value for n in names}
        self.deg_per_step = 360.0 / (self.config['base_steps_per_rev'] * self.config['microstepping'])

    def _wrap_360(self, angle): return angle % 360.0
    def _wrap_pm180(self, angle): return (angle + 180.0) % 360.0 - 180.0

    def current_angle_callback(self, msg: Float32):
        self.current_angle = float(msg.data)

    def target_angle_callback(self, msg: Float32):
        self.target_angle = float(msg.data)
        self.is_moving = True
        self.get_logger().info(f"New target received: {self.target_angle:.2f}°")
        self.driver.enable()

    def _on_param_update(self, params):
        updates = {p.name: p.value for p in params}
        if 'direction_mode' in updates and updates['direction_mode'] not in ('auto_shortest','force_cw','force_ccw'):
            return SetParametersResult(successful=False, reason="direction_mode must be auto_shortest|force_cw|force_ccw")
        if 'kp' in updates and updates['kp'] < 0.0:
            return SetParametersResult(successful=False, reason="kp must be >= 0")
        if ('max_speed_dps' in updates and 'min_speed_dps' in updates and
            updates['max_speed_dps'] < updates['min_speed_dps']):
            return SetParametersResult(successful=False, reason="max_speed_dps must be >= min_speed_dps")

        for p in params:
            self.set_parameters([Parameter(name=p.name, value=p.value)])
        self._reload_config()

        if any(k in updates for k in ('step_pin','dir_pin','enable_pin','direction_cw','direction_ccw')):
            self.driver.config = self.config
            self.driver.apply_pin_update(self.config)

        return SetParametersResult(successful=True)

    def _compute_error_and_direction(self):
        # ชดเชย offset ที่ฝั่ง measurement
        current_eff = self._wrap_360(self.current_angle + self.config['offset_deg'])
        target = self._wrap_360(self.target_angle)
        mode = self.config['direction_mode']
        tol = self.config['tolerance_deg']

        if mode == 'auto_shortest':
            err = self._wrap_pm180(target - current_eff)
            if abs(err) <= tol:
                return 0.0, 'CW'
            direction = 'CW' if err > 0 else 'CCW'
            return abs(err), direction

        elif mode == 'force_cw':
            err_cw = (target - current_eff) % 360.0  # 0..360
            if err_cw <= tol or (360.0 - err_cw) <= tol:
                return 0.0, 'CW'
            return err_cw, 'CW'

        elif mode == 'force_ccw':
            err_ccw = (current_eff - target) % 360.0  # 0..360
            if err_ccw <= tol or (360.0 - err_ccw) <= tol:
                return 0.0, 'CCW'
            return err_ccw, 'CCW'

        # fallback
        err = self._wrap_pm180(target - current_eff)
        direction = 'CW' if err > 0 else 'CCW'
        return abs(err), direction

    def _error_to_delay(self, abs_error_deg: float) -> float:
        # P-control -> speed_dps = clamp(kp*|e|, min, max) -> steps/s -> half-period delay
        kp = self.config['kp']
        min_dps = self.config['min_speed_dps']
        max_dps = self.config['max_speed_dps']
        speed_dps = max(min(kp * abs_error_deg, max_dps), min_dps)
        steps_per_sec = max(speed_dps / self.deg_per_step, 1e-3)
        delay = 1.0 / (2.0 * steps_per_sec)
        return max(min(delay, 0.05), 1e-6)  # safety clamp

    def control_loop(self):
        if not self.is_moving:
            return
        abs_err, direction = self._compute_error_and_direction()
        if abs_err <= self.config['tolerance_deg']:
            self.get_logger().info(f"Target reached! Final error: {abs_err:.2f}°")
            self.is_moving = False
            self.driver.disable()
            return
        delay = self._error_to_delay(abs_err)
        self.driver.step(direction, delay)

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
