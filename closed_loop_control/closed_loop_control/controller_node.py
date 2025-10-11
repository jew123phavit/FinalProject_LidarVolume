#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float32, String
from gpiozero import DigitalOutputDevice

class StepperDriver:
    def __init__(self, cfg):
        self.cfg = cfg
        self.step_pin = DigitalOutputDevice(cfg["step_pin"])
        self.dir_pin = DigitalOutputDevice(cfg["dir_pin"])
        self.enable_pin = DigitalOutputDevice(cfg["enable_pin"])
        self.disable()  # ENA high (ปิด)

    def enable(self):
        self.enable_pin.off()  # ENA low (เปิด)

    def disable(self):
        self.enable_pin.on()

    def _set_direction(self, direction: str):
        if direction.upper() == 'CW':
            self.dir_pin.value = bool(self.cfg["direction_cw"])
        else:  # 'CCW'
            self.dir_pin.value = bool(self.cfg["direction_ccw"])

    def step(self, direction: str, delay_s: float):
        self._set_direction(direction)
        self.step_pin.on()
        time.sleep(delay_s)
        self.step_pin.off()
        time.sleep(delay_s)

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.get_logger().info("controller_node started.")

        # (1) พารามิเตอร์หลัก (กำหนดค่าเริ่มต้น)
        self.declare_parameters('', [
            ('step_pin', 14),
            ('dir_pin', 15),
            ('enable_pin', 18),
            ('base_steps_per_rev', 200),
            ('microstepping', 8),
            ('direction_cw', 1),
            ('direction_ccw', 0),

            ('offset_deg', 0.0),  # ชดเชยมุมรวม

            ('max_velocity_deg_per_sec', 100.0),  # เร็วสุด
            ('min_velocity_deg_per_sec', 10.0),   # ช้าที่สุด
            ('tolerance_deg', 1.0),               # ระยะหยุด
            ('deceleration_zone_deg', 20.0),      # โซนชะลอ

            ('default_speed_deg_per_sec', 100.0), # ถ้า user ส่ง speed มาจะใช้ค่านั้น
            ('stop_on_crossing', False),          # หยุดเมื่อข้ามเป้าในโหมดบังคับทิศ
        ])

        # (2) โหลด config
        self.cfg = {}
        self._reload_cfg()

        # (3) สร้างไดรเวอร์ + ตัวแปรสถานะ
        self.driver = StepperDriver(self.cfg)
        self.current_angle = 0.0
        self.target_angle  = 0.0
        self.is_moving = False

        # โหมดคำสั่งจาก topic
        self.dir_mode = 'AUTO'   # 'AUTO' | 'CW' | 'CCW'
        self.cmd_speed = None    # None = ใช้ P-control

        # (4) Subscribers/Publishers
        self.create_subscription(Float32, 'current_angle', self._on_current, 10)
        self.create_subscription(Float32, 'target_angle',  self._on_target,  10)
        self.create_subscription(String,  'target_dir',    self._on_dir,     10)
        self.create_subscription(Float32, 'target_speed',  self._on_speed,   10)

        self.pub_error   = self.create_publisher(Float32, 'control_error', 10)
        self.pub_reached = self.create_publisher(Float32, 'motion_reached', 10)

        # (5) Control loop 200 Hz
        self.create_timer(0.005, self._loop)

        # (6) เปิด parameter callback
        self.add_on_set_parameters_callback(self._param_cb)

    # ---------- utilities ----------
    def _vel_to_delay(self, vel_deg_s: float) -> float:
        if vel_deg_s <= 0.0:
            return float('inf')
        steps_per_rev = self.cfg['base_steps_per_rev'] * self.cfg['microstepping']
        deg_per_step  = 360.0 / steps_per_rev
        steps_per_sec = vel_deg_s / deg_per_step
        return 1.0 / (2.0 * steps_per_sec)  # 1 cycle = 2*delay

    def _reload_cfg(self):
        names = [
            'step_pin','dir_pin','enable_pin',
            'base_steps_per_rev','microstepping',
            'direction_cw','direction_ccw',
            'offset_deg',
            'max_velocity_deg_per_sec','min_velocity_deg_per_sec',
            'tolerance_deg','deceleration_zone_deg',
            'default_speed_deg_per_sec','stop_on_crossing'
        ]
        for n in names:
            self.cfg[n] = self.get_parameter(n).value
        # pre-compute delay bounds
        self.cfg['delay_max'] = self._vel_to_delay(self.cfg['max_velocity_deg_per_sec'])
        self.cfg['delay_min'] = self._vel_to_delay(self.cfg['min_velocity_deg_per_sec'])

    # ---------- parameter callback ----------
    def _param_cb(self, params):
        new = dict(self.cfg)
        for p in params:
            name, t, v = p.name, p.type_, p.value
            def must_num():
                return t in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER)

            if name == 'offset_deg':
                if not must_num(): return SetParametersResult(False, "offset_deg must be number")
                if not (-360.0 <= float(v) <= 360.0):
                    return SetParametersResult(False, "offset_deg in [-360,360]")
                new[name] = float(v)

            elif name in ('max_velocity_deg_per_sec','min_velocity_deg_per_sec'):
                if not must_num(): return SetParametersResult(False, f"{name} must be number")
                if float(v) <= 0.0: return SetParametersResult(False, f"{name} > 0")
                new[name] = float(v)

            elif name == 'tolerance_deg':
                if not must_num(): return SetParametersResult(False, "tolerance_deg must be number")
                if not (0 < float(v) <= 5.0):
                    return SetParametersResult(False, "tolerance_deg in (0,5]")
                new[name] = float(v)

            elif name == 'deceleration_zone_deg':
                if not must_num(): return SetParametersResult(False, "deceleration_zone_deg must be number")
                if not (1.0 <= float(v) <= 90.0):
                    return SetParametersResult(False, "deceleration_zone_deg in [1,90]")
                new[name] = float(v)

            elif name == 'default_speed_deg_per_sec':
                if not must_num(): return SetParametersResult(False, "default_speed_deg_per_sec must be number")
                if float(v) <= 0.0:
                    return SetParametersResult(False, "default_speed_deg_per_sec > 0")
                new[name] = float(v)

            elif name in ('step_pin','dir_pin','enable_pin','base_steps_per_rev','microstepping','direction_cw','direction_ccw'):
                if not must_num(): return SetParametersResult(False, f"{name} must be number")
                new[name] = int(v)

            elif name == 'stop_on_crossing':
                if t != Parameter.Type.BOOL: return SetParametersResult(False, "stop_on_crossing must be bool")
                new[name] = bool(v)

        # cross check
        if new['max_velocity_deg_per_sec'] < new['min_velocity_deg_per_sec']:
            return SetParametersResult(False, "max_velocity < min_velocity")

        self.cfg.update(new)
        self.cfg['delay_max'] = self._vel_to_delay(self.cfg['max_velocity_deg_per_sec'])
        self.cfg['delay_min'] = self._vel_to_delay(self.cfg['min_velocity_deg_per_sec'])
        self.get_logger().info(
            f"params: offset={self.cfg['offset_deg']:.2f}, vmax={self.cfg['max_velocity_deg_per_sec']:.1f}, "
            f"vmin={self.cfg['min_velocity_deg_per_sec']:.1f}, tol={self.cfg['tolerance_deg']:.2f}, "
            f"decel={self.cfg['deceleration_zone_deg']:.1f}"
        )
        return SetParametersResult(successful=True)

    # ---------- topic callbacks ----------
    def _on_current(self, msg: Float32):
        self.current_angle = msg.data

    def _on_target(self, msg: Float32):
        self.target_angle = (msg.data + self.cfg['offset_deg']) % 360.0
        self.is_moving = True
        self.driver.enable()
        self.get_logger().info(f"target={self.target_angle:.2f}° (raw={msg.data:.2f}°, offset={self.cfg['offset_deg']:.2f}°)")

    def _on_dir(self, msg: String):
        val = msg.data.strip().upper()
        if val in ('AUTO','CW','CCW'):
            self.dir_mode = val
            self.get_logger().info(f"dir={self.dir_mode}")
        else:
            self.get_logger().warn(f"unknown dir '{msg.data}', keep {self.dir_mode}")

    def _on_speed(self, msg: Float32):
        self.cmd_speed = float(msg.data) if msg.data > 0.0 else None
        if self.cmd_speed:
            self.get_logger().info(f"speed={self.cmd_speed:.1f} deg/s (manual)")
        else:
            self.get_logger().info("speed=auto (P-control)")

    # ---------- main control loop ----------
    def _loop(self):
        if not self.is_moving:
            return

        # error ∈ [-180,180]
        error = ((self.target_angle - self.current_angle + 180.0) % 360.0) - 180.0
        self.pub_error.publish(Float32(data=float(error)))

        # ถึงเป้าหมายแล้ว -> หยุดทันที
        if abs(error) <= self.cfg['tolerance_deg']:
            self.is_moving = False
            self.pub_reached.publish(Float32(data=float(self.current_angle)))
            self.driver.disable()
            self.get_logger().info(f"reached (error={error:.2f}°)")
            return

        # เลือกทิศ
        if self.dir_mode == 'AUTO':
            direction = 'CW' if error > 0 else 'CCW'
        else:
            direction = self.dir_mode
            # ถ้าบังคับทิศและอยากหยุดเมื่อ "ข้าม" เป้าหมาย
            if self.cfg['stop_on_crossing']:
                going_positive = (direction == 'CW')
                if going_positive and error <= 0.0:
                    self.is_moving = False
                    self.pub_reached.publish(Float32(data=float(self.current_angle)))
                    self.driver.disable()
                    self.get_logger().info("stop on crossing (CW)")
                    return
                if (not going_positive) and error >= 0.0:
                    self.is_moving = False
                    self.pub_reached.publish(Float32(data=float(self.current_angle)))
                    self.driver.disable()
                    self.get_logger().info("stop on crossing (CCW)")
                    return

        # ความเร็ว → delay
        if self.cmd_speed and self.cmd_speed > 0.0:
            delay = self._vel_to_delay(self.cmd_speed)  # ความเร็วคงที่
        else:
            # P-control: ยิ่งใกล้เป้ายิ่งช้า
            decel = max(1e-6, float(self.cfg['deceleration_zone_deg']))
            s = min(1.0, max(abs(error)/decel, 0.0))
            s = max(s, 0.1)  # มีพื้นช้าไว้ 10%
            delay = self.cfg['delay_max'] + (1.0 - s) * (self.cfg['delay_min'] - self.cfg['delay_max'])

        # สั่งสเต็ป
        self.driver.step(direction, delay)

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()