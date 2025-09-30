import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
import time
import os

# ปรับปรุง matplotlib เล็กน้อยเพื่อไม่ให้มีปัญหาตอนรันบน SSH หรือ Server
plt.switch_backend('Agg')

class PerformancePlotter(Node):
    def __init__(self):
        super().__init__('performance_plotter')
        self.get_logger().info("Performance Plotter Node Started. Listening for data...")

        # --- ตัวแปรสำหรับเก็บข้อมูล ---
        self.start_time = time.time()
        self.timestamps = []
        self.current_angles = []
        self.target_angles = []
        self.errors = []
        self.last_target_angle = 0.0

        # --- สร้าง Subscribers ---
        self.create_subscription(Float32, '/current_angle', self.current_angle_callback, 10)
        self.create_subscription(Float32, '/target_angle', self.target_angle_callback, 10)
        self.create_subscription(Float32, '/control_error', self.error_callback, 10)

        # --- สั่งให้ฟังก์ชัน save_plot ทำงานตอนที่เรากด Ctrl+C ---
        rclpy.get_default_context().on_shutdown(self.save_plot)

    def current_angle_callback(self, msg):
        elapsed_time = time.time() - self.start_time
        self.timestamps.append(elapsed_time)
        self.current_angles.append(msg.data)
        self.target_angles.append(self.last_target_angle)
        
    def target_angle_callback(self, msg):
        self.last_target_angle = msg.data

    def error_callback(self, msg):
        self.errors.append(msg.data)

    def save_plot(self):
        self.get_logger().info("Shutdown signal received. Generating and saving plot...")
        
        if not self.timestamps or not self.current_angles:
            self.get_logger().warn("No data collected. Skipping plot generation.")
            return

        # --- ส่วนที่แก้ไข: สร้าง 3 กราฟย่อยในแนวตั้ง ---
        # เราเปลี่ยนจาก (2, 1) เป็น (3, 1) และเพิ่ม ax3 เข้ามา
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 15)) # เพิ่มความสูงของรูป
        fig.suptitle('Stepper Motor Performance Analysis', fontsize=16)

        # --- กราฟที่ 1: Target Angle vs Current Angle (เหมือนเดิม) ---
        ax1.plot(self.timestamps, self.target_angles, label='Target Angle', color='blue', linestyle='--')
        ax1.plot(self.timestamps, self.current_angles, label='Current Angle', color='green', alpha=0.8)
        ax1.set_title('Angle Tracking Performance')
        ax1.set_xlabel('Time (seconds)')
        ax1.set_ylabel('Angle (degrees)')
        ax1.legend()
        ax1.grid(True)

        # --- กราฟที่ 2: Control Error Over Time (เหมือนเดิม) ---
        # เราต้องสร้างแกนเวลาสำหรับ error แยกต่างหาก เพราะอาจมีจำนวนไม่เท่ากัน
        error_timestamps = self.timestamps[:len(self.errors)]
        ax2.plot(error_timestamps, self.errors, label='Control Error', color='red')
        ax2.set_title('Control Error Over Time')
        ax2.set_xlabel('Time (seconds)')
        ax2.set_ylabel('Error (degrees)')
        ax2.axhline(0, color='black', linewidth=0.5, linestyle='--')
        ax2.legend()
        ax2.grid(True)
        
        # --- กราฟที่ 3: Target Angle vs Error (เพิ่มใหม่) ---
        # กราฟนี้จะแสดงให้เห็นว่า Error เปลี่ยนแปลงไปอย่างไรเมื่อเทียบกับ Target
        ax3.plot(self.timestamps, self.target_angles, label='Target Angle', color='blue', linestyle='--')
        # (สำคัญ) เราใช้ self.errors ไม่ใช่ self.current_angles และใช้ error_timestamps
        ax3.plot(error_timestamps, self.errors, label='Error Angle', color='red', alpha=0.8)
        ax3.set_title('Target Angle vs. Error Angle')
        ax3.set_xlabel('Time (seconds)')
        ax3.set_ylabel('Angle / Error (degrees)')
        ax3.legend()
        ax3.grid(True)
        
        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        
        try:
            filename = os.path.expanduser('~/performance_plot.png')
            plt.savefig(filename)
            self.get_logger().info(f" Plot successfully saved to '{filename}' ")
        except Exception as e:
            self.get_logger().error(f"Failed to save plot: {e}")
        
        plt.close()

def main(args=None):
    rclpy.init(args=args)
    plotter_node = PerformancePlotter()
    try:
        rclpy.spin(plotter_node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            plotter_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()