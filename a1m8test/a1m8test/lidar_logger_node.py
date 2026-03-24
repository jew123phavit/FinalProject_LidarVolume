import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import csv
import os
from datetime import datetime
import time # สำหรับ timestamp ที่ละเอียดขึ้น

class LidarLoggerNode(Node):

    def __init__(self):
        super().__init__('lidar_logger_node')
        self.get_logger().info('Lidar Logger Node has been started.')

        # --- ถามองศาจากผู้ใช้ ---
        self.target_angle_deg = self._get_target_angle_from_user()
        self.target_angle_rad = math.radians(self.target_angle_deg)
        self.get_logger().info(f'Target angle set to: {self.target_angle_deg:.2f} degrees')

        # --- ตั้งค่า CSV ---
        self.csv_filename = self._generate_csv_filename()
        self.csv_file = open(self.csv_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        # เขียน Header
        self.csv_writer.writerow(['Timestamp (s)', 'Target Angle (deg)', 'Measured Distance (m)', 'Valid'])
        self.get_logger().info(f"Logging data to: {self.csv_filename}")

        # --- สร้าง Subscriber ---
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan', # Subscribe topic '/scan' เหมือนเดิม
            self.listener_callback,
            10) # QoS profile depth

        # --- จัดการการปิดไฟล์ CSV ---
        # ใช้ context manager หรือ atexit อาจจะดีกว่า แต่ on_shutdown ก็ใช้ได้
        rclpy.get_default_context().on_shutdown(self.cleanup)

    def _get_target_angle_from_user(self):
        """ถามองศาจากผู้ใช้และตรวจสอบความถูกต้อง"""
        while True:
            try:
                angle_str = input("Enter the target angle in degrees (-180 to 180): ")
                angle_deg = float(angle_str)
                # อนุญาตให้ใช้มุมได้ตั้งแต่ -180 ถึง 180
                if -180.0 <= angle_deg <= 180.0:
                    return angle_deg
                else:
                    print("Angle out of range. Please enter a value between -180 and 180.")
            except ValueError:
                print("Invalid input. Please enter a number.")
            except EOFError:
                 self.get_logger().info("EOF received, shutting down.")
                 # หากรันใน script อาจจะเจอ EOF ถ้าไม่มี input ต่อ
                 # อาจจะต้องพิจารณาการ shutdown node อย่างเหมาะสม
                 rclpy.shutdown()
                 sys.exit(0) # ออกจากโปรแกรม


    def _generate_csv_filename(self):
        """สร้างชื่อไฟล์ CSV ที่ไม่ซ้ำกัน"""
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        # ใช้ os.path.expanduser('~') เพื่อไปยัง home directory
        filename = os.path.expanduser(f'~/lidar_log_angle_{self.target_angle_deg:.1f}deg_{timestamp_str}.csv')
        return filename

    def listener_callback(self, msg: LaserScan):
        """Callback เมื่อได้รับข้อมูล /scan"""
        current_time_sec = time.time() # ใช้ timestamp จาก time.time() เพื่อความละเอียด

        # คำนวณ Index ที่ใกล้เคียงกับมุมเป้าหมายที่สุด
        # angle_min และ angle_max ปกติอยู่ในหน่วย radian และอยู่ในช่วง [-pi, pi]
        # target_angle_rad ก็อยู่ในช่วง [-pi, pi]
        # ระวัง: msg.angle_increment อาจเป็นค่าลบได้ในบาง LiDAR
        try:
            # คำนวณ index จากมุมเริ่มต้นและ increment
            # ต้องจัดการกรณี angle_increment เป็น 0 หรือใกล้ 0
            if abs(msg.angle_increment) < 1e-6:
                 index = 0 # หรือจัดการแบบอื่นตาม spec ของ LiDAR
            else:
                 # คำนวณ index โดยตรง
                 index_float = (self.target_angle_rad - msg.angle_min) / msg.angle_increment
                 # เลือก index ที่ใกล้ที่สุด (ปัดเศษ)
                 index = int(round(index_float))

            # ตรวจสอบว่า index อยู่ในขอบเขตหรือไม่
            if 0 <= index < len(msg.ranges):
                distance_m = msg.ranges[index]
                is_valid = False # ตั้งค่าเริ่มต้น

                # ตรวจสอบความถูกต้องของระยะทาง
                if msg.range_min < distance_m < msg.range_max:
                    is_valid = True
                    # อาจจะ print log แค่บางครั้งเพื่อไม่ให้รก terminal
                    # self.get_logger().info(f'Angle: {self.target_angle_deg:.2f} deg, Distance: {distance_m:.3f} m', throttle_duration_sec=1.0)
                else:
                    distance_m = -1.0 # ใส่ค่า default สำหรับข้อมูลที่ไม่ถูกต้อง
                    # self.get_logger().warn(f'Invalid range data ({msg.ranges[index]:.3f}) at index {index} for target angle {self.target_angle_deg:.2f} deg', throttle_duration_sec=1.0)

                # เขียนข้อมูลลง CSV
                self.csv_writer.writerow([
                    f"{current_time_sec:.4f}",
                    f"{self.target_angle_deg:.2f}",
                    f"{distance_m:.4f}" if is_valid else "INVALID", # แสดง INVALID หรือค่าตัวเลข
                    "Yes" if is_valid else "No"
                ])
                self.csv_file.flush() # บันทึกข้อมูลลงไฟล์ทันที (อาจลดประสิทธิภาพเล็กน้อย)

            else:
                # กรณี Index อยู่นอกขอบเขต (ไม่ควรเกิดถ้า target_angle_rad อยู่ใน [angle_min, angle_max])
                # self.get_logger().warn(f'Calculated index {index} is out of range [0, {len(msg.ranges)-1}] for target angle {self.target_angle_deg:.2f} deg.', throttle_duration_sec=1.0)
                # เขียน log ว่าหา index ไม่เจอ
                 self.csv_writer.writerow([
                    f"{current_time_sec:.4f}",
                    f"{self.target_angle_deg:.2f}",
                    "INDEX_OOR", # Index Out Of Range
                    "No"
                ])
                 self.csv_file.flush()

        except Exception as e:
            self.get_logger().error(f"Error in listener_callback: {e}")
            # อาจจะบันทึก error ลง CSV ด้วยก็ได้
            self.csv_writer.writerow([
                f"{current_time_sec:.4f}",
                f"{self.target_angle_deg:.2f}",
                "ERROR",
                "No"
            ])
            self.csv_file.flush()


    def cleanup(self):
        """ปิดไฟล์ CSV เมื่อ Node หยุดทำงาน"""
        self.get_logger().info("Shutting down and closing CSV file.")
        if self.csv_file and not self.csv_file.closed:
            self.csv_file.close()
            self.get_logger().info(f"CSV file '{self.csv_filename}' closed.")

def main(args=None):
    rclpy.init(args=args)
    lidar_logger_node = None
    try:
        lidar_logger_node = LidarLoggerNode()
        rclpy.spin(lidar_logger_node)
    except KeyboardInterrupt:
        pass # ปล่อยให้ cleanup ทำงาน
    except Exception as e:
        if lidar_logger_node:
            lidar_logger_node.get_logger().fatal(f"Unhandled exception: {e}")
        else:
            print(f"Unhandled exception during node initialization: {e}")
    finally:
        # Cleanup ทำงานผ่าน on_shutdown callback แล้ว ไม่ต้องเรียกซ้ำ
        if lidar_logger_node and rclpy.ok():
             # อาจจะเรียก destroy_node ที่นี่เพื่อให้แน่ใจ แต่ cleanup ควรจัดการไฟล์แล้ว
             lidar_logger_node.destroy_node()
        if rclpy.ok():
             rclpy.shutdown()

if __name__ == '__main__':
    main()