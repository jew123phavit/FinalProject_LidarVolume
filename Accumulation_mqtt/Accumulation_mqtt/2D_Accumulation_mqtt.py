import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
import json
import numpy as np
from sensor_msgs.msg import LaserScan
from skimage.draw import line # Library ช่วยวาดเส้นตรง

# --- การตั้งค่า ---
MAP_SIZE_PIXELS = 250       # ขนาดของแผนที่ (pixels)
MAP_RESOLUTION = 0.025      # ความละเอียดของแผนที่ (เมตร/pixel) -> 5cm per pixel
MQTT_BROKER = 'localhost'
MQTT_PORT = 1883
MQTT_TOPIC_MAP = 'mapping/grid'

class MapAccumulatorNode(Node):

    def __init__(self):
        super().__init__('map_accumulator_node')
        self.get_logger().info('Map Accumulator Node has been started.')

        # 1. สร้างแผนที่เปล่า (Occupancy Grid)
        # -1 = Unknown, 0 = Free, 100 = Occupied
        self.grid_map = np.full((MAP_SIZE_PIXELS, MAP_SIZE_PIXELS), -1, dtype=np.int8)
        self.map_origin_pixels = MAP_SIZE_PIXELS // 2 # จุดศูนย์กลางของแผนที่

        # 2. Setup MQTT Client
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
        self.mqtt_client.loop_start()

        # 3. Setup ROS 2 Subscription
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # 4. ตั้งเวลาส่งข้อมูลแผนที่ทุกๆ 1 วินาที
        self.create_timer(1.0, self.publish_map_callback)

    def scan_callback(self, msg):
        # จุดศูนย์กลางของ Lidar (ในพิกัด pixel ของแผนที่)
        lidar_px = self.map_origin_pixels
        lidar_py = self.map_origin_pixels

        # วนลูปข้อมูลทุกจุดใน scan
        for i, distance in enumerate(msg.ranges):
            # กรองข้อมูลที่ไม่ถูกต้องออก
            if not (msg.range_min < distance < msg.range_max):
                continue

            angle = msg.angle_min + i * msg.angle_increment

            # แปลง Polar (angle, distance) เป็น Cartesian (x, y) ในหน่วยเมตร
            x_m = distance * np.cos(angle)
            y_m = distance * np.sin(angle)

            # แปลงจากเมตรเป็นพิกัด pixel บนแผนที่
            # แกน y ของ numpy array จะกลับด้านกับแกน y ทางคณิตศาสตร์
            endpoint_px = int(lidar_px + y_m / MAP_RESOLUTION)
            endpoint_py = int(lidar_py + x_m / MAP_RESOLUTION)

            # ตรวจสอบว่าจุดอยู่ในขอบเขตของแผนที่หรือไม่
            if 0 <= endpoint_px < MAP_SIZE_PIXELS and 0 <= endpoint_py < MAP_SIZE_PIXELS:
                # ใช้อัลกอริทึมวาดเส้นตรงเพื่อระบุพื้นที่ว่าง (Free space)
                rr, cc = line(lidar_px, lidar_py, endpoint_px, endpoint_py)
                self.grid_map[rr, cc] = 0 # 0 = Free

                # ระบุว่าจุดที่สแกนเจอเป็นสิ่งกีดขวาง (Occupied)
                self.grid_map[endpoint_px, endpoint_py] = 100 # 100 = Occupied

    def publish_map_callback(self):
        map_data = {
            'width': MAP_SIZE_PIXELS,
            'height': MAP_SIZE_PIXELS,
            'data': self.grid_map.flatten().tolist() # แปลง array 2D เป็น list ยาวๆ
        }
        payload = json.dumps(map_data)
        self.mqtt_client.publish(MQTT_TOPIC_MAP, payload)
        #self.get_logger().info('Published accumulated map to MQTT.')

def main(args=None):
    rclpy.init(args=args)
    node = MapAccumulatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()