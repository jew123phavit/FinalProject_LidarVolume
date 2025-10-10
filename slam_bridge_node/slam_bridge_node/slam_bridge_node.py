import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
import json
import numpy as np

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# --- MQTT Settings ---
MQTT_BROKER = 'localhost'
MQTT_PORT = 1883

class SlamMqttBridge(Node):

    def __init__(self):
        super().__init__('slam_mqtt_bridge')
        self.get_logger().info('SLAM to MQTT Bridge has been started.')

        # 1. Setup MQTT Client
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
        self.mqtt_client.loop_start()

        # 2. Setup ROS 2 Subscriptions
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # 3. Setup TF2 Listener to get the pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.1, self.tf_callback) # Check for TF every 100ms

    def map_callback(self, msg):
        map_data = {
            'width': msg.info.width,
            'height': msg.info.height,
            'resolution': msg.info.resolution,
            'origin_x': msg.info.origin.position.x,
            'origin_y': msg.info.origin.position.y,
            'data': list(msg.data)
        }
        payload = json.dumps(map_data)
        self.mqtt_client.publish('slam/map', payload)
        self.get_logger().info('Published map update to MQTT.')

    def scan_callback(self, msg):
        scan_data = {
            'angle_min': msg.angle_min,
            'angle_increment': msg.angle_increment,
            'ranges': [r if r > msg.range_min and r < msg.range_max else 0.0 for r in msg.ranges]
        }
        payload = json.dumps(scan_data)
        self.mqtt_client.publish('slam/scan', payload)

    def tf_callback(self):
        try:
            # Get the latest transform from the map frame to the laser's frame
            trans = self.tf_buffer.lookup_transform('map', 'laser', rclpy.time.Time())

            # Extract position and orientation (yaw)
            q = trans.transform.rotation
            # Simple conversion from quaternion to yaw (2D rotation)
            yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

            pose_data = {
                'x': trans.transform.translation.x,
                'y': trans.transform.translation.y,
                'theta': yaw # in radians
            }
            payload = json.dumps(pose_data)
            self.mqtt_client.publish('slam/pose', payload)
        except TransformException as ex:
            # self.get_logger().warn(f'Could not transform map to laser: {ex}')
            pass

def main(args=None):
    rclpy.init(args=args)
    bridge_node = SlamMqttBridge()
    rclpy.spin(bridge_node)
    bridge_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()