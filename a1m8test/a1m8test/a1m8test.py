import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LidarParser(Node):

    def __init__(self):
        super().__init__('lidar_parser')
        self.get_logger().info('Lidar Parser Node has been started.')

        # Set the target angle in degrees. Note: 180 and -180 are the same point
        self.target_angle_deg = 180.0

        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.get_logger().info(f'Target angle is set to: {self.target_angle_deg} degrees')

    def listener_callback(self, msg):
        self.get_logger().info('--- New Scan Data Received ---')

        # Convert the target angle to radians
        # Note: We need to handle the case of -180 and 180 degrees
        target_angle_rad = math.radians(self.target_angle_deg)

        # Calculate the index for the target angle
        index = int((target_angle_rad - msg.angle_min) / msg.angle_increment)

        # Check if the calculated index is within the valid range of the data array
        if 0 <= index < len(msg.ranges):
            range_m = msg.ranges[index]

            # Check for valid range data
            if range_m > msg.range_min and range_m < msg.range_max:
                range_mm = range_m * 1000
                self.get_logger().info(
                    'Angle: {:.2f} degrees, Distance: {:.2f} mm'.format(
                        self.target_angle_deg, range_mm))
            else:
                self.get_logger().warn(f'Invalid range data at target angle {self.target_angle_deg} degrees.')
        else:
            self.get_logger().warn(f'Could not find a valid index for target angle {self.target_angle_deg} degrees.')

def main(args=None):
    rclpy.init(args=args)
    lidar_parser = LidarParser()
    rclpy.spin(lidar_parser)
    lidar_parser.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
