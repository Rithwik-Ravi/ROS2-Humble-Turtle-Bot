import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class GapFollower(Node):
    def __init__(self):
        super().__init__('gap_follower_node')

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        self.cmd = Twist()
        self.linear_speed = 0.2
        self.gap_min_width = 122  # ~61 degrees → ~8 unit wide gap at 7m
        self.target_distance = 7.0

    def lidar_callback(self, msg: LaserScan):
        ranges = list(msg.ranges)

        # Remove inf/nan and cap max range
        for i in range(len(ranges)):
            if math.isinf(ranges[i]) or math.isnan(ranges[i]):
                ranges[i] = msg.range_max

        gap_indices = self.find_gap_indices(ranges, self.target_distance, self.gap_min_width)

        if gap_indices:
            # Target gap: go to its center angle
            center_idx = (gap_indices[0] + gap_indices[1]) // 2
            angle = (center_idx - 180) * 0.5 * math.pi / 180.0  # Convert to radians

            self.cmd.linear.x = self.linear_speed
            self.cmd.angular.z = -angle  # Negative to rotate toward left for +ve angle

            self.get_logger().info(f"Gap found at {center_idx}, angle: {math.degrees(angle):.2f}°")
        else:
            # No gap found — rotate in place
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.8
            self.get_logger().info("No gap found, rotating to search...")

        self.publisher.publish(self.cmd)

    def find_gap_indices(self, ranges, target_dist, min_gap_width):
        # Scan through ranges to find a sequence of values close to target_dist
        threshold = 0.5  # Acceptable +/- deviation from target distance
        start = None

        for i in range(len(ranges)):
            if abs(ranges[i] - target_dist) <= threshold:
                if start is None:
                    start = i
            else:
                if start is not None:
                    if i - start >= min_gap_width:
                        return (start, i - 1)
                    start = None

        # Edge case: gap till end
        if start is not None and len(ranges) - start >= min_gap_width:
            return (start, len(ranges) - 1)

        return None

def main(args=None):
    rclpy.init(args=args)
    node = GapFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
