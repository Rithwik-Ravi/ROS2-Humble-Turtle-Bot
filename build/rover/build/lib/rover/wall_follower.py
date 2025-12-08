import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.timer = self.create_timer(0.05, self.timer_callback) # Faster update rate (20Hz)
        self.velocity = Twist()
        
        # Sensor Data
        self.front_dist = 10.0
        self.right_dist = 10.0
        
        # PID Parameters
        self.kp = 4.0           # Stronger gain for tighter wall following
        self.target_dist = 0.45 # Maintain closer distance (0.45m)
        self.speed = 0.3        # Base Forward speed

    def scan_callback(self, msg):
        ranges = msg.ranges
        size = len(ranges)
        
        # --- Helper: Robust Minimum ---
        def get_min(start_idx, end_idx):
            start_idx = max(0, start_idx)
            end_idx = min(size, end_idx)
            # Filter out bad values (0.0, inf, nan)
            slice_data = ranges[start_idx:end_idx]
            valid = [r for r in slice_data if not math.isinf(r) and not math.isnan(r) and r > 0.1]
            return min(valid) if valid else 10.0

        # --- Slicing for Depth Camera/Lidar ---
        
        # Right: 0% to 30% (Standard side view)
        # Front: 45% to 55% (Very narrow view to only see obstacles DIRECTLY ahead)
        
        right_end = int(size * 0.30)
        front_start = int(size * 0.45)
        front_end = int(size * 0.55)
        
        self.right_dist = get_min(0, right_end)
        self.front_dist = get_min(front_start, front_end)

    def timer_callback(self):
        msg = Twist()
        
        # --- LOGIC FLOW ---
        
        # 1. PANIC: Too Close! (Reverse)
        if self.front_dist < 0.25:
            msg.linear.x = -0.2
            msg.angular.z = 0.0
            self.get_logger().info("Too Close! Reversing")

        # 2. WEIGHTED CORNERING (Front approaching wall)
        # Instead of stopping, move slowly and turn progressively
        elif self.front_dist < 0.9:
            msg.linear.x = 0.15 # Keep moving slowly
            
            # Weighted Turn: The closer the wall, the sharper the turn.
            # Weight = 1 / distance. e.g. at 0.5m -> turn 2.0 rad/s
            weight = 1.0 / max(self.front_dist, 0.1)
            msg.angular.z = min(weight, 2.0) 
            
            self.get_logger().info(f"Cornering | Speed: {msg.linear.x} | Turn: {msg.angular.z:.2f}")

        # 3. LOST WALL / OPEN SPACE
        # Go STRAIGHT to find new areas. No drifting/spiraling.
        elif self.right_dist > 2.0:
            msg.linear.x = 0.35
            msg.angular.z = 0.0 # Drive dead straight
            self.get_logger().info("Open Space - Charging Forward")

        # 4. WALL FOLLOWING (PID Control)
        else:
            # Error = (Target - Actual)
            # If Actual < Target (Too Close): Error is Positive -> Turn Left (+)
            # If Actual > Target (Too Far): Error is Negative -> Turn Right (-)
            error = self.target_dist - self.right_dist
            
            # Dynamic Turning based on error magnitude
            msg.angular.z = error * self.kp
            
            # Clamp turning speed
            msg.angular.z = max(min(msg.angular.z, 2.5), -2.5)
            
            # Dynamic Linear Speed: Slow down if turning sharply
            msg.linear.x = max(0.15, self.speed - abs(msg.angular.z) * 0.1)
                
            self.get_logger().info(f"Following | Dist: {self.right_dist:.2f} | Err: {error:.2f}")

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()