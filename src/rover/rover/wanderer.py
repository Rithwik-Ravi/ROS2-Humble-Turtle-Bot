import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
import math
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

class Wanderer(Node):
    def __init__(self):
        super().__init__('wanderer')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribe to Depth Image
        self.subscription = self.create_subscription(
            Image, 
            '/camera/realsense_d435/depth/image_raw', 
            self.depth_callback, 
            qos_profile_sensor_data
        )
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.velocity = Twist()
        self.bridge = CvBridge()
        
        # Sensor Readings
        self.front_dist = 10.0
        self.left_front_dist = 10.0
        self.right_front_dist = 10.0
        self.data_received = False
        
        # Parameters
        self.front_threshold = 0.55 # Reduced from 1.2 to let it get closer
        self.kp = 2.0  # Increased Gain for sharper centering during movement
        self.state = "FORWARD"
        self.turn_direction = 0.0

    def depth_callback(self, msg):
        self.data_received = True
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        depth_array = np.array(cv_image, dtype=np.float32)
        # Treat 0.0/NaN as Far
        depth_array = np.nan_to_num(depth_array, posinf=10.0, neginf=0.0, nan=10.0)
        
        height, width = depth_array.shape
        
        # Define Sectors
        left_end = int(width * 0.35)
        right_start = int(width * 0.65)
        
        left_slice = depth_array[:, :left_end]
        center_slice = depth_array[:, left_end:right_start]
        right_slice = depth_array[:, right_start:]
        
        def get_robust_min(slice_arr):
            flat = slice_arr.flatten()
            valid = flat[flat > 0.05]
            if valid.size == 0:
                return 10.0
            return np.min(valid)

        self.left_front_dist = get_robust_min(left_slice)
        self.front_dist = get_robust_min(center_slice)
        self.right_front_dist = get_robust_min(right_slice)

    def control_loop(self):
        msg = Twist()
        
        if not self.data_received:
            return

        if self.state == "FORWARD":
            # 1. Check for Blockage
            if self.front_dist < self.front_threshold:
                self.state = "ROTATE"
                msg.linear.x = 0.0
                
                # Pick open direction
                if self.left_front_dist > self.right_front_dist:
                    self.turn_direction = 0.8 # Turn Left
                else:
                    self.turn_direction = -0.8 # Turn Right
                
                self.get_logger().info(f"Blocked. Turning.")
            
            else:
                # 2. Corridor Centering (P-Controller)
                # Error = Left - Right
                # If driving BACKWARDS, image Left = robot Right.
                # If Image Left (Robot Right) > Image Right (Robot Left):
                # We are closer to Robot Left wall.
                # We want to steer Back to the Right.
                # Front Left Turn (+Z) steers Back Right.
                
                error = self.left_front_dist - self.right_front_dist
                correction = error * self.kp
                
                # Clamp correction
                msg.angular.z = max(min(correction, 1.0), -1.0)
                
                # Move Backwards
                msg.linear.x = -0.3
                
                # self.get_logger().info(f"Centering: Err={error:.2f} Cmd={msg.angular.z:.2f}")

        elif self.state == "ROTATE":
            # Hysteresis to exit rotation
            clear_thresh = self.front_threshold + 0.5
            
            # Check if path is clear
            if self.front_dist > clear_thresh:
                self.state = "FORWARD"
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.get_logger().info("Clear -> Forward")
            else:
                # Keep turning
                msg.linear.x = 0.0
                msg.angular.z = self.turn_direction
                
        # Safety Override
        if self.front_dist < 0.25:
             msg.linear.x = 0.2 # Drive Forward (away)
             msg.angular.z = 0.0
             self.get_logger().info("Emergency Forward")
        
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Wanderer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()