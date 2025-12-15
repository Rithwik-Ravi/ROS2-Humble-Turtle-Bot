import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import numpy as np
import math

class SensorDiag(Node):
    def __init__(self):
        super().__init__('sensor_diag')
        
        # Subscribe to Lidar
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        
        # Subscribe to Depth Camera (trying both common topics)
        self.create_subscription(Image, '/camera/depth/image_raw', self.img_callback, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/realsense_d435/depth/image_raw', self.img_callback, qos_profile_sensor_data)
        
        self.bridge = CvBridge()
        self.get_logger().info("--- SENSOR DIAGNOSTIC RUNNING ---")
        self.get_logger().info("Place robot facing a wall to identify FRONT index.")

    def scan_callback(self, msg):
        size = len(msg.ranges)
        
        # Indices to check
        idx_0 = 0
        idx_quarter = int(size * 0.25)
        idx_mid = int(size * 0.5)
        idx_3quarter = int(size * 0.75)
        
        # Get values (0.0 if infinite)
        def get_val(idx):
            v = msg.ranges[idx]
            return 0.0 if math.isinf(v) else v

        self.get_logger().info(
            f"LIDAR (Size {size}): "
            f"Idx 0: {get_val(idx_0):.2f}m | "
            f"Idx {idx_quarter} (25%): {get_val(idx_quarter):.2f}m | "
            f"Idx {idx_mid} (50%): {get_val(idx_mid):.2f}m | "
            f"Idx {idx_3quarter} (75%): {get_val(idx_3quarter):.2f}m"
        )

    def img_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            height, width = cv_img.shape
            
            # Sample center pixel
            center_dist = cv_img[int(height/2), int(width/2)]
            
            self.get_logger().info(f"CAMERA ({width}x{height}): Center Pixel Distance: {center_dist:.2f}m")
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = SensorDiag()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()