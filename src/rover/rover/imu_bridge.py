import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial

class ImuBridge(Node):
    def __init__(self):
        super().__init__('imu_bridge')
        self.publisher_ = self.create_publisher(Imu, '/imu/data_raw', 10)
        
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            self.get_logger().info("Connected to Arduino on /dev/ttyUSB0")
        except Exception as e:
            self.get_logger().error(f"Failed to connect: {e}")

    def read_and_publish(self):
        if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                data = [float(x) for x in line.split(',')]
                
                if len(data) == 6:
                    msg = Imu()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'imu_link'
                    
                    # Acceleration
                    msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = data[:3]
                    # Gyro
                    msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = data[3:]
                    
                    self.publisher_.publish(msg)
            except Exception:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = ImuBridge()
    while rclpy.ok():
        node.read_and_publish()
    node.destroy_node()
    rclpy.shutdown()
