####
# - Main purpse is Obstical Avoiding by using LIDAR sensor  
# - Any object comes inside of scan Robot takes sharp turn
# - This code is going to publish on topic "cmd_vel" and
#    subscribe "/scan" topic
#
#  Written by Muhammad Luqman
#  ros2,Humble
#  13/6/21
#
###
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class ObstacleAvoidingBot(Node):
    def __init__(self):
        super().__init__('Go_to_position_node') ## name of the node
        # publisher
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 40)
        #subscriber
        self.subscription=self.create_subscription(LaserScan,'/scan',self.get_scan_values,40)
        #periodic publisher call
        timer_period = 0.1;self.timer = self.create_timer(timer_period, self.send_cmd_vel)
        ## Initializing Global values
        ## given a value for VELOCITY
        self.linear_vel = 0.22 
        ## Making dictionary to divide the area of lase scan 
        self.regions = {
            'r1': 100.0, 'r2': 100.0, 'r3': 100.0,
            'r4': 100.0, 'r5': 100.0, 'r6': 100.0,
            'r7': 100.0, 'r8': 100.0, 'r9': 100.0,
            'r10': 100.0, 'r11': 100.0, 'r12': 100.0
        }
        ## creating a message object to fit new velocities and publish them
        self.velocity=Twist()


    ## Subscriber Callback function 
    def get_scan_values(self,scan_data):
        # We have a 120-degree frontal view, divided into 12 regions of 10 degrees each.
        # LaserScan data is 360 points. We use indices 120 to 240.
        # region 'r1' is the furthest right, 'r12' is the furthest left.
        num_regions = 12
        for i in range(num_regions):
            start_angle = 120 + i * 10
            end_angle = start_angle + 10
            # We cap the minimum distance at 100 to avoid potential issues with large values.
            self.regions[f'r{i+1}'] = min(min(scan_data.ranges[start_angle:end_angle]), 100.0)

  
    ## Callback Publisher of velocities called every 0.2 seconds
    def send_cmd_vel(self):
        # Proportional control for steering
        # Weights for regions: regions closer to the center have a higher impact on turning.
        # The regions are r1 (rightmost) to r12 (leftmost).
        # A positive weight will result in a positive angular.z (left turn).
        # A negative weight will result in a negative angular.z (right turn).
        weights = [1.3, 1.1, 0.9, 0.7, 0.5, 0.3, -0.3, -0.5, -0.7, -0.9, -1.1, -1.3]
        
        obstacle_threshold = 1.5
        angular_velocity = 0.0
        linear_velocity = self.linear_vel
        
        obstacle_detected = False
        for i in range(len(weights)):
            region_dist = self.regions[f'r{i+1}']
            if region_dist < obstacle_threshold:
                obstacle_detected = True
                # The closer the obstacle, the stronger the turn
                turn_strength = (obstacle_threshold - region_dist) / obstacle_threshold
                angular_velocity += weights[i] * turn_strength

        # If front regions detect a close obstacle, slow down and turn more sharply
        if self.regions['r6'] < 0.8 or self.regions['r7'] < 0.8:
            linear_velocity = 0.1 # Slow down for safety
            print("Obstacle ahead, slowing down.")

        # If no obstacles are detected, move straight. Otherwise, use calculated velocities.
        self.velocity.linear.x = linear_velocity if obstacle_detected else self.linear_vel
        self.velocity.angular.z = angular_velocity

        # --- 4. Log the current action ---
        if self.velocity.angular.z > 0.8:
            self.get_logger().info("Turning LEFT sharply")
        elif self.velocity.angular.z < -0.8:
            self.get_logger().info("Turning RIGHT sharply")
        elif self.velocity.angular.z > 0.2:
            self.get_logger().info("Turning LEFT slightly")
        elif self.velocity.angular.z < -0.2:
            self.get_logger().info("Turning RIGHT slightly")
        else:
            self.get_logger().info("Moving FORWARD")
       
        ## lets publish the complete velocity
        self.publisher.publish(self.velocity)

def main(args=None):
    rclpy.init(args=args)
    oab=ObstacleAvoidingBot()
    rclpy.spin(oab)
    rclpy.shutdown()

if __name__ == '__main__':
    main()