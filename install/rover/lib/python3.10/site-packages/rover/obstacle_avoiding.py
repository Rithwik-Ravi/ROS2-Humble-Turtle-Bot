import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
import math

class ObstacleAvoidingBot(Node):
    def __init__(self):
        super().__init__('Go_to_position_node') ## name of the node
        
        # --- Publishers & Subscribers ---
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 40)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.get_scan_values, 40)
        
        # NEW: Path Visualization setup
        self.path_publisher = self.create_publisher(Path, '/robot_path', 10)
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Initialize Path message
        self.path = Path()
        self.path.header.frame_id = 'odom' # This must match the fixed frame in RViz

        # --- Periodic Timer ---
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.send_cmd_vel)
        
        ## Initializing Global values
        ## given a value for VELOCITY
        self.linear_vel = 0.22  # INCREASED SPEED 
        ## Making dictionary to divide the area of lase scan 
        self.regions = {
            'r1': 100.0, 'r2': 100.0, 'r3': 100.0,
            'r4': 100.0, 'r5': 100.0, 'r6': 100.0,
            'r7': 100.0, 'r8': 100.0, 'r9': 100.0,
            'r10': 100.0, 'r11': 100.0, 'r12': 100.0
        }
        ## creating a message object to fit new velocities and publish them
        self.velocity=Twist()
        
        # Timer to force a turn duration (prevents jittering)
        self.evade_timer = 0
        self.evade_dir = 0.0

    # NEW: Callback to track robot position for RViz path
    def odom_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        
        # Append current pose to the path
        self.path.poses.append(pose)
        self.path.header.stamp = self.get_clock().now().to_msg()
        
        # Publish the updated path
        self.path_publisher.publish(self.path)

    ## Subscriber Callback function 
    def get_scan_values(self,scan_data):
        # We have a 120-degree frontal view, divided into 12 regions of 10 degrees each.
        # LaserScan data is 360 points. We use indices 120 to 240.
        num_regions = 12
        for i in range(num_regions):
            start_angle = 120 + i * 10
            end_angle = start_angle + 10
            
            # Extract slice
            slice_data = scan_data.ranges[start_angle:end_angle]
            # Filter bad values
            valid_data = [x for x in slice_data if not math.isinf(x) and not math.isnan(x) and x > 0.05]
            
            # We cap the minimum distance at 100 to avoid potential issues with large values.
            self.regions[f'r{i+1}'] = min(valid_data) if valid_data else 100.0

  
    ## Callback Publisher of velocities called every 0.2 seconds
    def send_cmd_vel(self):
        
        # --- 0. Handle Persistence Timer ---
        # If we are currently executing a forced evasion, continue doing it
        if self.evade_timer > 0:
            self.evade_timer -= 1
            self.velocity.linear.x = 0.0
            self.velocity.angular.z = self.evade_dir
            self.get_logger().info(f"Executing Evasive Turn... ({self.evade_timer})")
            self.publisher.publish(self.velocity)
            return

        # --- 1. Calculate Minimum Distances for Safety Checks ---
        # Front is roughly regions 5, 6, 7, 8
        min_front = min(self.regions['r5'], self.regions['r6'], self.regions['r7'], self.regions['r8'])
        # Sides
        min_right = min(self.regions['r1'], self.regions['r2'], self.regions['r3'])
        min_left  = min(self.regions['r10'], self.regions['r11'], self.regions['r12'])

        # --- 2. TRAP / CRASH RECOVERY LOGIC ---
        
        # CASE A: Too Close / Crashed -> Reverse
        if min_front < 0.35:
            self.velocity.linear.x = -0.25 # Reverse Faster (was -0.15)
            self.velocity.angular.z = 0.0
            self.get_logger().info("CRITICAL: Too Close! Reversing...")
            self.publisher.publish(self.velocity)
            return

        # CASE B: Corner Trap (Blocked Front + Sides) -> Spin in place
        if min_front < 0.7 and (min_right < 0.7 or min_left < 0.7):
            # Set timer to force this spin for 1.5 seconds (15 ticks)
            self.evade_timer = 15
            
            # Spin towards the slightly more open side, or default Left
            if min_right > min_left:
                self.evade_dir = -1.5 # Spin Right Faster
            else:
                self.evade_dir = 1.5  # Spin Left Faster
                
            self.get_logger().info("TRAP: Corner detected! Starting Spin.")
            self.velocity.linear.x = 0.0
            self.velocity.angular.z = self.evade_dir
            self.publisher.publish(self.velocity)
            return

        # --- 3. STANDARD OBSTACLE AVOIDANCE (Proportional) ---
        
        weights = [1.3, 1.1, 0.9, 0.7, 0.5, 0.3, -0.3, -0.5, -0.7, -0.9, -1.1, -1.3]
        
        obstacle_threshold = 1.5
        angular_velocity = 0.0
        linear_velocity = self.linear_vel
        
        obstacle_detected = False
        for i in range(len(weights)):
            region_dist = self.regions[f'r{i+1}']
            if region_dist < obstacle_threshold:
                obstacle_detected = True
                turn_strength = (obstacle_threshold - region_dist) / obstacle_threshold
                angular_velocity += weights[i] * turn_strength

        # If front regions detect a close obstacle, slow down 
        if min_front < 0.8:
            linear_velocity = 0.2 # Increased from 0.1

        # FIX: Head-on Symmetry Breaker with Persistence
        # If facing wall dead-on, force a turn and HOLD it.
        if min_front < 1.0 and abs(angular_velocity) < 0.1:
            self.evade_timer = 15 # Force turn for 1.5s
            self.evade_dir = 1.5  # Force Left Faster
            self.get_logger().info("Head-on Wall -> Starting Forced Left Turn")
            self.velocity.linear.x = 0.0
            self.velocity.angular.z = self.evade_dir
            self.publisher.publish(self.velocity)
            return

        # Normal operation
        self.velocity.linear.x = linear_velocity if obstacle_detected else self.linear_vel
        self.velocity.angular.z = angular_velocity

        # --- 4. Log the current action ---
        if self.velocity.linear.x < 0:
             self.get_logger().info("Reversing")
        elif self.velocity.angular.z > 0.8:
            self.get_logger().info("Turning LEFT sharply")
        elif self.velocity.angular.z < -0.8:
            self.get_logger().info("Turning RIGHT sharply")
        elif self.velocity.angular.z > 0.2:
            self.get_logger().info("Turning LEFT slightly")
        elif self.velocity.angular.z < -0.2:
            self.get_logger().info("Turning RIGHT slightly")
        else:
            self.get_logger().info("Moving FORWARD")
       
        self.publisher.publish(self.velocity)

def main(args=None):
    rclpy.init(args=args)
    oab=ObstacleAvoidingBot()
    rclpy.spin(oab)
    rclpy.shutdown()

if __name__ == '__main__':
    main()