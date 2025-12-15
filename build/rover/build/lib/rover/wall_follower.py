import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.subscription_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.velocity = Twist()
        
        # 5-Zone Sensor Readings (default to infinity)
        self.regions = {
            'far_right': 10.0,
            'front_right': 10.0,
            'front': 10.0,
            'front_left': 10.0,
            'far_left': 10.0,
        }
        
        # Robot Parameters
        self.linear_speed = 0.2   # Slow forward speed
        # Robot Parameters
        self.linear_speed = 0.2   # Slow forward speed
        self.angular_speed = 0.5  # Slow turning speed (to prevent spinning)
        self.d_front = 0.9        # Front wall turn distance (earlier turn)
        self.d_side = 0.6         # Side wall following distance
        
        self.state = 0
        self.state_desc = ''
        
        # 90-Degree Turn State
        self.is_turning = False
        self.initial_yaw = 0.0
        self.current_yaw = 0.0
        self.target_yaw = 0.0 # Absolute target angle

    def odom_callback(self, msg):
        # Extract quaternion
        orientation_q = msg.pose.pose.orientation
        
        # Convert Quaternion to Euler (Yaw)
        # standardized math for yaw from quaternion
        t3 = +2.0 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        t4 = +1.0 - 2.0 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        self.current_yaw = math.atan2(t3, t4)

    def scan_callback(self, msg):
        ranges = msg.ranges
        size = len(ranges)
        
        # Helper: Get minimum valid value in a slice
        def get_min(start_idx, end_idx):
            start_idx = max(0, start_idx)
            end_idx = min(size, end_idx)
            slice_data = ranges[start_idx:end_idx]
            valid = [r for r in slice_data if not math.isinf(r) and not math.isnan(r) and r > 0.05]
            return min(valid) if valid else 10.0

        # Divide 360 Lidar into 5 zones
        # Index 0 is Right (standard ROS 2 Turtlebot convention)
        chunk = int(size / 5)
        
        self.regions['far_right']   = get_min(0, chunk)
        self.regions['front_right'] = get_min(chunk, 2 * chunk)
        self.regions['front']       = get_min(2 * chunk, 3 * chunk)
        self.regions['front_left']  = get_min(3 * chunk, 4 * chunk)
        self.regions['far_left']    = get_min(4 * chunk, size)

    def timer_callback(self):
        msg = Twist()
        
        # --- 90-Degree Turn Execution ---
        # Unpack for cleaner logic checks
        fr  = self.regions['far_right']
        f_r = self.regions['front_right']
        f   = self.regions['front']
        f_l = self.regions['front_left']
        fl  = self.regions['far_left']
        
        df = self.d_front
        ds = self.d_side

        # --- LOGIC STATE MACHINE ---
        # State 0: Find Wall (Go Straight)
        # State 1: Turn Left (Pivot/Avoid)
        # State 2: Follow Wall (Straight)
        # State 3: Correct Right (Slight Right)
        # State 4: Reverse (Stuck)

        # 0. SAFETY CHECK: Too close?
        if f < 0.25 or f_r < 0.25 or f_l < 0.25:
            self.state = 4
            self.state_desc = "EMERGENCY: Too Close -> Reverse"

        # 1. OBSTACLES DETECTED (Logic Table)
        
        # Case 1: No obstacles -> Find Wall
        elif fr > ds and f_r > df and f > df and f_l > df and fl > ds:
            self.state = 0
            self.state_desc = "Case 1: Open Space -> Going Straight"
            
        # Case 2: Front Blocked -> Turn Left
        elif fr > ds and f_r > df and f < df and f_l > df and fl > ds:
            self.state = 1
            self.state_desc = "Case 2: Front Blocked"

        # Case 3: Front-Right Blocked -> Turn Left
        elif fr > ds and f_r < df and f > df and f_l > df and fl > ds:
            self.state = 1
            self.state_desc = "Case 3: Corner (FR) -> Turning Left"

        # Case 4: Front-Left Blocked -> Turn Right (Correction)
        elif fr > ds and f_r > df and f > df and f_l < df and fl > ds:
            self.state = 0 # Just go straight/drift right
            self.state_desc = "Case 4: Front-Left Obstacle"

        # Case 5: Front + Front-Right -> Turn Left
        elif fr > ds and f_r < df and f < df and f_l > df and fl > ds:
            self.state = 1
            self.state_desc = "Case 5: Front + FR Blocked"

        # Case 6: Front + Front-Left -> Turn Left (Safe side)
        elif fr > ds and f_r > df and f < df and f_l < df and fl > ds:
            self.state = 1
            self.state_desc = "Case 6: Front + FL Blocked"

        # Case 7: Front + FR + FL -> Turn Left (Dead End?)
        elif fr > ds and f_r < df and f < df and f_l < df and fl > ds:
            self.state = 1
            self.state_desc = "Case 7: Front Wall Detected"

        # Case 9: Wall on Far Right -> Follow
        elif fr < ds and f_r > df and f > df and f_l > df and fl > ds:
            self.state = 2
            self.state_desc = "Case 9: Following Wall (Right)"

        # Case 10: Far Right + Front -> Turn Left
        elif fr < ds and f_r > df and f < df and f_l > df and fl > ds:
            self.state = 1
            self.state_desc = "Case 10: Wall + Front Obstacle"

        # Case 11: Far Right + Front Right -> Turn Left (Corner)
        elif fr < ds and f_r < df and f > df and f_l > df and fl > ds:
            self.state = 1
            self.state_desc = "Case 11: Approaching Corner"

        # Case 13: Wall Corner (FR + F_R + F)
        elif fr < ds and f_r < df and f < df and f_l > df and fl > ds:
            self.state = 1
            self.state_desc = "Case 13: Wall Corner"

        # Case X: Catch-all for "Something is near" -> Default to Left Turn Safety
        else:
            # If ANY of the front sensors are triggered, we prioritize turning left
            if f < df or f_r < df or f_l < df:
                self.state = 1
                self.state_desc = "Catch-All: Obstacle Ahead"
            else:
                # If only side sensors are triggered or weird combo, go straight
                self.state = 2
                self.state_desc = "Catch-All: Clear Path"

        # Check if we need to start a 90-degree turn
        if self.state == 1:
            self.is_turning = True
            
            # Grid Alignment Logic:
            # 1. Snap current yaw to nearest cardinal direction (0, pi/2, pi, -pi/2)
            # Round to nearest 90 deg block
            current_grid = round(self.current_yaw / (math.pi / 2)) * (math.pi / 2)
            
            # 2. Target is 90 degrees to the Left
            self.target_yaw = current_grid + (math.pi / 2)
            
            # 3. Normalize target to [-pi, pi]
            if self.target_yaw > math.pi: self.target_yaw -= 2 * math.pi
            if self.target_yaw <= -math.pi: self.target_yaw += 2 * math.pi
            
            self.state_desc = f"Initiating Grid Turn -> Target: {self.target_yaw:.2f}"

        # --- 90-Degree Turn Execution ---
        if self.is_turning:
            # Calculate error to target
            diff = self.target_yaw - self.current_yaw
            
            # Handle wrap-around (-pi to pi)
            if diff > math.pi: diff -= 2*math.pi
            if diff < -math.pi: diff += 2*math.pi
            
            error = abs(diff)
            
            if error < 0.02: # Arrived at target
                self.is_turning = False
                self.state_desc = "Turn Complete -> Resuming Control"
                msg.angular.z = 0.0
                self.publisher.publish(msg)
            else:
                # 2-Stage Turn: Slow down when getting close
                speed = self.angular_speed
                if error < 0.2: # Last 11 degrees
                    speed = 0.1 # Very slow finish
                
                self.state_desc = f"Grid Turn: Cur={self.current_yaw:.2f} Tgt={self.target_yaw:.2f} Err={error:.2f}"
                msg.linear.x = 0.0
                # Determine direction (should be left/positive, but math handles it)
                msg.angular.z = speed if diff > 0 else -speed # Safe check, though we always turn left
                self.publisher.publish(msg)
                
            self.get_logger().info(f"State: SPECIAL | {self.state_desc}")
            return # Skip normal logic while turning

        # --- EXECUTE STATE ---
        if self.state == 0: # Find Wall (Go Straight)
            msg.linear.x = self.linear_speed
            msg.angular.z = 0.0 # No turn, just go forward
            
        elif self.state == 1: # Turn Left
            msg.linear.x = 0.0 # Stop to turn
            msg.angular.z = self.angular_speed
            
        elif self.state == 2: # Follow Wall
            msg.linear.x = self.linear_speed
            msg.angular.z = 0.0
            
        elif self.state == 3: # Slow Straight
            msg.linear.x = self.linear_speed * 0.5
            msg.angular.z = 0.0
            
        elif self.state == 4: # Reverse
            msg.linear.x = -0.15
            msg.angular.z = 0.0

        self.publisher.publish(msg)
        self.get_logger().info(f"State: {self.state} | {self.state_desc}")

def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()