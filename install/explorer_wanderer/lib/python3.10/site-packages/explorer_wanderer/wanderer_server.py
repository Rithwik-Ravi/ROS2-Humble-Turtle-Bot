# written by Enrique Fernández-Laguilhoat Sánchez-Biezma and Daniel García López
# Modified for custom rover indices and thresholds

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from explorer_interfaces.action import Wander

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

from random import random
import math

# ros2 action send_goal wander explorer_interfaces/action/Wander "{strategy: 1, map_completed_thres: 0.6}"

# REDUCED: Allows robot to get closer to walls (e.g. 0.53m) without stopping
distance_from_wall = 0.4 

class Subscriber(Node):

    def __init__(self):
        super().__init__('subscriber')
        self.subscription = self.create_subscription(LaserScan, 'scan', self.listener_callback, 10)
        self.subscription
        self.forward_distance = 1000.0
        self.left_forward_distance = 1000.0
        self.right_forward_distance = 1000.0
        self.left_distance = 1000.0
        self.back_distance = 1000.0
        self.right_distance = 1000.0

    def listener_callback(self, msg):
        size = len(msg.ranges)
        
        # --- MAPPING INDICES FOR YOUR ROBOT ---
        # Front is Middle (0.5), Right is Low (0.25), Left is High (0.75)
        
        idx_front = int(size / 2)
        idx_left = int(size * 0.75)
        idx_right = int(size * 0.25)
        idx_back = 0
        idx_left_front = int(size * 0.625)  # ~45 deg left of front
        idx_right_front = int(size * 0.375) # ~45 deg right of front

        # Helper to handle inf/nan
        def get_range(index):
            val = msg.ranges[index]
            if math.isinf(val) or math.isnan(val) or val == 0.0:
                return 10.0
            return val

        self.forward_distance = get_range(idx_front)
        self.left_distance = get_range(idx_left)
        self.back_distance = get_range(idx_back)
        self.right_distance = get_range(idx_right)
        self.left_forward_distance = get_range(idx_left_front)
        self.right_forward_distance = get_range(idx_right_front)


class Publisher(Node):
    def __init__(self):
        super().__init__('publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)


def reset_commands(command):
    command.linear.x = 0.0
    command.linear.y = 0.0
    command.linear.z = 0.0
    command.angular.x = 0.0
    command.angular.y = 0.0
    command.angular.z = 0.0
    return command


def check_ranges(subscriber):
    rclpy.spin_once(subscriber)
    readings = [subscriber.forward_distance, subscriber.left_forward_distance, subscriber.right_forward_distance]
    min_value = min(readings)
    
    # Log distances for debugging
    # subscriber.get_logger().info(f"F: {subscriber.forward_distance:.2f} LF: {subscriber.left_forward_distance:.2f} RF: {subscriber.right_forward_distance:.2f}")

    if subscriber.forward_distance < distance_from_wall:
        subscriber.get_logger().info("Obstacle detected (Front)")
        return False, min_value
    else:
        # If walls are far enough away on sides, keep going
        if (subscriber.right_forward_distance > distance_from_wall and
                subscriber.left_forward_distance > distance_from_wall):
            return True, min_value
        else:
            # Logic to keep following a wall if aligned properly
            # If aiming away from the closer wall, keep going
            if (subscriber.left_forward_distance > subscriber.left_distance or
                    subscriber.right_forward_distance > subscriber.right_distance):
                return True, min_value
            else:
                subscriber.get_logger().info("Obstacle detected (Side approach)")
                return False, min_value


def go_forward_until_obstacle(subscriber, publisher, command):
    command = reset_commands(command)

    while check_ranges(subscriber)[0]:
        rclpy.spin_once(subscriber)
        speed = check_ranges(subscriber)[1] / 2.5
        if speed > 0.4:  # Reduced max speed for safety
            speed = 0.4
        command.linear.x = float(speed)
        publisher.publisher_.publish(command)

    command = reset_commands(command)
    publisher.publisher_.publish(command)


def rotate_until_clear(subscriber, publisher, command):
    command = reset_commands(command)
    rclpy.spin_once(subscriber)

    # Determine direction based on open space
    if subscriber.left_forward_distance < subscriber.right_forward_distance:
        # Rotate Right (Open space is on Right)
        while (subscriber.left_forward_distance < subscriber.left_distance or 
               subscriber.forward_distance < distance_from_wall):
            rclpy.spin_once(subscriber)
            command.angular.z = -0.6 # Constant rotation speed
            publisher.publisher_.publish(command)
    else:
        # Rotate Left (Open space is on Left)
        while (subscriber.right_forward_distance < subscriber.right_distance or 
               subscriber.forward_distance < distance_from_wall):
            rclpy.spin_once(subscriber)
            command.angular.z = 0.6
            publisher.publisher_.publish(command)

    publisher.get_logger().info("Path Clear -> Switching to FORWARD")
    command = reset_commands(command)
    publisher.publisher_.publish(command)


class WandererServer(Node):

    def __init__(self):
        super().__init__('wanderer_server')
        self._action_server = ActionServer(self, Wander, 'wander', self.execute_callback)
        self.watchtower_subscription = self.create_subscription(Float32, 'map_progress', self.watchtower_callback, 10)
        self.watchtower_subscription
        self.stop_wandering = False
        self.map_completed_thres = 1.0
        self.get_logger().info("Wanderer Server is ready")
        self.subscriber = Subscriber()
        self.publisher = Publisher()
        self.command = Twist()

    def watchtower_callback(self, msg):
        if msg.data > self.map_completed_thres:
            self.stop_wandering = True

    def execute_callback(self, goal_handle):
        self.get_logger().info("Wanderer Server received a goal")
        self.map_completed_thres = goal_handle.request.map_completed_thres
        
        while not self.stop_wandering:
            go_forward_until_obstacle(self.subscriber, self.publisher, self.command)
            rotate_until_clear(self.subscriber, self.publisher, self.command)

        self.get_logger().info('Wandering Finished')
        goal_handle.succeed()
        return Wander.Result()


def main(args=None):
    rclpy.init(args=args)
    wanderer_server = WandererServer()
    rclpy.spin(wanderer_server)
    rclpy.shutdown()


if __name__ == '__main__':
    main()