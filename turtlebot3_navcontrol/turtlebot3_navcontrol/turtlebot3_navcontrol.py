#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class TurtleBot3NavController(Node):
    def __init__(self):
        super().__init__('turtlebot3_navcontrol')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.current_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.target_pose = {'x': 0.0, 'y': 0.0}
        
        self.get_logger().info('Navigation Controller Node Initialized')
        self.run_navigation()

    def odom_callback(self, msg):
        """Callback function to update the robot's current position and orientation."""
        self.current_pose['x'] = msg.pose.pose.position.x
        self.current_pose['y'] = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        # Manually calculate yaw from quaternion
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        self.current_pose['theta'] = math.atan2(siny_cosp, cosy_cosp)

    def run_navigation(self):
        """Main function to handle navigation."""
        while True:
            self.get_logger().info('Starting new navigation cycle...')
            try:
                self.get_target_coordinates()
                self.navigate_to_target()
            except KeyboardInterrupt:
                self.get_logger().info('Navigation interrupted. Exiting.')
                break

    def get_target_coordinates(self):
        """Prompt the user to input target coordinates."""
        self.target_pose['x'] = float(input("Enter the target x-coordinate (meters): "))
        self.target_pose['y'] = float(input("Enter the target y-coordinate (meters): "))
        self.get_logger().info(f"Target coordinates set to x: {self.target_pose['x']}, y: {self.target_pose['y']}")

    def navigate_to_target(self):
        """Navigate the robot to the target coordinates."""
        rate = self.create_rate(10)  # 10 Hz
        while rclpy.ok():
            # Calculate the distance and angle to the target
            dx = self.target_pose['x'] - self.current_pose['x']
            dy = self.target_pose['y'] - self.current_pose['y']
            distance = math.sqrt(dx**2 + dy**2)
            target_angle = math.atan2(dy, dx)
            angle_diff = target_angle - self.current_pose['theta']

            # Normalize angle difference
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

            # Stop when close enough to the target
            if distance < 0.1:
                self.stop_robot()
                self.get_logger().info('Reached target location.')
                break

            # Rotate towards the target if angle difference is significant
            if abs(angle_diff) > 0.1:
                twist_msg = Twist()
                twist_msg.angular.z = 0.5 * angle_diff
                self.publisher_.publish(twist_msg)
            else:
                # Move forward towards the target
                twist_msg = Twist()
                twist_msg.linear.x = 0.2 * distance
                twist_msg.angular.z = 0.5 * angle_diff
                self.publisher_.publish(twist_msg)

            rate.sleep()

    def stop_robot(self):
        """Stop the robot."""
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.publisher_.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = TurtleBot3NavController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user.')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
