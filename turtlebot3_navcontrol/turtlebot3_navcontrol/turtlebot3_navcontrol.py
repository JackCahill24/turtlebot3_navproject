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

        # Current pose of the robot (x, y, and orientation theta)
        self.current_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        # Target pose (x, y)
        self.target_pose = {'x': 0.0, 'y': 0.0}

        self.get_logger().info('Navigation Controller Node Initialized')
        self.run_navigation()

    def odom_callback(self, msg):
        """Callback function to update the robot's current position and orientation."""
        self.current_pose['x'] = msg.pose.pose.position.x
        self.current_pose['y'] = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        # Calculate yaw (theta) from quaternion
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y**2 + orientation_q.z**2)
        self.current_pose['theta'] = math.atan2(siny_cosp, cosy_cosp)

    def run_navigation(self):
        """Main function to handle the navigation process."""
        while True:
            try:
                self.get_logger().info('Starting new navigation cycle...')
                self.get_target_coordinates()
                self.navigate_to_target()
            except KeyboardInterrupt:
                self.get_logger().info('Navigation interrupted by user. Exiting.')
                break

    def get_target_coordinates(self):
        """Prompt the user to input target coordinates."""
        self.target_pose['x'] = float(input("Enter the target x-coordinate (meters): "))
        self.target_pose['y'] = float(input("Enter the target y-coordinate (meters): "))
        self.get_logger().info(f"Target coordinates set to x: {self.target_pose['x']}, y: {self.target_pose['y']}")

    def navigate_to_target(self):
        """Navigate the robot to the target coordinates."""
        rate = self.create_rate(10)  # 10 Hz
        tolerance = 0.05  # Distance tolerance in meters
        while rclpy.ok():
            # Calculate the distance and angle to the target
            dx = self.target_pose['x'] - self.current_pose['x']
            dy = self.target_pose['y'] - self.current_pose['y']
            distance = math.sqrt(dx**2 + dy**2)
            target_angle = math.atan2(dy, dx)
            angle_diff = target_angle - self.current_pose['theta']

            # Normalize the angle difference to the range [-pi, pi]
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

            # Stop if the robot is within tolerance of the target
            if distance < tolerance:
                self.stop_robot()
                self.get_logger().info('Reached target location.')
                break

            # Create a Twist message for controlling the robot
            twist_msg = Twist()

            # Rotate the robot if not aligned with the target
            if abs(angle_diff) > 0.1:  # If angle difference is significant
                twist_msg.angular.z = 0.5 * angle_diff  # Proportional control for rotation
                twist_msg.linear.x = 0.0
            else:
                # Move forward if aligned with the target
                twist_msg.linear.x = 0.2 * distance  # Proportional control for speed
                twist_msg.angular.z = 0.0

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
