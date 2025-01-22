#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion

class TurtleBot3NavControl(Node):
    def __init__(self, goal_x, goal_y):
        super().__init__('turtlebot3_navcontrol')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # User-defined goal location
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.yaw = 0.0
        self.obstacle_detected = False

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f"TurtleBot3 Navigation Controller Node Started")
        self.get_logger().info(f"Navigating to Goal: x={goal_x:.2f}, y={goal_y:.2f}")

    def odom_callback(self, msg):
        # Extract position and orientation from odometry data
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w,
        ])

    def scan_callback(self, msg):
        # Detect obstacles within a certain range
        min_distance = min(msg.ranges)
        self.obstacle_detected = min_distance < 0.5  # 0.5 meters

    def control_loop(self):
        # Calculate distance to the goal
        distance_to_goal = math.sqrt((self.goal_x - self.pose_x) ** 2 +
                                     (self.goal_y - self.pose_y) ** 2)

        if distance_to_goal < 0.1:
            self.stop_robot()
            self.get_logger().info("Goal reached!")
            return

        if self.obstacle_detected:
            self.avoid_obstacle()
        else:
            self.navigate_to_goal()

    def navigate_to_goal(self):
        # Proportional controller for navigation
        angle_to_goal = math.atan2(self.goal_y - self.pose_y,
                                   self.goal_x - self.pose_x)
        angle_error = angle_to_goal - self.yaw

        # Normalize the angle to [-pi, pi]
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        msg = Twist()
        if abs(angle_error) > 0.1:
            # Rotate toward the goal
            msg.angular.z = 0.5 * angle_error
        else:
            # Move forward
            msg.linear.x = 0.2
        self.publisher.publish(msg)

    def avoid_obstacle(self):
        # Simple obstacle avoidance logic
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.5  # Rotate to avoid obstacles
        self.publisher.publish(msg)

    def stop_robot(self):
        # Publish zero velocity to stop the robot
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    # Ask the user for the goal coordinates
    print("Enter the target goal location in meters:")
    goal_x = float(input("  Goal X (meters): "))
    goal_y = float(input("  Goal Y (meters): "))

    # Start the navigation node
    node = TurtleBot3NavControl(goal_x, goal_y)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

