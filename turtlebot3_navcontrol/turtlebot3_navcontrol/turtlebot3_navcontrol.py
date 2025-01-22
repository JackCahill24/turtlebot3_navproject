#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class TurtleBot3NavControl(Node):
    def __init__(self, goal_x, goal_y):
        super().__init__('turtlebot3_navcontrol')

        # Goal coordinates
        self.goal_x = goal_x
        self.goal_y = goal_y

        # State variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.yaw = 0.0
        self.obstacle_detected = False

        # Safety and tolerance parameters
        self.safe_distance = 0.127  # 5 inches in meters
        self.goal_tolerance = 0.1  # Stop if within 0.1 meters of the goal

        # ROS 2 publishers and subscribers
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile=QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        # Control loop timer
        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(f"Navigator started. Goal: x={goal_x:.2f}, y={goal_y:.2f}")

    def odom_callback(self, msg):
        """Updates the robot's position and orientation."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1.0 - 2.0 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg):
        """Detects obstacles using LiDAR data."""
        self.obstacle_detected = any(distance < self.safe_distance for distance in msg.ranges)

    def control_loop(self):
        """Main control loop for navigation."""
        distance_to_goal = math.sqrt((self.goal_x - self.current_x) ** 2 + (self.goal_y - self.current_y) ** 2)

        if distance_to_goal < self.goal_tolerance:
            self.stop_robot()
            self.get_logger().info("Goal reached!")
            return

        if self.obstacle_detected:
            self.avoid_obstacle()
        else:
            self.move_toward_goal()

    def move_toward_goal(self):
        """Drives the robot in a straight line toward the goal."""
        angle_to_goal = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)
        angle_error = angle_to_goal - self.yaw
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))  # Normalize to [-pi, pi]

        msg = Twist()
        msg.linear.x = 0.2  # Move forward
        msg.angular.z = 0.5 * angle_error  # Adjust heading toward goal
        self.velocity_publisher.publish(msg)

    def avoid_obstacle(self):
        """Stops the robot and adjusts its heading to avoid obstacles."""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.5  # Turn in place to avoid obstacle
        self.velocity_publisher.publish(msg)
        self.get_logger().info("Obstacle detected! Turning to avoid.")

    def stop_robot(self):
        """Stops the robot."""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.velocity_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    # Get the goal coordinates from the user
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
