#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class TurtleBot3BasicController(Node):
    def __init__(self, goal_x, goal_y):
        super().__init__('turtlebot3_basic_controller')
        
        # Goal coordinates
        self.goal_x = goal_x
        self.goal_y = goal_y
        
        # State variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.yaw = 0.0
        self.closest_obstacle_distance = float('inf')
        self.obstacle_detected = False
        
        # Controller gains
        self.kp_linear = 0.4  # Proportional gain for linear velocity
        self.kp_angular = 1.2  # Proportional gain for angular velocity
        
        # Thresholds and tolerances
        self.goal_tolerance = 0.1  # Tolerance for goal distance
        self.safe_distance = 0.15  # Minimum safe distance from obstacles (meters)
        
        # Publisher and subscribers
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Control loop timer
        self.control_timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f"Controller started. Goal: x={goal_x}, y={goal_y}")

    def odom_callback(self, msg):
        """Updates the robot's current position and yaw based on odometry."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1.0 - 2.0 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg):
        """Detects obstacles using LiDAR data."""
        self.closest_obstacle_distance = min(msg.ranges)
        self.obstacle_detected = self.closest_obstacle_distance < self.safe_distance

    def control_loop(self):
        """Main control loop for navigating to the goal while avoiding obstacles."""
        distance_to_goal = math.sqrt((self.goal_x - self.current_x) ** 2 + (self.goal_y - self.current_y) ** 2)
        
        if distance_to_goal < self.goal_tolerance:
            self.stop_robot()
            self.get_logger().info("Goal reached!")
            return

        if self.obstacle_detected:
            self.avoid_obstacle()
        else:
            self.navigate_to_goal(distance_to_goal)

    def navigate_to_goal(self, distance_to_goal):
        """Controller for navigating toward the goal."""
        angle_to_goal = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)
        angle_error = angle_to_goal - self.yaw
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))  # Normalize to [-pi, pi]

        # Linear velocity control
        linear_velocity = self.kp_linear * distance_to_goal
        linear_velocity = min(linear_velocity, 0.3)  # Cap max speed to 0.3 m/s
        
        # Angular velocity control
        angular_velocity = self.kp_angular * angle_error

        # Publish velocity commands
        msg = Twist()
        msg.linear.x = linear_velocity if abs(angle_error) < 0.5 else 0.0  # Slow down when turning
        msg.angular.z = angular_velocity
        self.velocity_publisher.publish(msg)

        self.get_logger().info(
            f"Navigating: Distance={distance_to_goal:.2f}, Angle Error={angle_error:.2f}, Linear={msg.linear.x:.2f}, Angular={msg.angular.z:.2f}"
        )

    def avoid_obstacle(self):
        """Controller for avoiding obstacles."""
        # Stop forward motion and rotate slightly to adjust path
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.3  # Rotate to avoid the obstacle
        self.velocity_publisher.publish(msg)

        self.get_logger().info("Avoiding obstacle. Adjusting path.")

    def stop_robot(self):
        """Stops the robot gracefully."""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.velocity_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    # Ask the user for the goal coordinates
    print("Enter the target goal location in meters:")
    goal_x = float(input("  Goal X (meters): "))
    goal_y = float(input("  Goal Y (meters): "))

    # Start the controller node
    node = TurtleBot3BasicController(goal_x, goal_y)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()