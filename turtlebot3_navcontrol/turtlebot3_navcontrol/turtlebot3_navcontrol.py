import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math

class TurtleBot3NavControl(Node):
    def __init__(self):
        super().__init__('turtlebot3_navcontrol')

        # QoS for LiDAR subscription
        lidar_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)

        # Publishers and Subscribers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.lidar_subscriber = self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos_profile=lidar_qos)

        # Navigation and obstacle avoidance parameters
        self.target_x = 0.0
        self.target_y = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.goal_reached = True

        # Obstacle-related parameters
        self.obstacle_detected = False
        self.obstacle_direction = None
        self.obstacle_threshold = 0.1

        # Control parameters
        self.linear_speed = 0.15
        self.angular_speed = 0.5
        self.angle_tolerance = 0.05
        self.distance_tolerance = 0.1

        self.get_logger().info("TurtleBot3 NavControl Node Initialized")

    def odom_callback(self, msg):
        """
        Updates robot's current position and orientation from odometry data.
        """
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_theta = self.euler_from_quaternion(
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        )

        # If the goal is not reached, continue navigating
        if not self.goal_reached:
            self.navigate_to_goal()

    def lidar_callback(self, msg):
        """
        Checks for obstacles within a defined threshold using LiDAR data.
        """
        self.obstacle_detected = False
        self.obstacle_direction = None

        for i, distance in enumerate(msg.ranges):
            if 0 < distance < self.obstacle_threshold:
                angle = msg.angle_min + i * msg.angle_increment
                self.obstacle_detected = True
                self.obstacle_direction = angle
                break

    def navigate_to_goal(self):
        """
        Handles navigation to the target while avoiding obstacles.
        """
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx)

        # Stop the robot if the goal is reached
        if distance < self.distance_tolerance:
            self.goal_reached = True
            self.cmd_vel_publisher.publish(Twist())
            self.get_logger().info("Goal reached!")
            return

        twist = Twist()

        if self.obstacle_detected:
            # Avoid obstacle
            self.get_logger().info(f"Obstacle detected at angle {self.obstacle_direction:.2f} radians")
            if self.obstacle_direction > 0:
                twist.angular.z = self.angular_speed
            else:
                twist.angular.z = -self.angular_speed
            twist.linear.x = 0.0
        else:
            # Normal navigation
            angle_error = angle_to_goal - self.current_theta
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

            if abs(angle_error) > self.angle_tolerance:
                twist.angular.z = self.angular_speed * angle_error
                twist.linear.x = self.linear_speed * 0.5
            else:
                twist.linear.x = self.linear_speed
                twist.angular.z = 0.0

        # Publish the velocity command
        self.cmd_vel_publisher.publish(twist)

    @staticmethod
    def euler_from_quaternion(x, y, z, w):
        """
        Converts quaternion to euler angles.
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBot3NavControl()

    while rclpy.ok():
        try:
            if node.goal_reached:
                # Set a new goal relative to the current position
                relative_x = float(input("Enter target x coordinate relative to current position: "))
                relative_y = float(input("Enter target y coordinate relative to current position: "))
                node.target_x = node.current_x + relative_x
                node.target_y = node.current_y + relative_y
                node.goal_reached = False
                node.get_logger().info(f"New goal set: ({node.target_x}, {node.target_y})")

            rclpy.spin_once(node)
        except KeyboardInterrupt:
            break

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
