import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import numpy as np

class TurtleBot3NavControl(Node):
    def __init__(self):
        super().__init__('turtlebot3_navcontrol')

        # Parameters
        self.goal_tolerance = 0.05  # Goal proximity tolerance (meters)
        self.safe_distance = 0.3  # Minimum safe distance from obstacles (meters)
        self.k_v = 0.5  # Linear velocity gain
        self.k_omega = 1.0  # Angular velocity gain
        self.max_linear_vel = 0.22  # TurtleBot3 max linear velocity
        self.max_angular_vel = 2.84  # TurtleBot3 max angular velocity

        # Robot state
        self.current_pos = [0.0, 0.0, 0.0]  # [x, y, theta]
        self.goal_pos = [2.0, 2.0]  # Example default goal
        self.lidar_data = []

        # Publishers and Subscribers
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.update_odometry, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.update_lidar, 10)

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

    def update_odometry(self, msg):
        # Extract position and orientation
        self.current_pos[0] = msg.pose.pose.position.x
        self.current_pos[1] = msg.pose.pose.position.y

        # Convert quaternion to yaw
        orientation = msg.pose.pose.orientation
        _, _, yaw = self.quaternion_to_euler(orientation.x, orientation.y, orientation.z, orientation.w)
        self.current_pos[2] = yaw

    def update_lidar(self, msg):
        # Store LiDAR ranges
        self.lidar_data = np.array(msg.ranges)

    def quaternion_to_euler(self, x, y, z, w):
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return 0.0, 0.0, yaw

    def control_loop(self):
        if len(self.lidar_data) == 0:
            return  # Wait until LiDAR data is available

        # Compute goal-related errors
        x, y, theta = self.current_pos
        x_goal, y_goal = self.goal_pos
        error_theta = math.atan2(y_goal - y, x_goal - x) - theta
        error_theta = (error_theta + math.pi) % (2 * math.pi) - math.pi  # Normalize
        error_distance = math.sqrt((x_goal - x)**2 + (y_goal - y)**2)

        # Check if goal is reached
        if error_distance < self.goal_tolerance:
            self.stop_robot()
            self.get_logger().info('Goal reached!')
            return

        # Compute velocities
        linear_vel = min(self.k_v * error_distance, self.max_linear_vel)
        angular_vel = np.clip(self.k_omega * error_theta, -self.max_angular_vel, self.max_angular_vel)

        # Obstacle avoidance
        min_distance = np.min(self.lidar_data)
        if min_distance < self.safe_distance:
            # Adjust angular velocity to avoid obstacles
            obstacle_angle = np.argmin(self.lidar_data)  # Angle of the closest obstacle
            angular_vel += self.k_omega * (self.safe_distance - min_distance)

        # Publish velocity command
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.vel_pub.publish(twist)

    def stop_robot(self):
        # Publish zero velocities to stop the robot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBot3NavControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
