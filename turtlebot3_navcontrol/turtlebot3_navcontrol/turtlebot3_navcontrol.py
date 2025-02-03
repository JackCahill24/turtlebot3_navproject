import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class FeedbackController(Node):
    def __init__(self):
        super().__init__('turtlebot3_navcontrol')

        # ROS 2 Publisher & Subscriber
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        # Define Goal State (Modify as needed)
        self.goal_x = 2.0  # Target X position
        self.goal_y = 2.0  # Target Y position
        self.goal_phi = 0.0  # Desired final orientation

        # Define Control Gains (Modify as needed)
        self.kp = 0.1  # Proportional gain for linear velocity

        # Robot State (Updated from Odometry)
        self.x = 0.0
        self.y = 0.0
        self.phi = 0.0  # Orientation (yaw)

    def odom_callback(self, msg):
        """ Updates the robot's current position and orientation from odometry """
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y**2 + q.z**2)
        self.phi = math.atan2(siny_cosp, cosy_cosp)

        # Call the custom controller (Implement your control inside this function)
        self.compute_control()

    def compute_control(self):
        """ Feedback Control Law """

        # Extract current position and orientation
        current_x = self.x
        current_y = self.y
        current_phi = self.phi

        # Compute relative position error (xr, yr) in the robot's local frame
        xr = (self.goal_x - current_x) * math.cos(current_phi) + (self.goal_y - current_y) * math.sin(current_phi)
        yr = -(self.goal_x - current_x) * math.sin(current_phi) + (self.goal_y - current_y) * math.cos(current_phi)

        # Compute desired velocity using feedback control
        xdot_p = self.kp * (self.goal_x - current_x)
        ydot_p = self.kp * (self.goal_y - current_y)

        # Compute control inputs (v and w)
        v = (1 / max(abs(xr), 0.001)) * ((xr * xdot_p) + (yr * ydot_p))
        w = (1 / max(abs(xr), 0.001)) * ((-math.sin(current_phi) * xdot_p) + (math.cos(current_phi) * ydot_p))

        # Stop if close to goal
        if abs(self.goal_x - current_x) < 0.05 and abs(self.goal_y - current_y) < 0.05:
            v = 0.0
            w = 0.0

        # Publish control inputs
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        self.cmd_pub.publish(twist)

        # Debugging Output
        self.get_logger().info(f'Goal: ({self.goal_x}, {self.goal_y}), Position: ({self.x:.2f}, {self.y:.2f}), v={v:.2f}, w={w:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = FeedbackController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
