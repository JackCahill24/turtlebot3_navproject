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
        self.goal_x = 3.0  # Target X position
        self.goal_y = 2.0  # Target Y position
        self.goal_phi = 0.0  # Desired final orientation

        # Define Control Gains (Modify as needed)
        self.kp = 0.15  # Proportional gain for linear velocity

        # Robot State (Updated from Odometry, initialized at origin)
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

        # Compute reference point
        l = 0.1 # Dependent on robot dimensions (0.5 to 1.5 times robot radius)
        xr = l * math.cos(current_phi)
        yr = l * math.sin(current_phi)

        # Actual coordinates of reference point and its desired position in the global frame
        xp = current_x - xr
        yp = current_y - yr
        xpd = self.goal_x + l * math.cos(self.goal_phi)
        ypd = self.goal_y + l * math.sin(self.goal_phi)

        # Compute desired velocity using feedback control
        xdot_p = self.kp * (xpd - xp)
        ydot_p = self.kp * (ypd - yp)

        # Matrix algebra to compute control inputs
        A = xr * math.cos(current_phi) - yr * math.sin(current_phi)
        B = xr * math.sin(current_phi) + yr * math.cos(current_phi)
        C = -math.sin(current_phi)
        D = math.cos(current_phi)

        # Compute control inputs (v and w)
        v = (1 / max(abs(xr), 0.001)) * ((A * xdot_p) + (B * ydot_p))
        w = (1 / max(abs(xr), 0.001)) * ((C * xdot_p) + (D * ydot_p))

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
