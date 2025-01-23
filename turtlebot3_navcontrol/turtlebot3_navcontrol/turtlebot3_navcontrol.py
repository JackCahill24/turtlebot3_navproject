import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class TurtleBot3NavControl(Node):
    def __init__(self):
        super().__init__('turtlebot3_navcontrol')

        # Publishers and Subscribers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Desired goal (x, y)
        self.target_x = 0.0
        self.target_y = 0.0

        # Current position and orientation
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0  # Orientation in radians

        # Flags
        self.goal_reached = False

        self.get_logger().info("TurtleBot3 NavControl Node Initialized")
        self.get_logger().info("Set the target location by modifying 'self.target_x' and 'self.target_y'.")

    def odom_callback(self, msg):
        """
        Callback to process odometry data and update the robot's position and orientation.
        """
        # Extract the current position
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Extract the orientation (yaw) from the quaternion
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_theta = self.euler_from_quaternion(
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)

        # Check if the goal is reached
        if not self.goal_reached:
            self.navigate_to_goal()

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert quaternion to euler angles.
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

    def navigate_to_goal(self):
        """
        Navigation logic to move the robot to the target location.
        """
        # Compute the error
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        distance_to_goal = math.sqrt(dx**2 + dy**2)
        desired_theta = math.atan2(dy, dx)

        # If the robot is close to the goal, stop moving
        if distance_to_goal < 0.1:
            self.goal_reached = True
            twist = Twist()
            self.cmd_vel_publisher.publish(twist)  # Publish zero velocity
            self.get_logger().info("Goal reached!")
            return

        # Compute control commands
        twist = Twist()

        # Rotate towards the target
        angle_error = desired_theta - self.current_theta
        # Normalize angle_error to [-pi, pi]
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        if abs(angle_error) > 0.1:  # If orientation error is significant, rotate
            twist.linear.x = 0.0
            twist.angular.z = 0.5 * angle_error
        else:  # Move forward when aligned
            twist.linear.x = 0.2
            twist.angular.z = 0.0

        # Publish the velocity command
        self.cmd_vel_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBot3NavControl()

    # Set target location
    node.target_x = float(input("Enter target x coordinate: "))
    node.target_y = float(input("Enter target y coordinate: "))

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
