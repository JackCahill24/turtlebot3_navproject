import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math


class TurtleBot3NavControl(Node):
    def __init__(self):
        super().__init__('turtlebot3_navcontrol')

        # Publisher and Subscriber
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Target coordinates (relative to the current position)
        self.target_x = 0.0
        self.target_y = 0.0

        # Current position and orientation
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        # Flags
        self.goal_reached = True

        # Control parameters
        self.linear_speed = 0.15  # Consistent forward speed
        self.angular_speed = 0.5  # Consistent turning speed
        self.angle_tolerance = 0.05  # Tolerance for angle alignment (radians)
        self.distance_tolerance = 0.1  # Tolerance for reaching the goal (meters)

        self.get_logger().info("TurtleBot3 NavControl Node Initialized")

    def odom_callback(self, msg):
        # Update the robot's current position and orientation
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Convert quaternion to yaw (orientation)
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_theta = self.euler_from_quaternion(
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        )

        # Navigate to the target if a goal is set
        if not self.goal_reached:
            self.navigate_to_goal()

    def navigate_to_goal(self):
        # Calculate the distance and angle to the goal
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx)

        # Check if the goal is reached
        if distance < self.distance_tolerance:
            self.goal_reached = True
            self.cmd_vel_publisher.publish(Twist())  # Stop the robot
            self.get_logger().info("Goal reached!")
            return

        # Control logic for smoother motion
        twist = Twist()

        # Calculate angular error and normalize to [-pi, pi]
        angle_error = angle_to_goal - self.current_theta
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        if abs(angle_error) > self.angle_tolerance:  # Correct orientation
            twist.linear.x = self.linear_speed * 0.5  # Slow forward motion during turning
            twist.angular.z = self.angular_speed * angle_error
        else:  # Move toward the goal
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0

        # Publish the velocity command
        self.cmd_vel_publisher.publish(twist)

    def euler_from_quaternion(self, x, y, z, w):
        # Convert quaternion to euler angles
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
            if node.goal_reached:  # Only accept new input after reaching the previous goal
                # Get target coordinates relative to the current position
                relative_x = float(input("Enter target x coordinate relative to current position: "))
                relative_y = float(input("Enter target y coordinate relative to current position: "))

                # Calculate new target position in global coordinates
                node.target_x = node.current_x + relative_x
                node.target_y = node.current_y + relative_y

                node.goal_reached = False  # Reset the goal flag
                node.get_logger().info(f"Moving to target ({node.target_x}, {node.target_y})...")

            rclpy.spin_once(node)  # Allow the node to process callbacks
        except KeyboardInterrupt:
            break

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
