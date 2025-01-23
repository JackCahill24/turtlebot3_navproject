import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class TurtleBot3NavControl(Node):
    def __init__(self):
        super().__init__('turtlebot3_navcontrol')
        self.lidar_subscriber = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(
            Twist, '/cmd_vel', 10)

        # Controller parameters (adjust as needed)
        self.linear_velocity = 0.2  # Base forward speed
        self.angular_gain = 1.0  # Proportional gain for angular velocity

        self.get_logger().info("TurtleBot3 NavControl Node Initialized")

    def lidar_callback(self, msg):
        """
        Callback to process LiDAR data and control the robot.
        """
        # Split ranges into left, front, and right sectors
        front_ranges = msg.ranges[len(msg.ranges) // 3: 2 * len(msg.ranges) // 3]
        left_ranges = msg.ranges[:len(msg.ranges) // 3]
        right_ranges = msg.ranges[2 * len(msg.ranges) // 3:]

        # Compute the average distance for each sector
        front_avg = min(front_ranges) if front_ranges else float('inf')
        left_avg = min(left_ranges) if left_ranges else float('inf')
        right_avg = min(right_ranges) if right_ranges else float('inf')

        self.get_logger().info(f"Front: {front_avg}, Left: {left_avg}, Right: {right_avg}")

        # Closed-loop control logic
        twist = Twist()

        if front_avg < 0.075:  # Obstacle detected in front
            # Stop moving forward and adjust direction
            twist.linear.x = 0.0
            if left_avg > right_avg:
                twist.angular.z = self.angular_gain  # Turn left
            else:
                twist.angular.z = -self.angular_gain  # Turn right
        else:
            # Move forward and adjust orientation proportionally
            twist.linear.x = self.linear_velocity
            angular_error = (right_avg - left_avg)  # Difference between left and right distances
            twist.angular.z = self.angular_gain * angular_error  # Proportional control

        # Publish the velocity command
        self.cmd_vel_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBot3NavControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
