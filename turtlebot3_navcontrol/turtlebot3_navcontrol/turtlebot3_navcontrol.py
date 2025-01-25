import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import sqrt, atan2, cos, sin, pi

class TurtleBot3NavControl(Node):
    def __init__(self):
        super().__init__('turtlebot3_navcontrol')

        # Publisher to control robot velocity
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber for LiDAR data
        self.lidar_subscriber = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        self.robot_position = [12, 12]  # Starting position (inches)
        self.target_position = None
        self.grid_size = 55  # Size of the arena in inches
        self.lidar_data = []
        self.reached_goal = False
        self.angle_aligned = False  # Track if the robot is facing the correct direction
        self.current_angle = 0.0  # Track robot's heading in radians

        # Timer to control main loop
        self.timer = self.create_timer(0.1, self.main_loop)

        self.get_target_coordinates()

    def get_target_coordinates(self):
        while True:
            try:
                # Ask user for target coordinates
                user_input = input("Enter target coordinates (x y) within the 55x55 grid: ")
                # Remove any unwanted characters like parentheses and commas
                user_input = user_input.replace('(', '').replace(')', '').replace(',', '')
                self.target_position = list(map(int, user_input.split()))
                if not (0 <= self.target_position[0] <= self.grid_size and 0 <= self.target_position[1] <= self.grid_size):
                    raise ValueError("Coordinates must be within the 0 to 55 range.")
                break  # Valid input, exit loop
            except ValueError as e:
                self.get_logger().error(f"Invalid input: {e}. Please try again.")

    def lidar_callback(self, msg):
        # Update LiDAR data
        self.lidar_data = msg.ranges

    def is_path_clear(self):
        # Check for obstacles in a defined range
        if not self.lidar_data:
            return True  # Assume path is clear if no data available
        min_distance = min(self.lidar_data)
        return min_distance > 0.5  # 0.5 meters clearance threshold

    def calculate_direction(self, current, target):
        # Calculate the angle and distance to the target
        dx = target[0] - current[0]
        dy = target[1] - current[1]
        distance = sqrt(dx ** 2 + dy ** 2)
        target_angle = atan2(dy, dx)
        return distance, target_angle

    def move_robot(self, linear, angular):
        # Publish velocity commands
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.velocity_publisher.publish(cmd)

    def main_loop(self):
        if self.reached_goal or not self.target_position:
            return

        distance, angle_to_target = self.calculate_direction(self.robot_position, self.target_position)
        angle_error = angle_to_target - self.current_angle

        # Normalize angle to [-pi, pi]
        angle_error = (angle_error + pi) % (2 * pi) - pi

        if not self.angle_aligned:
            if abs(angle_error) > 0.1:  # Rotate until facing the correct heading
                angular_speed = 0.5 * angle_error  # Proportional control (Kp = 0.5)
                self.move_robot(0.0, angular_speed)
                self.current_angle += angular_speed * 0.1  # Update angle based on rotation
                self.current_angle = (self.current_angle + pi) % (2 * pi) - pi  # Normalize
            else:
                self.angle_aligned = True  # The robot is now facing the correct direction
        else:
            if distance < 1.0:  # Dead zone of 1 inch
                self.get_logger().info("Target reached!")
                self.move_robot(0.0, 0.0)
                self.reached_goal = True
                return

            if self.is_path_clear():
                linear_speed = 0.2  # Constant linear speed
                self.move_robot(linear_speed, 0.0)

                # Update robot position
                delta_x = cos(self.current_angle) * 0.2  # 0.2 is the step distance
                delta_y = sin(self.current_angle) * 0.2
                self.robot_position[0] += delta_x
                self.robot_position[1] += delta_y
            else:
                self.get_logger().info("Obstacle detected! Adjusting course.")
                self.move_robot(0.0, 0.5)  # Turn in place
                self.angle_aligned = False  # Re-align angle if obstacle changes path


def main():
    rclpy.init()
    node = TurtleBot3NavControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
