import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from differentialdrive_msgs.msg import DifferentialDriveStamped


class MotorController(Node):
    def __init__(self):
        super().__init__("motor_controller")
        self.car_name = self.get_namespace()[1:]

        self.get_logger().info(
            f"Initializing motor controller for cars: {self.car_name}"
        )

        self.motor_publisher = self.create_publisher(
            Float32MultiArray, "motor_signals", 10
        )
        self.command_subscriber = self.create_subscription(
            DifferentialDriveStamped,
            "speed_command",
            self.drive_callback,
            10,
        )

    def drive_callback(self, msg):
        """Convert differential drive signals (linear, angular) to 4 motor signals"""
        linear_velocity = (
            -msg.drive.linear
        )  # Negated for new robot assembly with spool mechanism in front
        angular_velocity = msg.drive.rotational

        # Convert to 4 motor signals using proper differential drive math
        motor_signals = self.convert_to_four_signals(linear_velocity, angular_velocity)

        # Publish the 4 motor signals for this specific vehicle
        motor_msg = Float32MultiArray()
        motor_msg.data = motor_signals
        self.motor_publisher.publish(motor_msg)

        # Log the 4 motor signals clearly
        # self.get_logger().info("=" * 50)
        # self.get_logger().info(f"   {vehicle_id} INPUT SIGNALS:")
        # self.get_logger().info(f"   Linear Velocity:  {linear_velocity:.3f}")
        # self.get_logger().info(f"   Angular Velocity: {angular_velocity:.3f}")
        # self.get_logger().info(f"   OUTPUT - 4 MOTOR SIGNALS:")
        # self.get_logger().info(f"   Left Front:  {motor_signals[0]:>6.0f}")
        # self.get_logger().info(f"   Left Rear:   {motor_signals[1]:>6.0f}")
        # self.get_logger().info(f"   Right Front: {motor_signals[2]:>6.0f}")
        # self.get_logger().info(f"   Right Rear:  {motor_signals[3]:>6.0f}")
        # self.get_logger().info("=" * 50)

    def convert_to_four_signals(self, linear_velocity, angular_velocity):
        """
        Convert differential drive to 4 individual motor signals using proper differential drive math

        Differential drive principle:
        - Left wheels: linear_velocity - angular_velocity
        - Right wheels: linear_velocity + angular_velocity

        Returns: [left_front, left_rear, right_front, right_rear]
        """

        # Differential drive calculation
        # For a robot turning right: left wheels go faster, right wheels go slower
        # For a robot turning left: right wheels go faster, left wheels go slower
        angular_coeff = 0.01
        left_speed = linear_velocity - angular_velocity * angular_coeff
        right_speed = linear_velocity + angular_velocity * angular_coeff

        # Scale to motor command range (adjust based on your robot's requirements)
        # Start with a conservative scaling factor and tune as needed
        scale_factor = 400000  # You may need to adjust this value

        left_scaled = int(left_speed * scale_factor)
        right_scaled = int(right_speed * scale_factor)

        # Apply safety limits to prevent motor saturation
        max_motor_value = 3000  # Adjust based on your robot's motor limits
        left_scaled = -1.0 * max(min(left_scaled, max_motor_value), -max_motor_value)
        right_scaled = -1.0 * max(min(right_scaled, max_motor_value), -max_motor_value)
        # self.get_logger().info(f"   right_scaled: {right_scaled:>6.0f} {linear_velocity}")
        # self.get_logger().info("+" * 50)

        # For 4WD: front and rear wheels on each side move together
        left_front = left_scaled
        left_rear = left_scaled
        right_front = right_scaled
        right_rear = right_scaled

        return [
            float(left_front),
            float(left_rear),
            float(right_front),
            float(right_rear),
        ]


def main(args=None):
    rclpy.init(args=args)
    robot_controller_node = MotorController()
    rclpy.spin(robot_controller_node)
    robot_controller_node.destroy_node()
    rclpy.shutdown()
