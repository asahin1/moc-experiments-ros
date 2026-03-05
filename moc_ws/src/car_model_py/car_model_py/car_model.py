from differentialdrive_msgs.msg import DifferentialDriveStamped
from geometry_msgs.msg import Pose

import numpy as np

import rclpy
from rclpy.node import Node

from transforms3d.euler import euler2quat

import math


def quaternion_from_euler(ai, aj, ak):
    return tuple(euler2quat(ai, aj, ak, "sxyz")[np.array([1, 2, 3, 0])])


class CarModel(Node):
    def __init__(self):
        super().__init__("car_model")
        self.car_name = self.get_namespace()[1:]
        self.declare_parameter("init_pose", [0.0, 0.0, 0.0])
        init_pose = self.get_parameter("init_pose").value
        self.model_rate = (
            self.declare_parameter("model_rate", 10).get_parameter_value().integer_value
        )
        self.x = init_pose[0]
        self.y = init_pose[1]
        self.theta = init_pose[2]
        self.command_subscription = self.create_subscription(
            DifferentialDriveStamped,
            "speed_command",
            self.apply_control,
            qos_profile=10,
        )
        self.pose_publisher = self.create_publisher(Pose, "pose", 1)

        timer_period = 1 / self.model_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Pose()
        msg.position.x = self.x
        msg.position.y = self.y
        msg.position.z = 0.0

        q = quaternion_from_euler(0, 0, self.theta)
        msg.orientation.x = q[0]
        msg.orientation.y = q[1]
        msg.orientation.z = q[2]
        msg.orientation.w = q[3]

        self.pose_publisher.publish(msg)
        # self.get_logger().info(
        #     f"[{self.car_name}] Pose: ({self.x:.2f}, {self.y:.2f}, {self.theta:.2f})"
        # )

    def apply_control(self, control_msg):
        move_dist = (1 / self.model_rate) * control_msg.drive.linear
        move_angle = (1 / self.model_rate) * control_msg.drive.rotational

        self.theta = np.mod(self.theta + move_angle, 2 * np.pi)

        move_x = move_dist * math.cos(self.theta)
        move_y = move_dist * math.sin(self.theta)

        self.x += move_x
        self.y += move_y


def main():
    rclpy.init()
    node = CarModel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by KeyboardInterrupt")
    finally:
        node.destroy_node()  # Destroy the node to release resources
        rclpy.shutdown()  # Shut down rclpy
