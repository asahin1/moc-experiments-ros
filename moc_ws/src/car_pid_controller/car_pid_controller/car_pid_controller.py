import rclpy
import rclpy.time
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from differentialdrive_msgs.msg import DifferentialDriveStamped
from geometry_msgs.msg import PointStamped

from transforms3d.euler import quat2euler

import numpy as np


class PID:
    def __init__(self, kp, ki, kd, i_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_limit = i_limit

        self.i = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def reset(self):
        self.i = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def update(self, error, now):
        if self.prev_time is None:
            self.prev_time = now
            self.prev_error = error
            return 0.0

        dt = (now - self.prev_time).nanoseconds * 1e-9
        dt = max(min(dt, 0.1), 1e-3)

        self.i += error * dt
        if self.i_limit is not None:
            self.i = max(min(self.i, self.i_limit), -self.i_limit)
        d = (error - self.prev_error) / dt

        self.prev_error = error
        self.prev_time = now

        u = self.kp * error + self.ki * self.i + self.kd * d
        print("U: ", u)

        return u


class CarPIDController(Node):
    def __init__(self):
        super().__init__("car_pid_controller")
        self.car_name = self.get_namespace()[1:]

        self.target_frame = (
            self.declare_parameter("target_frame", "map")
            .get_parameter_value()
            .string_value
        )

        self.declare_parameter("using_real_robot", True)
        self.using_real_robot = self.get_parameter("using_real_robot").value

        self.declare_parameter("pid_rate", 20)
        self.pid_rate = (
            self.get_parameter("pid_rate").get_parameter_value().integer_value
        )
        self.rate = self.create_rate(self.pid_rate)

        self.declare_parameter("pid_linear_gains", [0.0, 0.0, 0.0])
        self.pid_linear_gains = (
            self.get_parameter("pid_linear_gains")
            .get_parameter_value()
            .double_array_value
        )
        self.declare_parameter("pid_rotational_gains", [0.0, 0.0, 0.0])
        self.pid_rotational_gains = (
            self.get_parameter("pid_rotational_gains")
            .get_parameter_value()
            .double_array_value
        )
        self.declare_parameter("d_epsilon", 0.0)
        self.d_epsilon = self.get_parameter("d_epsilon").value
        self.declare_parameter("angle_epsilon", 0.0)
        self.angle_epsilon = self.get_parameter("angle_epsilon").value
        self.declare_parameter("max_linear_vel", 0.0)
        self.max_linear_vel = self.get_parameter("max_linear_vel").value
        self.declare_parameter("max_rotational_vel", 0.0)
        self.max_rotational_vel = self.get_parameter("max_rotational_vel").value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.distance_pid = PID(*self.pid_linear_gains)
        self.heading_pid = PID(*self.pid_rotational_gains)

        self.target = None

        self.target_sub = self.create_subscription(
            PointStamped, "target", self.set_target, 10
        )
        self.command_pub = self.create_publisher(
            DifferentialDriveStamped, "speed_command", 1
        )

        timer_period = 1 / self.pid_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def set_target(self, target_msg):
        new_target = np.array([target_msg.point.x, target_msg.point.y])

        if self.target is None or np.linalg.norm(new_target - self.target) > 1e-3:
            self.distance_pid.reset()
            self.heading_pid.reset()

        self.target = new_target

    def get_car_pose(self):
        from_frame_rel = self.car_name
        to_frame_rel = self.target_frame
        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel, from_frame_rel, rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().debug(
                f"Could not transform {from_frame_rel} to {to_frame_rel}: {ex}"
            )
            return None
        quat_msg = t.transform.rotation
        quat = (quat_msg.w, quat_msg.x, quat_msg.y, quat_msg.z)
        _, _, orientation = quat2euler(quat)
        return (t.transform.translation.x, t.transform.translation.y, orientation)

    def publish_speed_command(self, lin_vel, rot_vel):
        msg = DifferentialDriveStamped()
        msg.header.frame_id = self.car_name
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.linear = lin_vel
        msg.drive.rotational = rot_vel

        self.command_pub.publish(msg)
        self.get_logger().debug(
            f"[{self.car_name}] Command: linear={lin_vel:.3f}, rotational={rot_vel:.3f}"
        )

    def transform_heading_error(self, error_heading):
        mod_val = np.mod(error_heading, 360)
        if mod_val > 180:
            mod_val -= 360
        elif mod_val < -180:
            mod_val += 360
        error_heading = mod_val
        return error_heading

    def compute_controls(self, pose):
        position = np.array(pose[:2])
        orientation = np.rad2deg(pose[2])

        v = self.target - position
        dist = np.linalg.norm(v)

        if dist < self.d_epsilon:  # if target was reached
            return 0.0, 0.0

        los_angle = np.rad2deg(np.arctan2(v[1], v[0]))

        error_dist = dist

        if los_angle > 180:
            los_angle = -(360 - los_angle)
        if orientation > 180:
            orientation = -(360 - orientation)

        error_heading = los_angle - orientation
        error_heading = self.transform_heading_error(error_heading)

        now = self.get_clock().now()
        heading_factor = max(0.0, np.cos(np.deg2rad(error_heading)))
        linear_velocity = heading_factor * self.distance_pid.update(error_dist, now)
        rotational_velocity = self.heading_pid.update(error_heading, now)

        self.get_logger().info(f"l_vel:{linear_velocity}, r_vel:{rotational_velocity}")

        if self.using_real_robot:
            linear_velocity = -linear_velocity

        linear_velocity = float(
            max(
                min(linear_velocity, self.max_linear_vel),
                -self.max_linear_vel,
            )
        )
        rotational_velocity = float(
            max(
                min(rotational_velocity, self.max_rotational_vel),
                -self.max_rotational_vel,
            )
        )

        return linear_velocity, rotational_velocity

    def timer_callback(self):
        if self.target is not None:
            pose = self.get_car_pose()
            if pose is not None:
                linear, rotational = self.compute_controls(pose)
                self.publish_speed_command(linear, rotational)
            else:
                self.get_logger().debug("Pose not available.")
        else:
            self.get_logger().debug("No target set.")


def main():
    rclpy.init()
    node = CarPIDController()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by KeyboardInterrupt")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
