import rclpy
import rclpy.time
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer, CancelResponse

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from differentialdrive_msgs.msg import DifferentialDriveStamped
from geometry_msgs.msg import PointStamped
from navigation_action_interfaces.action import FollowPath

from transforms3d.euler import quat2euler

import numpy as np

from scipy.interpolate import splprep, splev, interp1d


class FeedbackLinearizationController(Node):
    def __init__(self):
        super().__init__("feedback_linearization_controller")
        self.car_name = self.get_namespace()[1:]

        self.target_frame = (
            self.declare_parameter("target_frame", "map")
            .get_parameter_value()
            .string_value
        )

        self.declare_parameter("control_frequency", 20)
        self.control_frequency = (
            self.get_parameter("control_frequency").get_parameter_value().integer_value
        )
        self.control_rate = self.create_rate(self.control_frequency)

        self.declare_parameter("d_epsilon", 0.0)
        self.d_epsilon: float = (
            self.get_parameter("d_epsilon").get_parameter_value().double_value
        )
        self.declare_parameter("angle_epsilon", 0.0)
        self.angle_epsilon = self.get_parameter("angle_epsilon").value
        self.declare_parameter("lookahead_distance", 0.0)
        self.lookahead_distance = self.get_parameter("lookahead_distance").value

        self.declare_parameter("max_linear_vel", 0.0)
        self.max_linear_vel = (
            self.get_parameter("max_linear_vel").get_parameter_value().double_value
        )
        self.declare_parameter("max_rotational_vel", 0.0)
        self.max_rotational_vel = (
            self.get_parameter("max_rotational_vel").get_parameter_value().double_value
        )

        self.declare_parameter("n_spline_samples", 1000)
        self.n_spline_samples = (
            self.get_parameter("n_spline_samples").get_parameter_value().integer_value
        )
        self.declare_parameter("lookahead_search_ahead", 1.0)
        self.lookahead_search_ahead = (
            self.get_parameter("lookahead_search_ahead")
            .get_parameter_value()
            .double_value
        )
        self.declare_parameter("lookahead_search_behind", 0.2)
        self.lookahead_search_behind = (
            self.get_parameter("lookahead_search_behind")
            .get_parameter_value()
            .double_value
        )

        self.declare_parameter("robot_names", [self.car_name])
        self.all_robots = (
            self.get_parameter("robot_names").get_parameter_value().string_array_value
        )

        self.declare_parameter("collision_radius", 0.6)
        self.collision_radius = (
            self.get_parameter("collision_radius").get_parameter_value().double_value
        )

        self.declare_parameter("collision_break_factor", 0.75)
        self.collision_break_factor = (
            self.get_parameter("collision_break_factor")
            .get_parameter_value()
            .double_value
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.path_following_server = ActionServer(
            self,
            FollowPath,
            "follow_path_action",
            self.action_execute_callback,
            cancel_callback=self.cancel_callback,
        )

        # Publish lookahead point for debugging (to replace target)
        self.lookahead_pub = self.create_publisher(PointStamped, "lookahead", 1)

        self.command_pub = self.create_publisher(
            DifferentialDriveStamped, "speed_command", 1
        )

        self.goal_position = None  # end of the path
        self.path_raw = None  # type of action request msg
        self.path_spline = None  # spline returned by scipy
        self.path_length = 0.0  # length of the requested path
        self.u_to_arc_length: interp1d
        self.arc_length_to_u: interp1d
        self.u_prev = 0.0  # previous linear velocity
        self.s_traveled = 0.0  # arc length along the path traveled so far

    def cancel_callback(self, cancel_request):
        self.get_logger().info(
            f"Received cancel request for {self.car_name} follow path"
        )
        self.stop_robot()
        # Add custom logic here to decide whether to accept or reject
        # For example, always accept:
        return CancelResponse.ACCEPT

    def get_robot_pose(self, robot_name: str) -> tuple[float, float, float] | None:
        from_frame_rel = robot_name
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

    def setup_splines(self) -> None:
        if self.path_raw is None:
            return
        x = np.array([pt.point.x for pt in self.path_raw])
        y = np.array([pt.point.y for pt in self.path_raw])
        self.path_spline, _ = splprep([x, y], s=0)

        u_fine = np.linspace(0, 1, self.n_spline_samples)
        x_fine, y_fine = splev(u_fine, self.path_spline)
        distances = np.sqrt(np.diff(x_fine) ** 2 + np.diff(y_fine) ** 2)
        arc_length = np.concatenate(([0], np.cumsum(distances)))
        self.path_length = arc_length[-1]
        self.get_logger().debug(f"Path length: {self.path_length}")

        self.arc_length_to_u = interp1d(arc_length, u_fine)
        self.u_to_arc_length = interp1d(u_fine, arc_length)
        self.s_traveled = 0.0

    def get_lookahead_point(self, position: np.ndarray) -> np.ndarray:
        s_min = max(0.0, self.s_traveled - self.lookahead_search_behind)
        s_max = min(self.path_length, self.s_traveled + self.lookahead_search_ahead)

        u_min = self.arc_length_to_u(s_min)
        u_max = self.arc_length_to_u(s_max)
        u_vals = np.linspace(u_min, u_max, self.n_spline_samples)

        spline_points = np.array(splev(u_vals, self.path_spline))
        distances = np.sqrt(
            (spline_points[0] - position[0]) ** 2
            + (spline_points[1] - position[1]) ** 2
        )
        idx_min = np.argmin(distances)
        u_closest = u_vals[idx_min]
        s_closest = self.u_to_arc_length(u_closest)

        # enforce monotonic progress
        self.s_traveled = max(self.s_traveled, s_closest)

        s_ahead = min(s_closest + self.lookahead_distance, self.path_length)
        u_ahead = self.arc_length_to_u(s_ahead)
        return np.array(splev(u_ahead, self.path_spline))

    def is_goal_reached(self, position):
        dist_to_goal = np.linalg.norm(position - self.goal_position)
        return dist_to_goal < self.d_epsilon and self.s_traveled > self.path_length / 2

    def compute_control(self, position, heading, now):
        if self.path_spline is None:
            return 0.0, 0.0

        lookahead_pt: np.ndarray = self.get_lookahead_point(position)

        if lookahead_pt is None:
            return 0.0, 0.0  # No path

        lookahead_msg = PointStamped()
        lookahead_msg.header.frame_id = self.target_frame
        lookahead_msg.point.x = float(lookahead_pt[0])
        lookahead_msg.point.y = float(lookahead_pt[1])
        self.lookahead_pub.publish(lookahead_msg)

        away_pt = position + np.array(
            [
                np.cos(heading) * self.u_prev,
                np.sin(heading) * self.u_prev,
            ]
        )
        v = lookahead_pt - away_pt

        for robot in self.all_robots:
            if robot == self.car_name:
                continue
            other_robot_pose = self.get_robot_pose(robot)
            if other_robot_pose is None:
                continue
            other_position = np.array(other_robot_pose[:2])
            repulsion_vec = away_pt - other_position
            collision_dist = np.linalg.norm(repulsion_vec)
            if collision_dist > self.collision_radius:
                continue
            dot_prod = np.dot(v, repulsion_vec)
            if dot_prod > 0:
                continue
            v_along_repulsion = dot_prod * repulsion_vec / (collision_dist**2)
            v -= self.collision_break_factor * v_along_repulsion

        target_heading = np.arctan2(v[1], v[0])
        heading_error = target_heading - heading
        heading_error = np.mod(heading_error, 2 * np.pi)
        if heading_error > np.pi:
            heading_error -= 2 * np.pi
        elif heading_error < -np.pi:
            heading_error += 2 * np.pi

        if abs(heading_error) < self.angle_epsilon:
            u = np.cos(heading) * v[0] + np.sin(heading) * v[1]
            w = (np.cos(heading) * v[1] - np.sin(heading) * v[0]) / np.linalg.norm(v)
        else:  # rotate on the spot
            u = 0.0
            w = heading_error

        # linear_vel = self.linear_pid.update(dist_error, now)
        u = np.clip(u, 0.0, self.max_linear_vel)
        self.u_prev = u

        # angular_vel = curvature * linear_vel
        w = np.clip(w, -self.max_rotational_vel, self.max_rotational_vel)

        return u, w

    def action_execute_callback(self, goal_handle):
        self.get_logger().info("Received new path to track")
        # Read the path from goal handle
        self.path_raw = goal_handle.request.path
        self.goal_position = np.array(
            [self.path_raw[-1].point.x, self.path_raw[-1].point.y]
        )
        self.get_logger().info(f"Goal position: {self.goal_position}")
        # Splines and path parameterization
        self.setup_splines()

        # Initialize feedback and result messages
        feedback_msg = FollowPath.Feedback()
        result = FollowPath.Result()

        # Run Control Loop
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Goal canceled by client")
                self.stop_robot()
                goal_handle.canceled()
                return result

            pose = self.get_robot_pose(self.car_name)
            if pose is None:
                self.get_logger().warn("Current pose unavailable, waiting...")
                self.control_rate.sleep()
                continue

            position = np.array(pose[:2])
            heading = pose[2]

            time = self.get_clock().now()
            lin_cmd, ang_cmd = self.compute_control(position, heading, time)
            self.publish_speed_command(lin_cmd, ang_cmd)

            # Compute some feedback
            goal_handle.publish_feedback(feedback_msg)

            # Check if goal reached (within lookahead distance of last path point)
            if self.is_goal_reached(position):
                self.get_logger().info("Goal reached successfully!")
                self.stop_robot()
                goal_handle.succeed()
                return result

            self.control_rate.sleep()

    def stop_robot(self):
        self.publish_speed_command(0.0, 0.0)

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


def main():
    rclpy.init()
    node = FeedbackLinearizationController()

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
