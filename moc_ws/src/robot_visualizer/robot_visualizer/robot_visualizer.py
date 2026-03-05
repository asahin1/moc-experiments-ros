import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class RobotVisualizer(Node):
    def __init__(self):
        super().__init__("robot_visualizer")
        self.car_name = self.get_namespace()[1:]

        self.target_frame = (
            self.declare_parameter("target_frame", "map")
            .get_parameter_value()
            .string_value
        )
        self.robot_radius = (
            self.declare_parameter("robot_radius", 0.3)
            .get_parameter_value()
            .double_value
        )
        self.robot_height = (
            self.declare_parameter("robot_height", 0.05)
            .get_parameter_value()
            .double_value
        )
        self.robot_color = (
            self.declare_parameter("robot_color", [0.1, 0.4, 1.0, 1.0])
            .get_parameter_value()
            .double_array_value
        )
        self.selected_robot_color = (
            self.declare_parameter("selected_robot_color", [0.1, 0.4, 1.0, 1.0])
            .get_parameter_value()
            .double_array_value
        )

        self.color = self.robot_color

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.selected_sub = self.create_subscription(
            String, "/selected_robot", self.set_color, 10
        )

        self.pose = None
        self.path = Path()

        self.marker_publisher = self.create_publisher(Marker, "marker", 10)
        self.path_publisher = self.create_publisher(Path, "path", 1)

        self.rate = 10
        timer_period = 1 / self.rate
        self.timer = self.create_timer(timer_period, self.vis_callback)

    def set_color(self, msg):
        if msg.data == self.car_name:
            self.color = self.selected_robot_color
        else:
            self.color = self.robot_color

    def update_pose(self):
        from_frame_rel = self.car_name
        to_frame_rel = self.target_frame
        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel, from_frame_rel, rclpy.time.Time()
            )
            self.pose = Pose()
            self.pose.position.x = t.transform.translation.x
            self.pose.position.y = t.transform.translation.y
        except TransformException as ex:
            self.get_logger().debug(
                f"Could not transform {from_frame_rel} to {to_frame_rel}: {ex}"
            )
            return

    def update_path(self):
        old_poses = self.path.poses
        new_pose = PoseStamped()
        new_pose.header.frame_id = self.target_frame
        new_pose.pose = self.pose
        self.path = Path()
        self.path.header.frame_id = self.target_frame
        self.path.poses = old_poses + [new_pose]

    def vis_callback(self):
        # Update info
        self.update_pose()
        if self.pose is None:
            return
        self.update_path()

        # Publish marker
        marker = Marker()
        marker.header.frame_id = self.target_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "disk"
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        # Disk size
        marker.scale.x = self.robot_radius * 2.0
        marker.scale.y = self.robot_radius * 2.0
        marker.scale.z = self.robot_height

        # Position
        marker.pose = self.pose

        # Color (blue)
        marker.color.r = self.color[0]
        marker.color.g = self.color[1]
        marker.color.b = self.color[2]
        marker.color.a = self.color[3]

        self.marker_publisher.publish(marker)

        # Publish path
        self.path_publisher.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = RobotVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
