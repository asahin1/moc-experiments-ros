import math
import yaml

from geometry_msgs.msg import TransformStamped

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from tf2_ros import TransformBroadcaster

from mocap4r2_msgs.msg import RigidBodies



class Mocap4r2TF2Broadcaster(Node):

    def __init__(self):
        super().__init__('tf2_broadcaster')


        # Declare the parameter
        self.declare_parameter('tracked_rigid_bodies', '{}') # Creates parameter with name "tracked_rigid_bodies" with default value of {}
        tracked_param = self.get_parameter('tracked_rigid_bodies').get_parameter_value().string_value
        self.get_logger().info(f'tracked parameter string: {tracked_param}')
        self.tracked_rigid_bodies = yaml.safe_load(tracked_param)

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to a /rigid_bodies topic and call handle_tf2_data callback function on each message
        self.subscription = self.create_subscription(
            RigidBodies,
            f'/rigid_bodies',
            self.handle_tf2_data,
            10)
        self.subscription  # prevent unused variable warning
   
    def handle_tf2_data (self, msg):
        self.get_logger().info('Received message with frame number: %d' % msg.frame_number)
        for body in msg.rigidbodies:
            self.get_logger().info('Received body with rigid body name: %s' % body.rigid_body_name)
            if body.rigid_body_name in self.tracked_rigid_bodies:
                pose = body.pose
                t = TransformStamped()
                self.get_logger().info('\t{} ({}): {}, {}, {}'.format(
                    body.rigid_body_name,
                    self.tracked_rigid_bodies[body.rigid_body_name],
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                    pose.orientation
                ))
                # Read message content and assign it to corresponding tf variables
                t.header.stamp = msg.header.stamp
                t.header.frame_id = msg.header.frame_id
                t.child_frame_id = self.tracked_rigid_bodies[body.rigid_body_name] # Using OUR frame names
                t.transform.translation.x = pose.position.x
                t.transform.translation.y = pose.position.y
                t.transform.translation.z = pose.position.z
                t.transform.rotation = pose.orientation

                self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = Mocap4r2TF2Broadcaster()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
