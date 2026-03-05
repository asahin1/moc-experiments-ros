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
        self.declare_parameter('tracked_rigid_bodies', '{}')
        tracked_param = self.get_parameter('tracked_rigid_bodies').get_parameter_value().string_value
        
        # DEBUG: Log the raw parameter string
        self.get_logger().info(f'Raw tracked parameter string: "{tracked_param}"')
        
        try:
            self.tracked_rigid_bodies = yaml.safe_load(tracked_param)
            # DEBUG: Log the parsed dictionary
            self.get_logger().info(f'Parsed tracked_rigid_bodies: {self.tracked_rigid_bodies}')
            self.get_logger().info(f'Type of tracked_rigid_bodies: {type(self.tracked_rigid_bodies)}')
            
            # DEBUG: Log each key and its type
            if self.tracked_rigid_bodies:
                for key, value in self.tracked_rigid_bodies.items():
                    self.get_logger().info(f'Key: {key} (type: {type(key)}), Value: {value} (type: {type(value)})')
            else:
                self.get_logger().warn('tracked_rigid_bodies is empty!')
                
        except yaml.YAMLError as e:
            self.get_logger().error(f'Failed to parse YAML: {e}')
            self.tracked_rigid_bodies = {}
        
        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribe to a /rigid_bodies topic
        self.subscription = self.create_subscription(
            RigidBodies,
            f'/rigid_bodies',
            self.handle_tf2_data,
            10)
        self.subscription  # prevent unused variable warning
   
    def handle_tf2_data(self, msg):
        self.get_logger().info('Received message with frame number: %d' % msg.frame_number)
        
        for body in msg.rigidbodies:
            body_name = body.rigid_body_name
            self.get_logger().info(f'Received body with rigid body name: {body_name} (type: {type(body_name)})')
            
            # DEBUG: Check if body name is in tracked bodies
            self.get_logger().info(f'Checking if {body_name} is in {list(self.tracked_rigid_bodies.keys())}')
            
            # Try both direct comparison and string conversion
            found_match = False
            target_frame = None
            
            if body_name in self.tracked_rigid_bodies:
                found_match = True
                target_frame = self.tracked_rigid_bodies[body_name]
                self.get_logger().info(f'Direct match found: {body_name} -> {target_frame}')
            elif str(body_name) in self.tracked_rigid_bodies:
                found_match = True
                target_frame = self.tracked_rigid_bodies[str(body_name)]
                self.get_logger().info(f'String match found: {str(body_name)} -> {target_frame}')
            else:
                self.get_logger().info(f'No match found for {body_name}')
                
            if found_match:
                pose = body.pose
                
                # Check for NaN values
                if (math.isnan(pose.position.x) or math.isnan(pose.position.y) or 
                    math.isnan(pose.position.z)):
                    self.get_logger().warn(f'Invalid pose for {body_name}, skipping')
                    continue
                
                t = TransformStamped()
                self.get_logger().info(f'Publishing TF: {body_name} -> {target_frame} at ({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})')
                
                # Read message content and assign it to corresponding tf variables
                t.header.stamp = msg.header.stamp
                t.header.frame_id = msg.header.frame_id
                t.child_frame_id = target_frame
                t.transform.translation.x = pose.position.x
                t.transform.translation.y = pose.position.y
                t.transform.translation.z = pose.position.z
                t.transform.rotation = pose.orientation
                
                # DEBUG: Log what we're about to broadcast
                self.get_logger().info(f'Broadcasting TF: {t.header.frame_id} -> {t.child_frame_id}')
                
                self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = Mocap4r2TF2Broadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
