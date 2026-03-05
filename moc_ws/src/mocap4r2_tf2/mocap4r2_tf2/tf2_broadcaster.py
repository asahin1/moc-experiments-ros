import math
import yaml
from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from tf2_ros import TransformBroadcaster
from mocap4r2_msgs.msg import RigidBodies

class Mocap4r2TF2BroadcasterMultiAgent(Node):
    def __init__(self):
        super().__init__('tf2_broadcaster_multi_agent')
        
        # Declare parameters for multi-agent setup
        self.declare_parameter('tracked_rigid_bodies', '{}')
        self.declare_parameter('active_vehicles', [])
        self.declare_parameter('base_frame', 'world')
        
        # Parse tracked rigid bodies parameter
        tracked_param = self.get_parameter('tracked_rigid_bodies').get_parameter_value().string_value
        
        # Get base frame
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        
        # Get active vehicles list (optional parameter)
        self.active_vehicles = self.get_parameter('active_vehicles').get_parameter_value().integer_array_value
        
        # DEBUG: Log the raw parameter string
        self.get_logger().info(f'Raw tracked parameter string: "{tracked_param}"')
        self.get_logger().info(f'Base frame: "{self.base_frame}"')
        self.get_logger().info(f'Active vehicles: {self.active_vehicles}')
        
        try:
            self.tracked_rigid_bodies = yaml.safe_load(tracked_param)
            # DEBUG: Log the parsed dictionary
            self.get_logger().info(f'Parsed tracked_rigid_bodies: {self.tracked_rigid_bodies}')
            self.get_logger().info(f'Vehicle IDs: {list(self.tracked_rigid_bodies.keys())}')
            self.get_logger().info(f'Vehicle frames: {list(self.tracked_rigid_bodies.values())}')
            
            # Log which vehicles will be tracked
            if self.active_vehicles:
                active_frames = [self.tracked_rigid_bodies.get(vid, f"unknown_{vid}") 
                               for vid in self.active_vehicles if vid in self.tracked_rigid_bodies]
                self.get_logger().info(f'Will track active vehicles: {active_frames}')
            else:
                self.get_logger().info(f'Will track all configured vehicles: {list(self.tracked_rigid_bodies.values())}')
                
        except yaml.YAMLError as e:
            self.get_logger().error(f'Failed to parse YAML: {e}')
            self.tracked_rigid_bodies = {}
        
        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Keep track of last seen vehicles for monitoring
        self.last_seen_vehicles = {}
        
        # Subscribe to rigid_bodies topic
        self.subscription = self.create_subscription(
            RigidBodies,
            '/rigid_bodies',
            self.handle_tf2_data,
            10)
        self.subscription  # prevent unused variable warning
        
        # Create a timer for periodic status reporting
        self.status_timer = self.create_timer(5.0, self.report_status)
   
    def handle_tf2_data(self, msg):
        self.get_logger().debug(f'Received message with frame number: {msg.frame_number}')
        
        vehicles_found_this_frame = []
        
        for body in msg.rigidbodies:
            # Try to convert rigid body name to integer to match YAML keys
            try:
                body_name_int = int(body.rigid_body_name)
                self.get_logger().debug(f'Processing body with ID: {body_name_int}')
                
                # Check if this vehicle should be tracked
                should_track = True
                if self.active_vehicles:
                    should_track = body_name_int in self.active_vehicles
                
                if body_name_int in self.tracked_rigid_bodies and should_track:
                    pose = body.pose
                    target_frame = self.tracked_rigid_bodies[body_name_int]
                    
                    # Check for NaN values
                    if (math.isnan(pose.position.x) or math.isnan(pose.position.y) or 
                        math.isnan(pose.position.z)):
                        self.get_logger().warn(f'Invalid pose for vehicle {target_frame} (ID: {body_name_int}), skipping')
                        continue
                    
                    # Create and populate transform
                    t = TransformStamped()
                    t.header.stamp = msg.header.stamp
                    t.header.frame_id = self.base_frame  # Use configurable base frame
                    t.child_frame_id = target_frame
                    t.transform.translation.x = pose.position.x
                    t.transform.translation.y = pose.position.y
                    t.transform.translation.z = pose.position.z
                    t.transform.rotation = pose.orientation
                    
                    # Broadcast the transform
                    self.tf_broadcaster.sendTransform(t)
                    
                    # Update tracking info
                    vehicles_found_this_frame.append(target_frame)
                    self.last_seen_vehicles[target_frame] = {
                        'timestamp': msg.header.stamp,
                        'position': (pose.position.x, pose.position.y, pose.position.z),
                        'rigid_body_id': body_name_int
                    }
                    
                    self.get_logger().debug(f'Published TF: {self.base_frame} -> {target_frame} at '
                                          f'({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})')
                
                elif body_name_int in self.tracked_rigid_bodies and not should_track:
                    self.get_logger().debug(f'Vehicle ID {body_name_int} found but not in active list')
                else:
                    self.get_logger().debug(f'Unknown vehicle ID: {body_name_int}')
                    
            except ValueError:
                # If conversion fails, rigid body name is not a number
                self.get_logger().warn(f'Could not convert rigid body name to integer: {body.rigid_body_name}')
        
        # Log summary of vehicles found in this frame
        if vehicles_found_this_frame:
            self.get_logger().debug(f'Frame {msg.frame_number}: Found vehicles {vehicles_found_this_frame}')
    
    def report_status(self):
        """Periodically report status of tracked vehicles"""
        if not self.last_seen_vehicles:
            self.get_logger().info("No vehicles currently being tracked")
            return
        
        status_msg = f"Currently tracking {len(self.last_seen_vehicles)} vehicles:"
        for vehicle_frame, info in self.last_seen_vehicles.items():
            pos = info['position']
            status_msg += f"\n  - {vehicle_frame} (ID: {info['rigid_body_id']}): ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})"
        
        self.get_logger().info(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Mocap4r2TF2BroadcasterMultiAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
