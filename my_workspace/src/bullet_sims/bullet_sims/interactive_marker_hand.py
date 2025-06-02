#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from interactive_markers import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from geometry_msgs.msg import Quaternion, PoseStamped

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener



class InteractiveMarkerNode(Node):
    def __init__(self):
        super().__init__('interactive_marker_node')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create an interactive marker server
        self.server = InteractiveMarkerServer(self, 'hand_target_marker')
        
        self.pose_publisher = self.create_publisher(PoseStamped, 'hand_target_pose', 10)
        
        self._hand_target_timer = self.create_timer(1.0, self.timer_at_hand_target)
        
        self.get_logger().info('Interactive marker server started')
        
        

    def timer_at_hand_target(self):
        """Timer callback to update the hand target marker"""
        try:
            transform = self.tf_buffer.lookup_transform('base_link', 
                                                        'arm_right_7_link', 
                                                        rclpy.time.Time())
            
            self.get_logger().debug(f'Found right arm transform: '
                                 f'pos=({transform.transform.translation.x:.3f}, '
                                 f'{transform.transform.translation.y:.3f}, '
                                 f'{transform.transform.translation.z:.3f})')
            
            # Create the interactive marker at this position
            self.create_6dof_marker(transform)
            
            # Apply changes and start serving
            self.server.applyChanges()
            
            self.get_logger().debug('Interactive marker created at right hand position')
            
            # Cancel the timer since we only need to do this once
            #self._hand_target_timer.cancel()
        except TransformException as e:
            self.get_logger().error(f'Could not transform from base_link to hand_target: {e}')


    def process_feedback(self, feedback):
        """Process feedback from the interactive marker"""
        p = feedback.pose.position
        o = feedback.pose.orientation
        self.get_logger().info(
            f'{feedback.marker_name} moved to position: x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}, '
            f'orientation: x={o.x:.3f}, y={o.y:.3f}, z={o.z:.3f}, w={o.w:.3f}'
        )
        
        # Publish the updated pose
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'base_link'
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose.position = p
        pose_stamped.pose.orientation = o
        self.pose_publisher.publish(pose_stamped)
        self.get_logger().info('Published updated hand target pose')

    def create_6dof_marker(self, transform):
        """Create a 6-DOF interactive marker"""
        # Create the main interactive marker
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = 'base_link'
        int_marker.name = 'hand_target'
        int_marker.description = '6-DOF Hand Target Control'
        int_marker.scale = 0.2

        # Set initial position (slightly in front of robot)
        int_marker.pose.position.x = transform.transform.translation.x
        int_marker.pose.position.y = transform.transform.translation.y
        int_marker.pose.position.z = transform.transform.translation.z
        int_marker.pose.orientation.x = transform.transform.rotation.x
        int_marker.pose.orientation.y = transform.transform.rotation.y
        int_marker.pose.orientation.z = transform.transform.rotation.z
        int_marker.pose.orientation.w = transform.transform.rotation.w

        # Create a box marker for visualization
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.scale.x = 0.1
        box_marker.scale.y = 0.1
        box_marker.scale.z = 0.1
        box_marker.color.r = 0.0
        box_marker.color.g = 0.8
        box_marker.color.b = 0.0
        box_marker.color.a = 0.8

        # Create a non-interactive control that contains the box
        box_control = InteractiveMarkerControl()
        box_control.always_visible = True
        box_control.markers.append(box_marker)
        int_marker.controls.append(box_control)

        # Add 6-DOF controls
        self.add_6dof_controls(int_marker)

        # Insert the marker into the server
        self.server.insert(int_marker, feedback_callback=self.process_feedback)

    def add_6dof_controls(self, int_marker):
        """Add 6-DOF controls to the interactive marker"""
        
        # X-axis translation
        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 1.0
        control.orientation.y = 0.0
        control.orientation.z = 0.0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        # Y-axis translation  
        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        # Z-axis translation
        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 0.0
        control.orientation.z = 1.0
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        # X-axis rotation
        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 1.0
        control.orientation.y = 0.0
        control.orientation.z = 0.0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        # Y-axis rotation
        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        # Z-axis rotation
        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 0.0
        control.orientation.z = 1.0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)


def main():
    rclpy.init(args=sys.argv)
    
    node = InteractiveMarkerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.server.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()