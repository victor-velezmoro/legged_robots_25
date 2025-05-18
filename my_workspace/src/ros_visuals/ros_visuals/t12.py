#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import pinocchio as pin
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TwistStamped

class CagePublisher(Node):
    def __init__(self):
        super().__init__('cage_publisher')
        self.br = TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.marker_pub2 = self.create_publisher(Marker, 'visualization_marker2', 10)
        self.twist_pub = self.create_publisher(TwistStamped, 'spatial_twist', 10)
        self.twist_pub2 = self.create_publisher(TwistStamped, 'spatial_twist_world', 10)
        self.twist_pub3 = self.create_publisher(TwistStamped, 'spatial_twist_invers', 10)
        self.twist_pub4 = self.create_publisher(TwistStamped, 'spatial_twist_world_invers', 10)
        self.twist_pub5 = self.create_publisher(TwistStamped, 'spatial_twist_adjoint', 10)
        self.twist_pub6 = self.create_publisher(TwistStamped, 'spatial_twist_world_adjoint', 10)
        self.twist_pub7 = self.create_publisher(TwistStamped, 'spatial_twist_adjoint_invers', 10)
        self.twist_pub8 = self.create_publisher(TwistStamped, 'spatial_twist_world_adjoint_invers', 10)

    
        # Timer to call self.broadcast_transforms every 0.5 seconds
        self.timer = self.create_timer(0.5, self.broadcast_transforms)

        # Define cube corners using SE3 transforms (size = 1 unit cube)
        self.transforms = self.create_cube_transforms()

        # Initialize linear and angular velocities
        self.linear_velocity = np.array([0.1, 0.0, 0.0])  # Translation velocity (m/s)
        self.angular_velocity = np.array([0.0, 0.0, 0.1])  # Rotation velocity (rad/s)

        # Initialize pose for quaternion-based integration
        self.translation_quat = np.array([0.0, 0.0, 0.0])
        self.rotation_quat = pin.Quaternion(1.0, 0.0, 0.0, 0.0)

        # # Initialize pose for exp6-based integration
        self.pose_exp6 = pin.SE3.Identity()
        
        #point as marker
        self.p_corner = np.array([0.0, 0.0, 0.0])
        
        self.spatial_twist = pin.Motion(np.array([1.0, 0.0, 0.0, 0.5, 0.0, 0.0]))  # c V = [ωx, ωy, ωz, vx, vy, vz]^T
        
    def publish_p_corner(self, point, frame_id):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "p_corner"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = point[2]
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1  
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 1.0  
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  
        
        if frame_id == "world":
            self.marker_pub2.publish(marker)
        else:
            self.marker_pub.publish(marker)
            
        
    
        
    def publish_twist(self, transformed_twist, frame_id):
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = frame_id
        twist_msg.twist.linear.x = transformed_twist.linear[0]
        twist_msg.twist.linear.y = transformed_twist.linear[1]
        twist_msg.twist.linear.z = transformed_twist.linear[2]
        twist_msg.twist.angular.x = transformed_twist.angular[0]
        twist_msg.twist.angular.y = transformed_twist.angular[1]
        twist_msg.twist.angular.z = transformed_twist.angular[2]
        
        # Publish the twist
        if frame_id == "world":
            self.twist_pub2.publish(twist_msg)
        else:
            self.twist_pub.publish(twist_msg)

        
    def publish_twist_with_color(self, twist, frame_id):
        """
        Publishes a twist with a specific color in RViz.
        :param twist: pinocchio.Motion (6D twist vector)
        :param frame_id: Frame ID for the twist
        :param color: List of RGB values for the twist visualization
        """
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = frame_id
        twist_msg.twist.linear.x = twist.linear[0]
        twist_msg.twist.linear.y = twist.linear[1]
        twist_msg.twist.linear.z = twist.linear[2]
        twist_msg.twist.angular.x = twist.angular[0]
        twist_msg.twist.angular.y = twist.angular[1]
        twist_msg.twist.angular.z = twist.angular[2]
        
        

        # Publish the twist
        if frame_id == "world":
            self.twist_pub4.publish(twist_msg)
        else:
            self.twist_pub3.publish(twist_msg)
            
    def publish_twist_with_action(self, twist, frame_id):
    
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = frame_id
        twist_msg.twist.linear.x = twist.linear[0]
        twist_msg.twist.linear.y = twist.linear[1]
        twist_msg.twist.linear.z = twist.linear[2]
        twist_msg.twist.angular.x = twist.angular[0]
        twist_msg.twist.angular.y = twist.angular[1]
        twist_msg.twist.angular.z = twist.angular[2]
        
        # Publish the twist
        if frame_id == "world":
            self.twist_pub6.publish(twist_msg)
        else:
            self.twist_pub5.publish(twist_msg)
            
            
    def publish_twist_with_action_invers(self, twist, frame_id):
    
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = frame_id
        twist_msg.twist.linear.x = twist.linear[0]
        twist_msg.twist.linear.y = twist.linear[1]
        twist_msg.twist.linear.z = twist.linear[2]
        twist_msg.twist.angular.x = twist.angular[0]
        twist_msg.twist.angular.y = twist.angular[1]
        twist_msg.twist.angular.z = twist.angular[2]
        
        # Publish the twist
        if frame_id == "world":
            self.twist_pub8.publish(twist_msg)
        else:
            self.twist_pub7.publish(twist_msg)
    


        
        
        

    def create_cube_transforms(self):
        """Creates 8 corner SE3 transforms of a cube centered at origin."""
        cube_size = 1.0  # 1 unit
        points = []

        # 8 corners of a cube (x, y, z): combinations of ±half
       
        translation = np.array([-0.5, -0.4, -0.4])
        T = pin.SE3(np.eye(3), translation)
        points.append(T)
        translation = np.array([0.5, -0.4, -0.4])
        rotation_z_90 = self.rotation_matrix_z(np.pi / 2)
        T = pin.SE3(rotation_z_90, translation)
        points.append(T)
        translation = np.array([0.5, 0.4, -0.4])
        rotation_z = self.rotation_matrix_z(np.pi)
        T = pin.SE3(rotation_z, translation)
        points.append(T)
        translation = np.array([-0.5, 0.4, -0.4])
        rotation_z = self.rotation_matrix_z(-np.pi/2)
        T = pin.SE3(rotation_z, translation)
        points.append(T)
        translation = np.array([-0.5, -0.4, 0.4])
        rotation_x = self.rotation_matrix_x(np.pi)
        rotation_z = self.rotation_matrix_z(-np.pi/2)
        T = pin.SE3(rotation_x @ rotation_z, translation)
        points.append(T)
        translation = np.array([0.5, -0.4, 0.4])
        rotation_x = self.rotation_matrix_x(np.pi)
        rotation_z = self.rotation_matrix_z(np.pi)
        T = pin.SE3(rotation_x @ rotation_z, translation)
        points.append(T)
        translation = np.array([0.5, 0.4, 0.4])
        rotation_x = self.rotation_matrix_x(np.pi)
        rotation_z = self.rotation_matrix_z(np.pi/2)
        T = pin.SE3(rotation_x @ rotation_z, translation)
        points.append(T)
        translation = np.array([-0.5, 0.4, 0.4])
        rotation_x = self.rotation_matrix_x(np.pi)
        T = pin.SE3(rotation_x, translation)
        points.append(T)
        
        return points
    
    def rotation_matrix_x(self, angle_rad):
        """Returns a rotation matrix for a rotation around the X-axis."""
        c, s = np.cos(angle_rad), np.sin(angle_rad)
        return np.array([
            [1,  0,  0],
            [0,  c, -s],
            [0,  s,  c]
        ])

    def rotation_matrix_y(self, angle_rad):
        """Returns a rotation matrix for a rotation around the Y-axis."""
        c, s = np.cos(angle_rad), np.sin(angle_rad)
        return np.array([
            [ c,  0,  s],
            [ 0,  1,  0],
            [-s,  0,  c]
        ])

    def rotation_matrix_z(self, angle_rad):
        """Returns a rotation matrix for a rotation around the Z-axis."""
        c, s = np.cos(angle_rad), np.sin(angle_rad)
        return np.array([
            [ c, -s,  0],
            [ s,  c,  0],
            [ 0,  0,  1]
        ])

    def twist_coordinate_transform(self, twist, transform):
        """
        Transforms a spatial twist using a given SE3 transform.
        :param twist: pinocchio.Motion (6D twist vector)
        :param transform: pinocchio.SE3 (SE3 transformation matrix)
        :return: Transformed pinocchio.Motion
        """
        return transform.act(twist)

    def broadcast_transforms(self):
        now = self.get_clock().now().to_msg()

        # Update poses using both methods
        #self.update_pose_quaternion(0.1)  # 0.5 seconds timestep
        self.update_pose_exp6(0.5)

        # Broadcast quaternion-based transform
        # center_transform_quat = TransformStamped()
        # center_transform_quat.header.stamp = now
        # center_transform_quat.header.frame_id = "world"
        # center_transform_quat.child_frame_id = "center"
        # center_transform_quat.transform.translation.x = self.translation_quat[0]
        # center_transform_quat.transform.translation.y = self.translation_quat[1]
        # center_transform_quat.transform.translation.z = self.translation_quat[2]
        # center_transform_quat.transform.rotation.x = self.rotation_quat.x
        # center_transform_quat.transform.rotation.y = self.rotation_quat.y
        # center_transform_quat.transform.rotation.z = self.rotation_quat.z
        # center_transform_quat.transform.rotation.w = self.rotation_quat.w
        #self.br.sendTransform(center_transform_quat)

        # Broadcast exp6-based transform
        center_transform_exp6 = TransformStamped()
        center_transform_exp6.header.stamp = now
        center_transform_exp6.header.frame_id = "world"
        center_transform_exp6.child_frame_id = "center"
        center_transform_exp6.transform.translation.x = self.pose_exp6.translation[0]
        center_transform_exp6.transform.translation.y = self.pose_exp6.translation[1]
        center_transform_exp6.transform.translation.z = self.pose_exp6.translation[2]
        q_exp6 = pin.Quaternion(self.pose_exp6.rotation)
        center_transform_exp6.transform.rotation.x = q_exp6.x
        center_transform_exp6.transform.rotation.y = q_exp6.y
        center_transform_exp6.transform.rotation.z = q_exp6.z
        center_transform_exp6.transform.rotation.w = q_exp6.w
        self.br.sendTransform(center_transform_exp6)

        # Broadcast cube corners relative to quaternion-based center
        for i, T in enumerate(self.transforms):
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = "center"
            t.child_frame_id = f"corner_{i+1}"

            t.transform.translation.x = T.translation[0]
            t.transform.translation.y = T.translation[1]
            t.transform.translation.z = T.translation[2]

            q = pin.Quaternion(T.rotation)
            t.transform.rotation.x = q.x
            t.transform.rotation.y = q.y
            t.transform.rotation.z = q.z
            t.transform.rotation.w = q.w

            self.br.sendTransform(t)
            
            
        ################
        ## Marker 
        ################
        
        corner_transform = self.transforms[3]
        transformed_point = corner_transform.act(self.p_corner)
        
        self.publish_p_corner(transformed_point, "corner_3")
        
        world_transform = self.pose_exp6  
        point_in_world = world_transform.act(transformed_point)
        
        self.publish_p_corner(point_in_world, "world")
        
        

        #################
        ## Twist 
        #t###############
        corner_transform = self.transforms[3]  
        twist_transform = self.spatial_twist
        transformed_twist = corner_transform.act(twist_transform)
       

        self.publish_twist(transformed_twist, "corner_3")
        
        # spatial twist with respect to world
        world_transform = self.pose_exp6
        transformed_twist_world = world_transform.act(twist_transform) #instead of transformed_twist
        
        self.publish_twist(transformed_twist_world, "world")
        
        #################
        ## Twist Inversion
        #t###############
        world_twist = pin.Motion(np.array([1.0, 0.0, 0.0, 0.5, 0.0, 0.0]))  
        corner_2_transform = self.transforms[2]  
        corner_2_twist = corner_2_transform.actInv(world_twist)
        corner_2_twist_world = world_transform.actInv(world_twist)
        
        self.publish_twist_with_color(corner_2_twist_world, "world")  
        self.publish_twist_with_color(corner_2_twist, "corner_2")  
        
        
        #############
        ## Adjoint
        #############
        adj = corner_transform.action 
        adjoint_twist = adj @ twist_transform.vector
        adjoint_twist = pin.Motion(adjoint_twist)
        self.publish_twist_with_action(adjoint_twist, "corner_3")
        
        
        adjoint_twist_world = world_transform.action @ twist_transform.vector
        adjoint_twist_world = pin.Motion(adjoint_twist_world)
        self.publish_twist_with_action(adjoint_twist_world, "world")
        
        ###########
        ## Adjoint Inversion
        ###########
        adjoint_twist_invers = np.linalg.inv(corner_2_transform.action) @ world_twist.vector
        adjoint_twist_invers = pin.Motion(adjoint_twist_invers)
        self.publish_twist_with_action_invers(adjoint_twist_invers, "corner_2")
        
        adjoint_twist_world_invers = np.linalg.inv(world_transform.action) @ world_twist.vector
        adjoint_twist_world_invers = pin.Motion(adjoint_twist_world_invers)
        self.publish_twist_with_action_invers(adjoint_twist_world_invers, "world")
        
        
        ######
        ##Answer to roation question
        ######
        
        # When rotating the cube around the world, the angular velocity vector will remain fixed in the world frame, while the liner velocity vector rotates with the cube.This happens because the linear velocity is tied to the cube's local frame, and as the frame rotates, the linear velocity vector rotates with it.
        
        
        
    def update_pose_quaternion(self, dt):
        # Update translation
        self.translation_quat += self.linear_velocity * dt

        # Update rotation
        delta_angle = self.angular_velocity * dt
        delta_quat = pin.Quaternion(pin.exp3(delta_angle))
        self.rotation_quat = self.rotation_quat * delta_quat
        self.rotation_quat.normalize()

    def update_pose_exp6(self, dt):
        
        # Create a motion vector (6D: angular + linear)
        motion_vector = np.hstack((self.angular_velocity, self.linear_velocity))
        delta_pose = pin.exp6(motion_vector * dt)
        self.pose_exp6 = self.pose_exp6 * delta_pose

def main(args=None):
    rclpy.init(args=args)
    node = CagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
