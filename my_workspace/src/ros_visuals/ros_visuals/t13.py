#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import pinocchio as pin
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import WrenchStamped


class CagePublisher(Node):
    def __init__(self):
        super().__init__('cage_publisher')
        self.br = TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.marker_pub2 = self.create_publisher(Marker, 'visualization_marker2', 10)
        self.wrench_pub = self.create_publisher(WrenchStamped, 'wrench', 10)
        self.wrench_pub2 = self.create_publisher(WrenchStamped, 'wrench2', 10)
        self.wrench_pub3 = self.create_publisher(WrenchStamped, 'wrench3', 10)
        self.wrench_pub4 = self.create_publisher(WrenchStamped, 'wrench4', 10)
        self.wrench_pub5 = self.create_publisher(WrenchStamped, 'wrench5', 10)
        self.wrench_pub6 = self.create_publisher(WrenchStamped, 'wrench6', 10)

    
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
        
        
        
    def publish_wrench(self, wrench, frame_id):
        
        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = self.get_clock().now().to_msg()
        wrench_msg.header.frame_id = frame_id
        wrench_msg.wrench.force.x = wrench.linear[0]
        wrench_msg.wrench.force.y = wrench.linear[1]
        wrench_msg.wrench.force.z = wrench.linear[2]
        wrench_msg.wrench.torque.x = wrench.angular[0]
        wrench_msg.wrench.torque.y = wrench.angular[1]
        wrench_msg.wrench.torque.z = wrench.angular[2]
        
        if frame_id == "world":
            self.wrench_pub2.publish(wrench_msg)
        else:
            self.wrench_pub.publish(wrench_msg)
            
    def publish_wrench_inv(self, wrench, frame_id):
        
        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = self.get_clock().now().to_msg()
        wrench_msg.header.frame_id = frame_id
        wrench_msg.wrench.force.x = wrench.linear[0]
        wrench_msg.wrench.force.y = wrench.linear[1]
        wrench_msg.wrench.force.z = wrench.linear[2]
        wrench_msg.wrench.torque.x = wrench.angular[0]
        wrench_msg.wrench.torque.y = wrench.angular[1]
        wrench_msg.wrench.torque.z = wrench.angular[2]
        
    
        self.wrench_pub3.publish(wrench_msg)
    
    def publish_wrench_adj(self, wrench, frame_id):
        
        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = self.get_clock().now().to_msg()
        wrench_msg.header.frame_id = frame_id
        wrench_msg.wrench.force.x = wrench.linear[0]
        wrench_msg.wrench.force.y = wrench.linear[1]
        wrench_msg.wrench.force.z = wrench.linear[2]
        wrench_msg.wrench.torque.x = wrench.angular[0]
        wrench_msg.wrench.torque.y = wrench.angular[1]
        wrench_msg.wrench.torque.z = wrench.angular[2]
        
    
        if frame_id == "world":
            self.wrench_pub5.publish(wrench_msg)
        else:
            self.wrench_pub4.publish(wrench_msg)

    def publish_wrench_adj_inv(self, wrench, frame_id):
        
        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = self.get_clock().now().to_msg()
        wrench_msg.header.frame_id = frame_id
        wrench_msg.wrench.force.x = wrench.linear[0]
        wrench_msg.wrench.force.y = wrench.linear[1]
        wrench_msg.wrench.force.z = wrench.linear[2]
        wrench_msg.wrench.torque.x = wrench.angular[0]
        wrench_msg.wrench.torque.y = wrench.angular[1]
        wrench_msg.wrench.torque.z = wrench.angular[2]
        self.wrench_pub6.publish(wrench_msg)
        

    def act_function_manual(self, transform, wrench):
        
        # Extract the rotation and translation from the SE3 transform
        R = transform.rotation
        p = transform.translation

        
        #transformed_wrench = pin.Force(R @ wrench.linear + np.cross(p, wrench.angular))
        
        p_skew = np.array([
            [0, -p[2], p[1]],
            [p[2], 0, -p[0]],
            [-p[1], p[0], 0]
        ])
        
        adjoint = np.block([
            [R, np.zeros((3, 3))],
            [p_skew @ R, R]
        ])
        
        transformed_wrench = adjoint @ wrench.vector
        return pin.Motion(transformed_wrench)
        

        

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
        
        
    def wrench_formula(self, r,f):
         
        r_skew = np.array([
            [0, -r[2], r[1]],
            [r[2], 0, -r[0]],
            [-r[1], r[0], 0]
        ])
        moment = r_skew @ f
        force = f
        wrench = np.hstack((moment, force))
        return wrench

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
        ##Marker
        ################
            
        corner_transform = self.transforms[3]
        transformed_point = corner_transform.act(self.p_corner)
        
        self.publish_p_corner(transformed_point, "corner_3")
        
        world_transform = self.pose_exp6 
        point_in_world = world_transform.act(transformed_point)
        
        self.publish_p_corner(point_in_world, "world")
        
        
        ################
        ##Wrench
        ################
        
        # corner_transform = self.transforms[1] 
        # spatial_wrench = pin.Force(np.array([1.0, 0.5, 0.2, 0.3, 0.4, 0.6]))
        # transformed_wrench = corner_transform.act(spatial_wrench) 
        # self.publish_wrench(transformed_wrench, "corner_1")
        
        
        ##or
        
        #wrench to corner
        corner_transform1 = self.transforms[1]
        r = np.array([0.5, 0.5, 0.5])
        f = np.array([1.0, 0.5, 0.2])
        wrench = self.wrench_formula(r, f)
        wrench = pin.Force(wrench)
        transformed_wrench = self.act_function_manual(corner_transform1, wrench)
        #transformed_wrench = corner_transform1.act(wrench)
        #print("Using act:", transformed_wrench)
        self.publish_wrench(transformed_wrench, "corner_1")
        
        #corner to world
        world_transform = self.pose_exp6
        point_in_world = world_transform.act(transformed_wrench)
        self.publish_wrench(point_in_world, "world")
        
        
        ############
        ## Wrench (world to corner)
        ############
        
        world_wrench = pin.Force(np.array([0.5, 0.5, 0.5, 0.3, 0.4, 0.6]))
        corner_transform2= self.transforms[2]
        corner_transform_inv2 = corner_transform2.actInv(world_wrench)
        self.publish_wrench_inv(corner_transform_inv2, "corner_2")
        
        
        ################
        ##Wrench (action)
        ###############
        #wrench to corner
        corner_transform1 = self.transforms[1]
        adj = corner_transform1.action
        adjoint_wrench = adj @ wrench.vector
        adjoint_wrench = pin.Force(adjoint_wrench)
        self.publish_wrench_adj(adjoint_wrench, "corner_1")
        #print("Using action:", adjoint_wrench)
        
        #wrench to world
        world_transform = self.pose_exp6
        adjoint_wrench_world = world_transform.action @ adjoint_wrench.vector
        point_in_world = pin.Force(adjoint_wrench_world)
        self.publish_wrench_adj(point_in_world, "world")
        
        
        # world to corner
        adjoint_wrench_invers = np.linalg.inv(corner_transform2.action) @ world_wrench.vector
        adjoint_wrench_invers_motion = pin.Force(adjoint_wrench_invers)
        self.publish_wrench_adj_inv(adjoint_wrench_invers_motion, "corner_2")
        
        
        
        
        
        ######
        ##Answer
        ######
        
        #When the cage spins, the force component of the wrench stays constant because it is independent of the frame's rotation. The torque component changes because it depends on the relative position vector, which varies with the rotation of the cage.

        
        

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
