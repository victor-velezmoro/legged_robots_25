#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import pinocchio as pin
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class CagePublisher(Node):
    def __init__(self):
        super().__init__('cage_publisher')
        self.br = TransformBroadcaster(self)

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

        # Initialize pose for exp6-based integration
        self.pose_exp6 = pin.SE3.Identity()

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

    def broadcast_transforms(self):
        now = self.get_clock().now().to_msg()

        # Update poses using both methods
        self.update_pose_quaternion(0.5)  # 0.5 seconds timestep
        #self.update_pose_exp6(0.5)

        # Broadcast quaternion-based transform
        center_transform_quat = TransformStamped()
        center_transform_quat.header.stamp = now
        center_transform_quat.header.frame_id = "world"
        center_transform_quat.child_frame_id = "center"
        center_transform_quat.transform.translation.x = self.translation_quat[0]
        center_transform_quat.transform.translation.y = self.translation_quat[1]
        center_transform_quat.transform.translation.z = self.translation_quat[2]
        center_transform_quat.transform.rotation.x = self.rotation_quat.x
        center_transform_quat.transform.rotation.y = self.rotation_quat.y
        center_transform_quat.transform.rotation.z = self.rotation_quat.z
        center_transform_quat.transform.rotation.w = self.rotation_quat.w
        self.br.sendTransform(center_transform_quat)

        # # Broadcast exp6-based transform
        # center_transform_exp6 = TransformStamped()
        # center_transform_exp6.header.stamp = now
        # center_transform_exp6.header.frame_id = "world"
        # center_transform_exp6.child_frame_id = "center_exp6"
        # center_transform_exp6.transform.translation.x = self.pose_exp6.translation[0]
        # center_transform_exp6.transform.translation.y = self.pose_exp6.translation[1]
        # center_transform_exp6.transform.translation.z = self.pose_exp6.translation[2]
        # q_exp6 = pin.Quaternion(self.pose_exp6.rotation)
        # center_transform_exp6.transform.rotation.x = q_exp6.x
        # center_transform_exp6.transform.rotation.y = q_exp6.y
        # center_transform_exp6.transform.rotation.z = q_exp6.z
        # center_transform_exp6.transform.rotation.w = q_exp6.w
        # self.br.sendTransform(center_transform_exp6)

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

    def update_pose_quaternion(self, dt):
        """Update pose using quaternion-based integration."""
        # Update translation
        self.translation_quat += self.linear_velocity * dt

        # Update rotation
        delta_angle = self.angular_velocity * dt
        delta_quat = pin.Quaternion(pin.exp3(delta_angle))
        self.rotation_quat = self.rotation_quat * delta_quat
        self.rotation_quat.normalize()

    def update_pose_exp6(self, dt):
        """Update pose using exp6-based integration."""
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
