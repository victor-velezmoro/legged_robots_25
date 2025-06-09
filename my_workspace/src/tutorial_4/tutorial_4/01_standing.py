import numpy as np
from numpy import nan
from numpy.linalg import norm as norm
import matplotlib.pyplot as plt

# pinocchio
import pinocchio as pin

# simulator
import pybullet as pb
from simulator.pybullet_wrapper import PybulletWrapper
from simulator.robot import Robot

# robot and controller
from tutorial_4.tsid_wrapper import TSIDWrapper
import tutorial_4.config as conf

# ROS
import rclpy
from rclpy.node import Node
import tf2_ros
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped

################################################################################
# settings
################################################################################

DO_PLOT = True

################################################################################
# Robot
################################################################################

class Talos(Robot):
    def __init__(self, simulator, urdf, model, q=None, verbose=True, useFixedBase=True):
        # Call base class constructor with floating base
        super().__init__(
            simulator,
            filename=urdf,
            model=model,
            q=q,
            useFixedBase=useFixedBase,
            verbose=verbose
        )
        # ROS publisher for joint states
        self.joint_pub = rclpy.node.Node('talos_joint_state_publisher').create_publisher(
            JointState, 'joint_states', 10)
        
        # TF broadcaster for base_link -> world
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(rclpy.node.Node('talos_tf_broadcaster'))

    def update(self):
        # Update base class (updates state from pybullet)
        super().update()

    def publish(self, T_b_w):
        # Publish joint states
        msg = JointState()
        msg.header.stamp = rclpy.clock.Clock().now().to_msg()
        msg.name = self.actuatedJointNames()
        msg.position = self.actuatedJointPosition().tolist()
        msg.velocity = self.actuatedJointVelocity().tolist()
        self.joint_pub.publish(msg)

        # Broadcast transformation T_b_w (Pinocchio SE3)
        tf_msg = tf2_ros.TransformStamped()
        tf_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
        tf_msg.header.frame_id = "world"
        tf_msg.child_frame_id = "base_link"
        tf_msg.transform.translation.x = float(T_b_w.translation[0])
        tf_msg.transform.translation.y = float(T_b_w.translation[1])
        tf_msg.transform.translation.z = float(T_b_w.translation[2])
        # Convert rotation matrix to quaternion
        from scipy.spatial.transform import Rotation as R
        quat = R.from_matrix(T_b_w.rotation).as_quat()
        tf_msg.transform.rotation.x = float(quat[0])
        tf_msg.transform.rotation.y = float(quat[1])
        tf_msg.transform.rotation.z = float(quat[2])
        tf_msg.transform.rotation.w = float(quat[3])
        self.tf_broadcaster.sendTransform(tf_msg)

################################################################################
# main
################################################################################

def main(): 
    rclpy.init()
    node = rclpy.create_node('tutorial_4_standing_node')

    # Instantiate TSIDWrapper
    tsid = TSIDWrapper(conf)

    # Instantiate Simulator
    simulator = PybulletWrapper()

    # Instantiate Talos robot with floating base
    robot = Talos(
        simulator=simulator,
        urdf=conf.urdf,
        model=tsid.robot.model(),
        q=conf.q_home,
        verbose=True,
        useFixedBase=False
    )

    t_publish = 0.0

    while rclpy.ok():
        # elapsed time
        t = simulator.simTime()

        # Update simulator and robot
        simulator.step()
        robot.update()

        # Update TSID controller
        q = robot.q()
        v = robot.v()
        tau_sol, dv_sol = tsid.update(q, v, t)

        # Command to the robot
        robot.setActuatedJointTorques(tau_sol)

        # Publish to ROS at 30Hz
        if t - t_publish > 1./30.:
            t_publish = t
            # Get current BASE Pose from TSID
            T_b_w, _ = tsid.baseState()
            robot.publish(T_b_w)

if __name__ == '__main__': 
    main()
