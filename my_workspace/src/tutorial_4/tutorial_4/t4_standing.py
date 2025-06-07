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
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

# ROS
import rclpy
from rclpy.node import Node
import tf2_ros
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Header

################################################################################
# settings
################################################################################

DO_PLOT = True

################################################################################
# Robot
################################################################################

class Talos(Robot, Node):
    def __init__(self, simulator, urdf, model, q=None, verbose=True, use_fixed_base=True):
        # Initialize as an rclpy Node
        Node.__init__(self, 'talos_controller_node')

        # Determine initial base pose from q or use defaults from conf.q_home if q is similar
        initial_base_pos = np.array([0.0, 0.0, 0.9]) # Default from conf.q_home
        initial_base_quat = np.array([0.0, 0.0, 0.0, 1.0]) # Default from conf.q_home (x,y,z,w)

        if q is not None and len(q) >= 7:
            initial_base_pos = q[0:3]
            # PyBullet uses [x,y,z,w] for quaternions, Pinocchio q[3:7] is [qx,qy,qz,qw]
            initial_base_quat = q[3:7] 

        # Call Robot base class constructor
        Robot.__init__(self,
                       simulator,
                       filename=urdf,
                       model=model,
                       basePosition=initial_base_pos,        
                       baseQuationerion=initial_base_quat,   
                       q=q,
                       useFixedBase=False,
                       verbose=verbose)
        
        # ROS Publishers and Broadcasters
        self.joint_state_publisher = self.create_publisher(
            JointState, 
            'joint_states', 
            10)
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
    

    def update(self):
        # Update base class (Robot)
        super().update()
    
    def publish(self):
        # Get current full configuration [base_pose_SE3, joint_angles] and velocity
        current_q_pin = self.q() 
        
        now = self.get_clock().now().to_msg()
        
        # Publish JointState with proper joint ordering
        joint_state_msg = JointState()
        joint_state_msg.header = Header()
        joint_state_msg.header.stamp = now
        
        # Get joint names and ensure they match the expected order
        joint_names = self.actuatedJointNames()
        joint_positions = self.actuatedJointPosition()
        joint_velocities = self.actuatedJointVelocity()
        
        # Ensure we have valid torque commands
        if hasattr(self, '_tau_cmd') and self._tau_cmd is not None:
            joint_efforts = self._tau_cmd
        else:
            joint_efforts = np.zeros(len(joint_names))
        
        joint_state_msg.name = joint_names
        joint_state_msg.position = joint_positions.tolist()
        joint_state_msg.velocity = joint_velocities.tolist()
        joint_state_msg.effort = joint_efforts.tolist()
            
        self.joint_state_publisher.publish(joint_state_msg)
        
        # Broadcast TF for base_link to world
        # Base pose from Pinocchio q (first 7 elements: x,y,z, qx,qy,qz,qw)
        base_translation = current_q_pin[0:3]
        base_orientation_quat = current_q_pin[3:7] # Pinocchio uses [qx, qy, qz, qw]

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "world"      # Parent frame
        t.child_frame_id = self.baseName() # Child frame (robot's base link name)

        t.transform.translation.x = float(base_translation[0])
        t.transform.translation.y = float(base_translation[1])
        t.transform.translation.z = float(base_translation[2])

        t.transform.rotation.x = float(base_orientation_quat[0])
        t.transform.rotation.y = float(base_orientation_quat[1])
        t.transform.rotation.z = float(base_orientation_quat[2])
        t.transform.rotation.w = float(base_orientation_quat[3])
        
        self.tf_broadcaster.sendTransform(t)

################################################################################
# main
################################################################################

def main(): 
    rclpy.init()

    # Instantiate TSIDWrapper
    tsid_controller = TSIDWrapper(conf)
    
    # Instantiate Simulator
    simulator = PybulletWrapper()
    
    # Add ground plane and set physics for contact
    # pb.setGravity(0, 0, -9.81)
    # ground_id = pb.loadURDF("plane.urdf", [0, 0, 0])
    
    # Instantiate Robot (Talos node)
    robot_node = Talos(
        simulator=simulator,
        urdf=conf.urdf, # URDF path from config
        model=tsid_controller.model, # Pinocchio model from TSIDWrapper
        q=conf.q_home # Initial full configuration from config
    )
    
    # Set the posture reference to home position in TSID controller
    # This tells TSID to drive the robot joints to the home configuration
    tsid_controller.setPostureRef(conf.q_actuated_home)
    robot_node.get_logger().info("Set posture reference to home position")
    
    t_publish = 0.0 # For controlling publish rate

    try:
        while rclpy.ok():
            # Process ROS events (e.g., for subscribers or timers if any)
            rclpy.spin_once(robot_node, timeout_sec=0.00001)

            # Elapsed time
            current_sim_time = simulator.simTime()

            # Update the simulator and the robot
            simulator.step()
            robot_node.update() # This updates robot.q() and robot.v() from simulator
            
            # Update TSID controller
            q_pin_current = robot_node.q()
            v_pin_current = robot_node.v()
            
            # tsid_controller.update returns (tau, dv)
            tau_sol, dv_sol = tsid_controller.update(q_pin_current, v_pin_current, current_sim_time)
            
        
            # Command to the robot
            robot_node.setActuatedJointTorques(tau_sol)

            # Debug info every second
            if current_sim_time % 1.0 < 0.001:
                joint_error = np.linalg.norm(q_pin_current[7:] - conf.q_actuated_home)
                robot_node.get_logger().info(f"Joint error to home: {joint_error:.4f}")
                base_height = q_pin_current[2]
                robot_node.get_logger().info(f"Base height: {base_height:.3f}m")

            # Publish to ROS at a controlled rate (e.g., 30 Hz)
            if current_sim_time - t_publish >= (1.0 / 30.0):
                t_publish = current_sim_time
                robot_node.publish() 
    
    except KeyboardInterrupt:
        robot_node.get_logger().info("Keyboard interrupt received, shutting down.")
    finally:
        # Cleanly shutdown ROS
        robot_node.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__': 
    main()
    
