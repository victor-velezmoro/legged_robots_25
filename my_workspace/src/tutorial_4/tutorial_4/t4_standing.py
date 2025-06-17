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
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper



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
        initial_base_pos = np.array([0.0, 0.0, 1.1]) # Default from conf.q_home
        initial_base_quat = np.array([0.0, 0.0, 0.0, 1.0]) # Default from conf.q_home (x,y,z,w)

        # if q is not None and len(q) >= 7:
        #     initial_base_pos = q[0:3]
        #     # PyBullet uses [x,y,z,w] for quaternions, Pinocchio q[3:7] is [qx,qy,qz,qw]
        #     initial_base_quat = q[3:7] 
        print(f"Initial base position: {initial_base_pos}")
        print(f"Initial base orientation (quat): {initial_base_quat}")

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
        
        self.pin_model = model
        self.pin_data = self.pin_model.createData()
        
        # ROS Publishers and Broadcasters
        self.joint_state_publisher = self.create_publisher(
            JointState, 
            'joint_states', 
            10)
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
    

    def update(self):
        # Update base class (Robot)
        super().update()
        # pin.forwardKinematics(self.pin_model, self.pin_data, self.q())
        # pin.updateFramePlacements(self.pin_model, self.pin_data)
    
    def publish(self):

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
    q_init = np.zeros(30)
    q_init = np.hstack([np.array([0, 0, 1.1, 0, 0, 0, 1]), q_init])
    
    # Instantiate Robot (Talos node)
    robot_node = Talos(
        simulator=simulator,
        urdf=conf.urdf, 
        model=tsid_controller.model,
        q=q_init 
    )
    

    
    # Set the posture reference to home position in TSID controller
    # This tells TSID to drive the robot joints to the home configuration
    #tsid_controller.setPostureRef(conf.q_actuated_home)
    robot_node.get_logger().debug("Set posture reference to home position")
    robot_node.get_logger().debug(f"Initial base position: {robot_node.q()[:3]}")
    robot_node.get_logger().debug(f"Initial base orientation: {robot_node.q()[3:7]}")


    t_publish = 0.0 # For controlling publish rate
    start_com_shift = 1.0
    time_to_lift_foot = 3.0
    squat_start_time = 5.0
    circle_motion_start_time = 8.0
    rh_motion_activated = False

    try:
        while rclpy.ok():
            # Process ROS events (e.g., for subscribers or timers if any)
            # rclpy.spin_once(robot_node, timeout_sec=0.00001)

            # Elapsed time
            current_sim_time = simulator.simTime()

            # Update the simulator and the robot
            simulator.step()
            simulator.debug()
            robot_node.update() # This updates robot.q() and robot.v() from simulator
            
            # Update TSID controller
            amplitude = 0.05  # Amplitude of the squat motion
            frequency = 0.5  # Frequency of the squat motion
            
            # Update TSID controller
            q_pin_current = robot_node.q()
            v_pin_current = robot_node.v()
            
            p_com = tsid_controller.comState().pos()
            p_RF = tsid_controller.get_placement_RF().translation
            
            if current_sim_time >= start_com_shift:
                p_ref = np.array([p_RF[0], p_RF[1], p_com[2]])
                tsid_controller.setComRefState(p_ref)
                robot_node.get_logger().info(f"Set CoM reference to: {p_ref}")
            if current_sim_time >= start_com_shift:
                LF_pose = tsid_controller.get_placement_LF()
                goal_pose = pin.SE3(LF_pose.rotation, LF_pose.translation + np.array([0.0, 0.0, 0.3]))
                tsid_controller.set_LF_pose_ref(goal_pose)
                if current_sim_time >= time_to_lift_foot and current_sim_time < time_to_lift_foot + 0.1:
                    tsid_controller.remove_contact_LF()
                    robot_node.get_logger().info("Removed left foot contact and set new reference")
                    
            if current_sim_time >= squat_start_time:
                
                t_squat = current_sim_time - squat_start_time
                squat_z = amplitude * np.sin(2 * np.pi * frequency * t_squat)
                squat_dz = amplitude * 2 * np.pi * frequency * np.cos(2 * np.pi * frequency * t_squat)
                squat_ddz = -amplitude * (2 * np.pi * frequency) ** 2 * np.sin(2 * np.pi * frequency * t_squat)

                p_com_ref = np.array([p_com[0], p_com[1], p_com[2] + squat_z]) #maybe home is not the best choice here
                v_com_ref = np.array([0.0, 0.0, squat_dz])
                a_com_ref = np.array([0.0, 0.0, squat_ddz])

                tsid_controller.setComRefState(p_com_ref, v_com_ref, a_com_ref)
                
            # if current_sim_time >= circle_motion_start_time:
    
            #     t_circle = current_sim_time - circle_motion_start_time
            #     radius = 0.2
            #     omega = 2 * np.pi * 0.1  # 0.1 Hz
            #     center = np.array([0.4, -0.2, 1.1])
            #     # p = c + [0, r cos(ωt), r sin(ωt)]
            #     p_circle_ref = center + np.array([0.0, radius * np.cos(omega * t_circle), radius * np.sin(omega * t_circle)])

            #     simulator.addGlobalDebugTrajectory(
            #         p_circle_ref,
            #         np.array([0.0, 0.0, 1.0]),
            #         0.1,
            #         color=[1.0, 0.0, 0.0, 1.0]
            #     )
            #     # Activate right hand task only once
            #     if not rh_motion_activated:
            #         # Set orientation gains to zero
            #         tsid_controller.rightHandTask.setKp(100 * np.array([1, 1, 1, 0, 0, 0]))
            #         tsid_controller.rightHandTask.setKd(2.0 * np.sqrt(100) * np.array([1, 1, 1, 0, 0, 0]))
            #         tsid_controller.add_motion_RH()  # Activate the right hand task
            #         rh_motion_activated = True

            #     tsid_controller.set_RH_pos_ref(p_circle_ref, 
            #                                     np.array([0.0, 0.0, 0.0, 1.0]), 
            #                                     np.array([0.0, 0.0, 0.0, 1.0]))
                
                
            

            tau_sol, dv_sol = tsid_controller.update(q_pin_current, v_pin_current, current_sim_time)
            robot_node.setActuatedJointTorques(tau_sol)

            # Debug info every second
            joint_error = np.linalg.norm(q_pin_current[7:] - conf.q_actuated_home) # Compare with conf.q_actuated_home
            robot_node.get_logger().info(f"Joint error to home: {joint_error:.4f}")
            base_height = q_pin_current[2]
            robot_node.get_logger().debug(f"Base height: {base_height:.3f}m")
            robot_node.get_logger().info(f"Current simulation time: {current_sim_time:.3f}s")

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

