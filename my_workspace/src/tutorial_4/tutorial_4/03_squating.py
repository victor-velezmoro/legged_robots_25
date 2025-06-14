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
        initial_base_pos = np.array([0.0, 0.0, 1.1]) # Default from conf.q_home
        initial_base_quat = np.array([0.0, 0.0, 0.0, 1.0]) # Default from conf.q_home (x,y,z,w)

        # if q is not None and len(q) >= 7:
        #     initial_base_pos = q[0:3]
        #     # PyBullet uses [x,y,z,w] for quaternions, Pinocchio q[3:7] is [qx,qy,qz,qw]
        #     initial_base_quat = q[3:7] 

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
    simulator = PybulletWrapper(sim_rate=1000)
    
    # Instantiate Talos robot with floating base
    robot_node = Talos(
        simulator=simulator,
        urdf=conf.urdf, # URDF path from config
        model=tsid_controller.model, # Pinocchio model from TSIDWrapper
        q=conf.q_home # Initial full configuration from config
    )
    
     # Data storage for plotting
    time_log = []
    com_ref_log = []
    com_tsid_log = []
    com_sim_log = []
    v_com_ref_log = []
    v_com_tsid_log = []
    v_com_sim_log = []
    a_com_ref_log = []
    a_com_tsid_log = []
    a_com_sim_log = []

    prev_v_com_sim = None
    prev_time = None
    
    # After robot creation, add:
    robot_node.get_logger().info(f"Initial robot mass: {robot_node.pin_model.inertias[1].mass}")  # Check base mass
    robot_node.get_logger().info(f"Robot total mass estimate: {sum([inertia.mass for inertia in robot_node.pin_model.inertias])}")
    
    # Not sure if this is needed, but let's set it
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
            time_buffer = 0.0
            duration = 2.0 + time_buffer  # Duration for the TSID controller
            squat_start_time = 4.0 + time_buffer  # Start squat motion after 4 seconds
            circle_motion_start_time = 8.0 + time_buffer  # Start circle motion after 8 seconds
            amplitude = 0.01  # Amplitude of the squat motion
            frequency = 0.5  # Frequency of the squat motion
            
            q_pin_current = robot_node.q()
            v_pin_current = robot_node.v()

            
            # Simulator (PyBullet) CoM
            com_sim = robot_node.baseWorldPosition()
            v_com_sim = robot_node.baseWorldLinearVeloctiy()
            com_sim_log.append(np.array(com_sim))
            v_com_sim_log.append(np.array(v_com_sim))
            # Estimate acceleration numerically
            if prev_v_com_sim is not None and prev_time is not None:
                a_sim = (np.array(v_com_sim) - np.array(prev_v_com_sim)) / (current_sim_time - prev_time)
            else:
                a_sim = np.zeros(3)
            a_com_sim_log.append(a_sim)
            prev_v_com_sim = v_com_sim
            prev_time = current_sim_time
            
            
            # Get current COM and Right foot positions
            p_com = tsid_controller.comState().pos()
            p_RF = tsid_controller.get_placement_RF().translation

            #Only update CoM reference for the first 2 seconds
            if current_sim_time > time_buffer and current_sim_time < duration:
                # Set COM reference to XY position of right foot, keep current COM height
                p_ref = np.array([p_RF[0], p_RF[1], p_com[2]])
                tsid_controller.setComRefState(p_ref)
            if current_sim_time >= duration:
                # After 2 seconds, remove left foot contact and lift it
                # Get current left foot pose
                LF_pose = tsid_controller.get_placement_LF()
                
                # Set new reference 0.3 meters above current position
                goal_pose = pin.SE3(LF_pose.rotation, LF_pose.translation + np.array([0.0, 0.0, 0.3]))
                tsid_controller.set_LF_pose_ref(goal_pose)
                
                # Remove left foot contact 
                if current_sim_time >= duration and current_sim_time < duration + 0.1:  # Small window to avoid repeated calls
                    tsid_controller.remove_contact_LF()
                    robot_node.get_logger().info("Removed left foot contact and set new reference")

            if current_sim_time >= squat_start_time:
                robot_node.get_logger().info("sim time: " + str(current_sim_time))
                
                t_squat = current_sim_time - squat_start_time
                squat_z = amplitude * np.sin(2 * np.pi * frequency * t_squat)
                squat_dz = amplitude * 2 * np.pi * frequency * np.cos(2 * np.pi * frequency * t_squat)
                squat_ddz = -amplitude * (2 * np.pi * frequency) ** 2 * np.sin(2 * np.pi * frequency * t_squat)

                p_com_ref = np.array([p_com[0], p_com[1], conf.q_home[2] + squat_z]) #maybe home is not the best choice here
                v_com_ref = np.array([0.0, 0.0, squat_dz])
                a_com_ref = np.array([0.0, 0.0, squat_ddz])

                tsid_controller.setComRefState(p_com_ref, v_com_ref, a_com_ref)

            if current_sim_time >= circle_motion_start_time:
    
                t_circle = current_sim_time - circle_motion_start_time
                radius = 0.2
                omega = 2 * np.pi * 0.1  # 0.1 Hz
                center = np.array([0.4, -0.2, 1.1])
                # p = c + [0, r cos(ωt), r sin(ωt)]
                p_circle_ref = center + np.array([0.0, radius * np.cos(omega * t_circle), radius * np.sin(omega * t_circle)])

                simulator.addGlobalDebugTrajectory(
                    p_circle_ref,
                    np.array([0.0, 0.0, 1.0]),
                    0.1,
                    color=[1.0, 0.0, 0.0, 1.0]
                )

                # Activate right hand task only once
                if not rh_motion_activated:
                    # Set orientation gains to zero
                    tsid_controller.rightHandTask.setKp(100 * np.array([1, 1, 1, 0, 0, 0]))
                    tsid_controller.rightHandTask.setKd(2.0 * np.sqrt(100) * np.array([1, 1, 1, 0, 0, 0]))
                    tsid_controller.add_motion_RH()  # Activate the right hand task
                    rh_motion_activated = True

                tsid_controller.set_RH_pos_ref(p_circle_ref)

            tau_sol, dv_sol = tsid_controller.update(q_pin_current, v_pin_current, current_sim_time)
            robot_node.setActuatedJointTorques(tau_sol)

            # robot_node.get_logger().info(f"Current CoM position: {p_com}")
            # robot_node.get_logger().info(f"Current Right Foot position: {p_RF}")
            # robot_node.get_logger().info(f"Set CoM reference to: {p_ref}")
            # robot_node.get_logger().info(f"Current ref CoM position: {tsid_controller.comReference().pos()}")
            # robot_node.get_logger().info(f"Current ref CoM velocity: {tsid_controller.comReference().vel()}")
            # robot_node.get_logger().info(f"Current ref CoM acceleration: {tsid_controller.comReference().acc()}")


            # Add logging to see if TSID is computing reasonable torques
            robot_node.get_logger().debug(f"Computed torques: {tau_sol[:6]}")  # First 6 joint torques
            robot_node.get_logger().debug(f"Max torque magnitude: {np.max(np.abs(tau_sol))}")
            robot_node.get_logger().debug(f"Joint velocities: {v_pin_current[7:]}")

            # Command to the robot
            robot_node.setActuatedJointTorques(tau_sol)
            
            # TSID reference and state
            time_log.append(current_sim_time)
            com_ref = tsid_controller.comReference()
            com_state = tsid_controller.comState()
            com_ref_log.append(com_ref.pos().copy())
            #print(f"Current CoM ref log: {com_ref_log[-1]}")
            v_com_ref_log.append(com_ref.vel().copy())
            a_com_ref_log.append(com_ref.acc().copy())
            com_tsid_log.append(com_state.pos().copy())
            v_com_tsid_log.append(com_state.vel().copy())
            a_com_tsid_log.append(np.zeros(3))


            # Debug info every second
            if current_sim_time % 1.0 < 0.001:
                joint_error = np.linalg.norm(q_pin_current[7:] - conf.q_actuated_home)
                robot_node.get_logger().debug(f"Joint error to home: {joint_error:.4f}")
                base_height = q_pin_current[2]
                robot_node.get_logger().debug(f"Base height: {base_height:.3f}m")

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
        
        # --- Plotting ---
        def ensure_2d(arr):
            arr = np.array(arr)
            if arr.ndim == 1:
                arr = arr.reshape(-1, 3)
            return arr

        com_ref_log = ensure_2d(com_ref_log)
        com_tsid_log = ensure_2d(com_tsid_log)
        com_sim_log = ensure_2d(com_sim_log)
        v_com_ref_log = ensure_2d(v_com_ref_log)
        v_com_tsid_log = ensure_2d(v_com_tsid_log)
        v_com_sim_log = ensure_2d(v_com_sim_log)
        a_com_ref_log = ensure_2d(a_com_ref_log)
        a_com_tsid_log = ensure_2d(a_com_tsid_log)
        a_com_sim_log = ensure_2d(a_com_sim_log)
        
        min_len = min(len(time_log), len(com_ref_log), len(com_tsid_log), len(com_sim_log),
                    len(v_com_ref_log), len(v_com_tsid_log), len(v_com_sim_log),
                    len(a_com_ref_log), len(a_com_tsid_log), len(a_com_sim_log))
        time_log = time_log[:min_len]
        com_ref_log = com_ref_log[:min_len]
        com_tsid_log = com_tsid_log[:min_len]
        com_sim_log = com_sim_log[:min_len]
        v_com_ref_log = v_com_ref_log[:min_len]
        v_com_tsid_log = v_com_tsid_log[:min_len]
        v_com_sim_log = v_com_sim_log[:min_len]
        a_com_ref_log = a_com_ref_log[:min_len]
        a_com_tsid_log = a_com_tsid_log[:min_len]
        a_com_sim_log = a_com_sim_log[:min_len]

        fig, axs = plt.subplots(3, 3, figsize=(15, 10), sharex=True)
        labels = ['x', 'y', 'z']

        # Position
        for i in range(3):
            axs[0, i].plot(time_log, com_ref_log[:, i], label='CoM ref')
            axs[0, i].plot(time_log, com_tsid_log[:, i], label='CoM TSID')
            axs[0, i].plot(time_log, com_sim_log[:, i], label='CoM sim')
            axs[0, i].set_ylabel(f'Pos {labels[i]} [m]')
            axs[0, i].legend()
            axs[0, i].grid()

        # Velocity
        for i in range(3):
            axs[1, i].plot(time_log, v_com_ref_log[:, i], label='CoM ref')
            axs[1, i].plot(time_log, v_com_tsid_log[:, i], label='CoM TSID')
            axs[1, i].plot(time_log, v_com_sim_log[:, i], label='CoM sim')
            axs[1, i].set_ylabel(f'Vel {labels[i]} [m/s]')
            axs[1, i].legend()
            axs[1, i].grid()

        # Acceleration
        for i in range(3):
            axs[2, i].plot(time_log, a_com_ref_log[:, i], label='CoM ref')
            axs[2, i].plot(time_log, a_com_tsid_log[:, i], label='CoM TSID')
            axs[2, i].plot(time_log, a_com_sim_log[:, i], label='CoM sim')
            axs[2, i].set_ylabel(f'Acc {labels[i]} [m/s²]')
            axs[2, i].set_xlabel('Time [s]')
            axs[2, i].legend()
            axs[2, i].grid()

        plt.tight_layout()
        plt.savefig('com_analysis.png', dpi=300)
        plt.show()
    
if __name__ == '__main__': 
    main()
