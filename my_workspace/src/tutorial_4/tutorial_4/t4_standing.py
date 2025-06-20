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
import time



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


    robot_node.get_logger().debug("Set posture reference to home position")
    robot_node.get_logger().debug(f"Initial base position: {robot_node.q()[:3]}")
    robot_node.get_logger().debug(f"Initial base orientation: {robot_node.q()[3:7]}")

    t_publish = 0.0 # For controlling publish rate
    start_com_shift = 1.0
    time_to_lift_foot = 3.0
    squat_start_time = 6.0
    circle_motion_start_time = 8.0
    rh_motion_activated = False
    
    # Circle parameters
    c = np.array([0.4, -0.2, 1.1])  # center
    r = 0.2  # radius
    f = 0.1  # frequency (Hz)
    omega = 2 * np.pi * f  # angular frequency
    
    # Generate trajectory points for visualization (one full circle)
    t_viz = np.linspace(0, 1/f, 100)  # 100 points over one period
    X_traj = c[0] + np.zeros_like(t_viz)  # x stays constant (Y-Z plane)
    Y_traj = c[1] + r * np.cos(omega * t_viz)
    Z_traj = c[2] + r * np.sin(omega * t_viz)
    
    # Add trajectory visualization
    simulator.addGlobalDebugTrajectory(X_traj, Y_traj, Z_traj)
    
    # Set right hand task gains
    tsid_controller.rightHandTask.setKp(100*np.array([1,1,1,0,0,0]))
    tsid_controller.rightHandTask.setKd(2.0*np.sqrt(100)*np.array([1,1,1,0,0,0]))
    
    start_time = time.time()


    try:
        while rclpy.ok():
            # Process ROS events (e.g., for subscribers or timers if any)
            # rclpy.spin_once(robot_node, timeout_sec=0.00001)

            # Elapsed time
            current_sim_time = simulator.simTime()
            
            elapsed_time = current_sim_time - start_time

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
            
            
            #  Simulator (PyBullet) CoM
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
                
            if current_sim_time >= circle_motion_start_time and not tsid_controller.motion_RH_active:
                # Activate right hand motion task
                tsid_controller.add_motion_RH()
                print("Right hand motion task activated!")
            
            if current_sim_time >= circle_motion_start_time:  # After 6 seconds
                # Calculate current position on circle
                t = current_sim_time - circle_motion_start_time  # time since circle started
                print(f"Current time: {current_sim_time:.2f}s, t: {t:.2f}s")
                pos = np.array([
                    c[0],  # x constant (Y-Z plane)
                    c[1] + r * np.cos(omega * t),
                    c[2] + r * np.sin(omega * t)
                ])
                
                # Calculate velocity for smooth motion
                vel = np.array([
                    0.0,  # x velocity is zero
                    -r * omega * np.sin(omega * t),
                    r * omega * np.cos(omega * t)
                ])
                
                # Calculate acceleration
                acc = np.array([
                    0.0,  # x acceleration is zero
                    -r * omega**2 * np.cos(omega * t),
                    -r * omega**2 * np.sin(omega * t)
                ])
                
                # Update right hand reference
                tsid_controller.set_RH_pos_ref(pos, vel, acc)

            tau_sol, dv_sol = tsid_controller.update(q_pin_current, v_pin_current, current_sim_time)
            robot_node.setActuatedJointTorques(tau_sol)

            # Debug info every second
            joint_error = np.linalg.norm(q_pin_current[7:] - conf.q_actuated_home) # Compare with conf.q_actuated_home
            robot_node.get_logger().debug(f"Joint error to home: {joint_error:.4f}")
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
        plt.savefig('com_analysis3.png', dpi=300)
        plt.show()
        
    
if __name__ == '__main__': 
    main()

