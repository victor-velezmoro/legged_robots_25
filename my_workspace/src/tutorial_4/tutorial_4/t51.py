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
from enum import Enum

import matplotlib.pyplot as plt
from collections import deque
import threading

DO_Plot = False

class PushDirection():
    RIGHT = np.array([0.0, -1.0, 0.0])  
    LEFT = np.array([0.0, 1.0, 0.0])    
    BACK = np.array([-1.0, 0.0, 0.0])  
    
    @staticmethod
    def get_name(direction):
        if np.allclose(direction, PushDirection.RIGHT):
            return "RIGHT"
        elif np.allclose(direction, PushDirection.LEFT):
            return "LEFT"
        elif np.allclose(direction, PushDirection.BACK):
            return "BACK"
        return "UNKNOWN"
    
class PushState(Enum):
    WAITING = "waiting"
    PUSHING = "pushing"
    COMPLETED = "completed"

class PushStateMachine:
    def __init__(self,t_pause, tpush, logger):
        self.t_pause = t_pause
        self.t_push = tpush
        self.logger = logger
        
        self.push_sequence = [
            {"direction": PushDirection.RIGHT, "magnitude": 25.0},
            {"direction": PushDirection.LEFT, "magnitude": 25.0},
            {"direction": PushDirection.BACK, "magnitude": 25.0}
        ]
        # State tracking
        self.state = PushState.WAITING
        self.current_push_index = 0
        self.state_start_time = 0.0
        self.cycle_start_time = 0.0
        self.force_vector = np.zeros(3)
        
        # Visualization
        self.line_id = -1
        

    def update(self, current_time):
        
        if self.current_push_index >= len(self.push_sequence):
            # All pushes completed
            self.state = PushState.COMPLETED
            self.force_vector = np.zeros(3)
            return self.force_vector, False, False
        
        time_in_cycle = current_time - self.cycle_start_time
        
        if self.state == PushState.WAITING:
            self.force_vector = np.zeros(3)
            
            # Check if it's time to start pushing
            if time_in_cycle >= self.t_pause:
                self._start_push(current_time)
                return self.force_vector, False, True
            
        elif self.state == PushState.PUSHING:
            # Apply current push force
            current_push = self.push_sequence[self.current_push_index]
            self.force_vector = current_push["magnitude"] * current_push["direction"]
            
            time_pushing = current_time - self.state_start_time
            
            # Check if push duration is complete
            if time_pushing >= self.t_push:
                self._end_push(current_time)
                return self.force_vector, False, False
            
            return self.force_vector, True, True  # Visualize and need hip position
        
        return self.force_vector, False, False
    
    def _start_push(self, current_time):
        """Start a new push"""
        self.state = PushState.PUSHING
        self.state_start_time = current_time
        
        current_push = self.push_sequence[self.current_push_index]
        direction_name = current_push["direction"]
        
        self.logger.info(f"Starting push {self.current_push_index + 1}/3: {direction_name} "
                        f"with {current_push['magnitude']}N for {self.t_push}s")
    
    def _end_push(self, current_time):
        """End current push and prepare for next"""
        self.state = PushState.WAITING
        self.current_push_index += 1
        self.cycle_start_time = current_time
        self.force_vector = np.zeros(3)
        
        if self.current_push_index < len(self.push_sequence):
            self.logger.info(f"Push completed. Waiting {self.t_pause}s for next push...")
        else:
            self.logger.info("All pushes completed!")
    
    def get_current_state_info(self):
        """Return current state information for logging"""
        if self.current_push_index >= len(self.push_sequence):
            return "All pushes completed"
        
        current_push = self.push_sequence[self.current_push_index]
        direction_name = current_push["direction"]
        return f"State: {self.state.value}, Push: {self.current_push_index + 1}/3 ({direction_name})"
   
class DataRecorder:
    def __init__(self, max_samples=10000):
        self.max_samples = max_samples
        self.time_data = deque(maxlen=max_samples)
        self.com_x = deque(maxlen=max_samples)
        self.com_y = deque(maxlen=max_samples)
        self.zmp_x = deque(maxlen=max_samples)
        self.zmp_y = deque(maxlen=max_samples)
        self.cmp_x = deque(maxlen=max_samples)
        self.cmp_y = deque(maxlen=max_samples)
        self.cp_x = deque(maxlen=max_samples)
        self.cp_y = deque(maxlen=max_samples)
        
    def record(self, time, com_pos, zmp_pos, cmp_pos, cp_pos):
        self.time_data.append(time)
        self.com_x.append(com_pos[0])
        self.com_y.append(com_pos[1])
        self.zmp_x.append(zmp_pos[0])
        self.zmp_y.append(zmp_pos[1])
        self.cmp_x.append(cmp_pos[0])
        self.cmp_y.append(cmp_pos[1])
        self.cp_x.append(cp_pos[0])
        self.cp_y.append(cp_pos[1])
    
    def save_data(self, filename="robot_data.npz"):
        np.savez(filename,
                 time=np.array(self.time_data),
                 com_x=np.array(self.com_x), com_y=np.array(self.com_y),
                 zmp_x=np.array(self.zmp_x), zmp_y=np.array(self.zmp_y),
                 cmp_x=np.array(self.cmp_x), cmp_y=np.array(self.cmp_y),
                 cp_x=np.array(self.cp_x), cp_y=np.array(self.cp_y))
        print(f"Data saved to {filename}")
    
    def plot_realtime(self):
        if len(self.time_data) < 2:
            return
            
        plt.clf()
        
        # Plot X components
        plt.subplot(2, 1, 1)
        plt.plot(self.time_data, self.com_x, 'b-', label='CoM X', linewidth=2)
        plt.plot(self.time_data, self.zmp_x, 'r-', label='ZMP X', linewidth=1)
        plt.plot(self.time_data, self.cmp_x, 'g-', label='CMP X', linewidth=1)
        plt.plot(self.time_data, self.cp_x, 'm-', label='CP/DCM X', linewidth=1)
        plt.ylabel('X Position (m)')
        plt.legend()
        plt.grid(True)
        plt.title('Ground Reference Points and CoM - X Components')
        
        # Plot Y components
        plt.subplot(2, 1, 2)
        plt.plot(self.time_data, self.com_y, 'b-', label='CoM Y', linewidth=2)
        plt.plot(self.time_data, self.zmp_y, 'r-', label='ZMP Y', linewidth=1)
        plt.plot(self.time_data, self.cmp_y, 'g-', label='CMP Y', linewidth=1)
        plt.plot(self.time_data, self.cp_y, 'm-', label='CP/DCM Y', linewidth=1)
        plt.xlabel('Time (s)')
        plt.ylabel('Y Position (m)')
        plt.legend()
        plt.grid(True)
        
        plt.tight_layout()
        plt.pause(0.01)
        
class ForceVisualizer(Node, Robot):
    def __init__(self, simulator):
        self.simulator = simulator
        self.line_id = -1
    
    def visualize_force(self, start_pos, force_vector, scale_factor=0.1):
        """Visualize force as a red line in pybullet"""
        if np.linalg.norm(force_vector) > 0:
            end_pos = start_pos + force_vector * scale_factor
            self.line_id = self.simulator.addGlobalDebugLine(
                start_pos, end_pos, self.line_id, color=[1, 0, 0]
            )
            return self.line_id
        return -1
    
    def remove_visualization(self):
        """Remove the current force visualization"""
        if self.line_id != -1:
            self.simulator.removeDebugItem(self.line_id)
            self.line_id = -1

class AnkleController:
    def __init__(self):
        self.Kx = 3.0
        self.Kp = 1.5
        self.omega = 2.45
        
        self.x_ref = np.array([0.0, 0.0, 0.9])
        self.v_ref = np.array([0.0, 0.0, 0.0])
        self.p_ref = np.array([0.0, 0.0, 0.0])
        
        self.x_desired = self.x_ref.copy()
        self.initialized = True
        
    def update(self, current_zmp, current_com_pose):
        
        if not self.initialized:
            if current_com_pose is None:
                raise ValueError("Current CoM pose must be provided for initialization.")
            self.x_ref = current_com_pose.copy()
            self.x_desired = current_com_pose.copy()
            self.initialized = True
            
            print(f"Ankle controller initialized with CoM: {current_com_pose}")
            return self.x_desired, self.v_ref.copy()
        
        position_error = self.x_desired - self.x_ref
        zmp_error = current_zmp - self.p_ref
        
        v_desired = np.zeros(3)
        v_desired[0] = self.v_ref[0] - self.Kx * position_error[0] + self.Kp * zmp_error[0]
        v_desired[1] = self.v_ref[1] - self.Kx * position_error[1] + self.Kp * zmp_error[1]
        v_desired[2] = self.v_ref[2] - self.Kx * position_error[2] + self.Kp * zmp_error[2]
        
        dt = 0.001
        self.x_desired[0:2] += v_desired[0:2] * dt
        self.x_desired[2] = self.x_ref[2]  
        
        
        
        return self.x_desired, v_desired
    
class Talos(Robot, Node):
    def __init__(self, simulator, urdf, model, q=None, verbose=True, use_fixed_base=True):
        # Initialize as an rclpy Node
        Node.__init__(self, 'talos_controller_node')

        # Determine initial base pose from q or use defaults from conf.q_home if q is similar
        initial_base_pos = np.array([0.0, 0.0, 1.1]) # Default from conf.q_home
        initial_base_quat = np.array([0.0, 0.0, 0.0, 1.0]) # Default from conf.q_home (x,y,z,w)

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

        self.push_state_machine = PushStateMachine(t_pause=1.0,
                                                   tpush=0.5,
                                                   logger=self.get_logger())
        
        self.force_visualizer = ForceVisualizer(simulator)
        
        self.ankle_controller = AnkleController()
        
        self.hip_frame_name = "base_link"
        
    def apply_push_force(self, force_vector):
        """Apply push force to robot hip"""
        # Use self.applyForce() or pb.applyExternalForce()
        if np.linalg.norm(force_vector) > 0:
            hip_position = self.get_hip_position()
            self.applyForce(force_vector, hip_position)

    def get_hip_position(self):
        """Get current hip/base position for visualization"""
        hip_position = self.q()[:3]  
        return hip_position
        
    
    def update_push_system(self, current_time):
        """Main update function for push system"""
        force_vector, should_visualize, need_hip_pos = self.push_state_machine.update(current_time)
        self.apply_push_force(force_vector)
        
        # 3. Handle visualization
        if should_visualize and need_hip_pos:
            hip_pos = self.get_hip_position()
            self.force_visualizer.visualize_force(hip_pos, force_vector)
        elif not should_visualize:
            self.force_visualizer.remove_visualization()
        
        # 4. Log state changes (every few seconds to avoid spam)
        if int(current_time) % 2 == 0 and current_time - int(current_time) < 0.01:
            state_info = self.push_state_machine.get_current_state_info()
            if np.linalg.norm(force_vector) > 0:
                self.get_logger().info(f"{state_info} | Force: {np.linalg.norm(force_vector):.1f}N")
            else:
                self.get_logger().debug(state_info)
        
    
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
    
    
    def com_controller():
            robot_node.update() 
            q_pin_current = robot_node.q()
            v_pin_current = robot_node.v()
            
            p_com = tsid_controller.comState().pos()
            p_goal = np.array([0.0, 0.0, 1.1])  # Desired CoM position

            p_ref = np.array([p_com[0], p_com[1], p_goal[2]])
            tsid_controller.setComRefState(p_ref)
            robot_node.get_logger().info(f"Set CoM reference to: {p_ref}")
        
    pb.enableJointForceTorqueSensor(robot_node.id(), robot_node.jointNameIndexMap()['leg_right_6_joint'], True)
    pb.enableJointForceTorqueSensor(robot_node.id(), robot_node.jointNameIndexMap()['leg_left_6_joint'], True)

    
    robot_node.get_logger().info("=== Talos Push Force Test Started ===")
    robot_node.get_logger().info("Push sequence: RIGHT -> LEFT -> BACK")
    robot_node.get_logger().info(f"Timing: {robot_node.push_state_machine.t_pause}s wait, {robot_node.push_state_machine.t_push}s push")


    t_publish = 0.0 # For controlling publish rate
    
    data_recorder = DataRecorder()
    plt.ion()
    plt.figure(figsize=(12, 8))


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
            q_pin_current = robot_node.q()
            v_pin_current = robot_node.v()
            
            #State Machine Update
            robot_node.update_push_system(current_sim_time)
            
            
            wren = pb.getJointState(robot_node.id(), robot_node.jointNameIndexMap()['leg_right_6_joint'])[2]
            wnp = np.array([-wren[0], -wren[1], -wren[2], -wren[3], -wren[4], -wren[5]])
            wr_ankle = pin.Force(wnp)
            wren = pb.getJointState(robot_node.id(), robot_node.jointNameIndexMap()['leg_left_6_joint'])[2]
            wnp = np.array([-wren[0], -wren[1], -wren[2], -wren[3], -wren[4], -wren[5]])
            wl_ankle = pin.Force(wnp)
            
            data = robot_node._model.createData()
            pin.framesForwardKinematics(tsid_controller.model, data, q_pin_current)
            H_w_lsole = data.oMf[tsid_controller.model.getFrameId("left_sole_link")]
            H_w_rsole = data.oMf[tsid_controller.model.getFrameId("right_sole_link")]
            H_w_lankle = data.oMf[tsid_controller.model.getFrameId("leg_left_6_joint")]
            H_w_rankle = data.oMf[tsid_controller.model.getFrameId("leg_right_6_joint")]
            
            def calculate_zmp(wr_ankle, wl_ankle, H_w_lsole, H_w_rsole, H_w_lankle, H_w_rankle):
                
                # Calculate ZMP based on ankle forces and foot placements
                f_r_ankle = wr_ankle.linear
                tau_r_ankle = wr_ankle.angular
                f_l_ankle = wl_ankle.linear
                tau_l_ankle = wl_ankle.angular
                

                p_r_sole = H_w_rsole.translation
                p_l_sole = H_w_lsole.translation
                
                d = 0.1 
                
                if abs(f_r_ankle[2]) > 1.0:  
                    p_xR = (-tau_r_ankle[1] - f_r_ankle[0] * d) / f_r_ankle[2]
                    p_yR = (tau_r_ankle[0] - f_r_ankle[1] * d) / f_r_ankle[2]
                    # Transform to world coordinates
                    zmp_r_world = p_r_sole + np.array([p_xR, p_yR, 0.0])
                else:
                    p_xR = p_yR = 0.0
                    zmp_r_world = p_r_sole
                    f_r_ankle[2] = 0.0  # No contribution
                
                # Left foot ZMP  
                if abs(f_l_ankle[2]) > 1.0:  
                    p_xL = (-tau_l_ankle[1] - f_l_ankle[0] * d) / f_l_ankle[2]
                    p_yL = (tau_l_ankle[0] - f_l_ankle[1] * d) / f_l_ankle[2]
                    # Transform to world coordinates
                    zmp_l_world = p_l_sole + np.array([p_xL, p_yL, 0.0])
                else:
                    p_xL = p_yL = 0.0
                    zmp_l_world = p_l_sole
                    f_l_ankle[2] = 0.0  # No contribution
                
                # Apply the weighted ZMP formula from your attachment
                f_zR = abs(f_r_ankle[2])
                f_zL = abs(f_l_ankle[2])
                total_fz = f_zR + f_zL
                
                if total_fz > 1.0:  
                    # Weighted average using the exact formula from attachment
                    p_x = (zmp_r_world[0] * f_zR + zmp_l_world[0] * f_zL) / total_fz
                    p_y = (zmp_r_world[1] * f_zR + zmp_l_world[1] * f_zL) / total_fz
                    p_z = 0.0  # ZMP is on the ground
                    
                    return np.array([p_x, p_y, p_z])
                
                return np.array([0.0, 0.0, 0.0])
            
            def calculate_cmp(robot_node, tsid_controller, wr_ankle, wl_ankle):
                
                x = tsid_controller.comState().pos()
                grf = wr_ankle.linear + wl_ankle.linear
                f = grf
                
                if abs(f[2]) > 1.0:
                    r_x = x[0] - (f[0] / f[2]) * x[2]
                    r_y = x[1] - (f[1] / f[2]) * x[2]
                    r_z = 0.0  # CMP is on the ground
                    return np.array([r_x, r_y, r_z])
                else:
                    return np.array([x[0], x[1], 0.0])
            
            def calculate_cp_dcm(robot_node, tsid_controller, gravity=9.81):
                com_state = tsid_controller.comState()
                x_com = com_state.pos()
                v_com = com_state.vel()
                x_com = np.array(x_com) if not isinstance(x_com, np.ndarray) else x_com
                
                v_com = np.array(v_com) if not isinstance(v_com, np.ndarray) else v_com
                
                com_hight = x_com[2]
                omega = np.sqrt(gravity / com_hight)
                
                xi_x = x_com[0] + (v_com[0] / omega)
                xi_y = x_com[1] + (v_com[1] / omega)
                xi_z = 0.0  
                
                cp_dcm = np.array([xi_x, xi_y, xi_z])
                return cp_dcm, omega
                
            zmp_position = calculate_zmp(wr_ankle, wl_ankle, H_w_rankle, H_w_lankle, H_w_rsole, H_w_lsole)
            cmp_position = calculate_cmp(robot_node, tsid_controller, wr_ankle, wl_ankle)
            cp_dcm_position, omega = calculate_cp_dcm(robot_node, tsid_controller, gravity=9.81)
            
            com_position = tsid_controller.comState().pos()
            robot_node.get_logger().info(f"CoM position: [{com_position[0]:.3f}, {com_position[1]:.3f}, {com_position[2]:.3f}]")
            com_velocity = tsid_controller.comState().vel()
            
            desired_com_pos, desired_com_vel = robot_node.ankle_controller.update( 
                current_zmp=zmp_position,
                current_com_pose=com_position
            )
            
            tsid_controller.setComRefState(desired_com_pos, desired_com_vel)

            if int(current_sim_time) % 2 == 0:
                zmp_error = np.linalg.norm(zmp_position[0:2] - robot_node.ankle_controller.p_ref[0:2])
                robot_node.get_logger().info(f"ZMP error: {zmp_error:.4f}m, Desired CoM: [{desired_com_pos[0]:.3f}, {desired_com_pos[1]:.3f}]")
                robot_node.get_logger().info(f"Desired CoM velocity: [{desired_com_vel[0]:.3f}, {desired_com_vel[1]:.3f}, {desired_com_vel[2]:.3f}]")
            
            if DO_Plot==True:
                data_recorder.record(current_sim_time, com_position, zmp_position, cmp_position, cp_dcm_position)
            
                if int(current_sim_time * 100)%10 == 0:
                    data_recorder.plot_realtime()
            
            
            # Debug info every 2 seconds
            if int(current_sim_time) % 2 == 0:
                force_right = np.linalg.norm(wr_ankle.linear)
                force_left = np.linalg.norm(wl_ankle.linear)
                robot_node.get_logger().debug(f"Ankle forces - Right: {force_right:.2f}N, Left: {force_left:.2f}N")
                robot_node.get_logger().info(f"ZMP position: [{zmp_position[0]:.3f}, {zmp_position[1]:.3f}, {zmp_position[2]:.3f}]")
                robot_node.get_logger().debug(f"CMP position: [{cmp_position[0]:.3f}, {cmp_position[1]:.3f}, {cmp_position[2]:.3f}]")
                robot_node.get_logger().debug(f"CP/DCM position: [{cp_dcm_position[0]:.3f}, {cp_dcm_position[1]:.3f}, {cp_dcm_position[2]:.3f}]")


            tau_sol, dv_sol = tsid_controller.update(q_pin_current, v_pin_current, current_sim_time)
            robot_node.setActuatedJointTorques(tau_sol)

            # Debug info every second
            joint_error = np.linalg.norm(q_pin_current[7:] - conf.q_actuated_home) # Compare with conf.q_actuated_home
            robot_node.get_logger().debug(f"Joint error to home: {joint_error:.4f}")
            base_height = q_pin_current[2]
            robot_node.get_logger().debug(f"Base height: {base_height:.3f}m")
            robot_node.get_logger().debug(f"Current simulation time: {current_sim_time:.3f}s")

            # Publish to ROS at a controlled rate (e.g., 30 Hz)
            if current_sim_time - t_publish >= (1.0 / 30.0):
                t_publish = current_sim_time
                robot_node.publish() 
    
    except KeyboardInterrupt:
        robot_node.get_logger().info("Keyboard interrupt received, shutting down.")
    finally:
        # Cleanly shutdown ROS
        data_recorder.save_data("robot_data_test2.npz")
        plt.ioff
        plt.show()
        robot_node.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__': 
    main()

