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



DO_PLOT = True

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
            {"direction": PushDirection.RIGHT, "magnitude": 15.0},
            {"direction": PushDirection.LEFT, "magnitude": 15.0},
            {"direction": PushDirection.BACK, "magnitude": 12.0}
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

        self.push_state_machine = PushStateMachine(t_pause=2.0,
                                                   tpush=0.5,
                                                   logger=self.get_logger())
        
        self.force_visualizer = ForceVisualizer(simulator)
        
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
        
    
    
    robot_node.get_logger().info("=== Talos Push Force Test Started ===")
    robot_node.get_logger().info("Push sequence: RIGHT -> LEFT -> BACK")
    robot_node.get_logger().info(f"Timing: {robot_node.push_state_machine.t_pause}s wait, {robot_node.push_state_machine.t_push}s push")


    t_publish = 0.0 # For controlling publish rate


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
            
            p_com = tsid_controller.comState().pos()
            p_RF = tsid_controller.get_placement_RF().translation
            
            #State Machine Update
            robot_node.update_push_system(current_sim_time)
            
        
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
        robot_node.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__': 
    main()

