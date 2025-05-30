import pybullet as pb
import numpy as np
from simulator.pybullet_wrapper import PybulletWrapper
from simulator.robot import Robot
import pinocchio as pin
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Header


##Everything is more beautiful in a class :)
##Why i changed it? Because i love logger

class RobotSimulation(Node):
    def __init__(self):
        super().__init__('robot_simulation')
        
        
        #Publisher
        self.joint_state_publisher = self.create_publisher(
            JointState, 
            'joint_states', 
            10)
        
        self.done = False
        
        self.urdf = "src/talos_description/robots/talos_reduced.urdf"
        self.path_meshes = "src/talos_description/meshes/../.."
        
        
        '''
        Talos
        0, 1, 2, 3, 4, 5, 			    # left leg
        6, 7, 8, 9, 10, 11, 			# right leg
        12, 13,                         # torso
        14, 15, 16, 17, 18, 19, 20, 21  # left arm
        22, 23, 24, 25, 26, 27, 28, 29  # right arm
        30, 31                          # head

        REEMC
        0, 1, 2, 3, 4, 5, 			    # left leg
        6, 7, 8, 9, 10, 11, 			# right leg
        12, 13,                         # torso
        14, 15, 16, 17, 18, 19, 20,     # left arm
        21, 22, 23, 24, 25, 26, 27,     # right arm
        28, 29                          # head
        '''
        
        # q_home
        self.z_init = 1.15
        self.q_actuated_home = np.zeros(32)
        self.q_actuated_home[:6] = np.array([0, 0, -0.44, 0.9, -0.45, 0])
        self.q_actuated_home[6:12] = np.array([0, 0, -0.44, 0.9, -0.45, 0])
        self.q_actuated_home[14:22] = np.array([0, -0.24, 0, -1, 0, 0, 0, 0])
        self.q_actuated_home[22:30] = np.array([0, -0.24, 0, -1, 0, 0, 0, 0])
        
        
        self.q_home = np.hstack([np.array([0, 0, self.z_init, 0, 0, 0, 1]), self.q_actuated_home])
        # p = position folating base, Q quaternion (floaing base  orientation), q full sytem position state (actuated joints position)

        # setup the task stack
        self.modelWrap = pin.RobotWrapper.BuildFromURDF(self.urdf,
                                                        self.path_meshes,
                                                        pin.JointModelFreeFlyer(),
                                                        True,
                                                        None)
        # Get model from wrapper
        self.model = self.modelWrap.model
        
        # setup the simulator
        self.simulator = PybulletWrapper(sim_rate=1000)
        
        # Create Pybullet-Pinocchio map
        self.robot = Robot(self.simulator,
                          self.urdf,
                          self.model,
                          [0, 0, self.z_init],
                          [0, 0, 0, 1],
                          q=self.q_home,
                          useFixedBase=False,
                          verbose=True)
        
        # Setup pybullet camera
        pb.resetDebugVisualizerCamera(
            cameraDistance=1.2,
            cameraYaw=90,
            cameraPitch=-20,
            cameraTargetPosition=[0.0, 0.0, 0.8])
        
        # Additional setup code
        self.data = self.robot._model.createData()
        
        M = pin.crba(self.robot._model, self.data,self.robot._q )

        pin.ccrba(self.robot._model, self.data, self.robot._q, self.robot._v )
        hg = self.data.hg
        Ag = self.data.Ag
        com = self.data.com[0]

        nle = pin.nonLinearEffects(self.robot._model, self.data, self.robot._q, self.robot._v)
        
        self.simulator.addLinkDebugFrame(-1, -1)
        
        

    
        # Controller setup
        self.sim_dt = 1.0/1000.0
        self.duration = 2.0  
        self.publish_counter = 0
        self.publish_interval_steps = int((1.0/30.0) / self.sim_dt)  

        self.setup_controller()
        self.setup_trajectory()
        
        self.timer = self.create_timer(self.sim_dt, self.simulation_step)
    
    def setup_controller(self):
        # PD controller setup
        kp_base = 300# 300
        kd_base = 0.5
        
        # Create diagonal gain1matrices
        self.Kp = np.eye(32)
        self.Kd = np.eye(32)
        
        # Apply different gains based on robot segments
        # Legs (indices 0-11)
        for i in range(0, 12):
            self.Kp[i, i] = 3.0 * kp_base
            self.Kd[i, i] = 2.0 * kd_base
        # Torso (indices 12-13)
        for i in range(12, 14):
            self.Kp[i, i] = 2.0 * kp_base
            self.Kd[i, i] = 2.0 * kd_base
        # Arms (indices 14-29)
        for i in range(14, 30):
            self.Kp[i, i] = 1.0 * kp_base
            self.Kd[i, i] = 1.0 * kd_base
        # Head (indices 30-31)
        for i in range(30, 32):
            self.Kd[i, i] = 1.0 * kd_base

    def setup_trajectory(self):
        self.q_ini = self.robot._q
        
        #self.get_logger().debug(f"q_ini: {self.q_ini}")
        #self.get_logger().debug(f"q_home: {self.q_home}")
        
        # Generate the trajectory
        self.trajectory = self.spline_interpolation(self.robot, self.robot._model, 
                                               self.data, self.q_ini, self.q_home, 
                                               self.duration, self.sim_dt)
        
        self.trajectory_index = 0
        self.trajectory_active = True
        
    
    def spline_interpolation(self, robot, model, data, q_init, q_home, duration, t):
        steps = int(duration / t)
        trajectory = []
        
        for i in range(steps + 1):
            t = min(1.0, i / steps)
            q_i = pin.interpolate(model, q_init, q_home, t)
            trajectory.append(q_i.copy())
        
        return trajectory
    
    def publish_joint_states(self, q, v, tau):
        joint_state_msg = JointState()
        joint_state_msg.header = Header()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.robot.actuatedJointNames()
        joint_state_msg.position = q.tolist()
        joint_state_msg.velocity = v.tolist()
        joint_state_msg.effort = tau.tolist()
        #self.get_logger().debug(f"Joint State: {joint_state_msg}")
        
        self.joint_state_publisher.publish(joint_state_msg)
    
    def simulation_step(self):
        
        if self.done or not rclpy.ok():
            return
            
        # update the simulator and the robot
        self.simulator.step()
        self.simulator.debug()
        self.robot.update()
            
        if self.trajectory_active and self.trajectory_index < len(self.trajectory):
            # Get the next point in trajectory
            q_desired_full = self.trajectory[self.trajectory_index]
            # Extract only actuated joints for control
            q_desired = q_desired_full[7:]
            self.get_logger().debug(f"q_desired: {q_desired}")
            
            self.trajectory_index += 1
            
            if self.trajectory_index >= len(self.trajectory):
                self.trajectory_active = False
                self.get_logger().info("Trajectory completed!")
        else:
            # Use zero position as desired once trajectory is finished
            q_desired = self.q_home[7:]

        q_current = self.robot._q[7:]
        v_current = self.robot._v[6:]
        
        position_error = q_desired - q_current
        tau = self.Kp @ position_error - self.Kd @ (v_current)
        
        # command to the robot
        self.robot.setActuatedJointTorques(tau)
        
        
        #publishing with counter to reach 30 Hz
        #Note: due to CPU limitation 1kHz is never reached, max 250 Hz (therefor calculation steps to reach 30 Hz is not prefect) 
        self.publish_counter += 1
        if self.publish_counter  >= self.publish_interval_steps:
            self.publish_joint_states(q_current, v_current, tau)
            self.publish_counter = 0

def main(args=None):
    rclpy.init(args=args)
    robot_sim = RobotSimulation()
    try:
        rclpy.spin(robot_sim)
    except KeyboardInterrupt:
        pass
    finally:
        robot_sim.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()