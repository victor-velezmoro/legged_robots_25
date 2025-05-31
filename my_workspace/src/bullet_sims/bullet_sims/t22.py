import pybullet as pb
import numpy as np
from simulator.pybullet_wrapper import PybulletWrapper
from simulator.robot import Robot
import pinocchio as pin
import rclpy
from rclpy.node import Node


##Everything is more beautiful in a class :)
##Why i changed it? Because i love logging

class RobotSimulation(Node):
    def __init__(self):
        super().__init__('robot_simulation')
        
        
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
        
        
        # Controller setup
        self.setup_controller()
        
        # Trajectory setup
        self.setup_trajectory()
        
        # Start simulation
        self.run_simulation()
    
    def setup_controller(self):
        # PD controller setup
        kp_base = 70.0
        kd_base = 3.0
        
        # Create diagonal gain matrices
        self.Kp = np.eye(32)
        self.Kd = np.eye(32)
        
        # Apply different gains based on robot segments
        # Legs (indices 0-11)
        for i in range(0, 12):
            self.Kp[i, i] = 3.5 * kp_base
            self.Kd[i, i] = 2 * kd_base
        # Torso (indices 12-13)
        for i in range(12, 14):
            self.Kp[i, i] = 3.0 * kp_base
            self.Kd[i, i] = 2.5 * kd_base
        # Arms (indices 14-29)
        for i in range(14, 30):
            self.Kp[i, i] = 0.8 * kp_base
            self.Kd[i, i] = 1.0 * kd_base
        # Head (indices 30-31)
        for i in range(30, 32):
            self.Kp[i, i] = 0.6 * kp_base
            self.Kd[i, i] = 1.0 * kd_base
    
    def setup_trajectory(self):
        self.sim_dt = 1.0/1000.0
        self.duration = 0.5
        self.q_ini = self.robot._q
        
        self.get_logger().debug(f"q_ini: {self.q_ini}")
        self.get_logger().debug(f"q_home: {self.q_home}")
        
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
    
    def run_simulation(self):
        # Needed for compatibility
        self.simulator.addLinkDebugFrame(-1, -1)
        
        done = False
        while not done and rclpy.ok():
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
                q_desired = np.zeros(32)
            
            q_current = self.robot._q[7:]
            v_current = self.robot._v[6:]
            
            position_error = q_desired - q_current
            tau = self.Kp @ position_error - self.Kd @ (v_current)
            
            # command to the robot
            self.robot.setActuatedJointTorques(tau)

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