import numpy as np
import numpy.linalg as la

# simulator (#TODO: set your own import path!)
from simulator.pybullet_wrapper import PybulletWrapper
from simulator.robot import Robot

# modeling
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
from std_msgs.msg import String, Header

from enum import Enum

# ROS
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

################################################################################
# utility functions
################################################################################

class State(Enum):
    JOINT_SPLINE = 0,
    CART_SPLINE = 1

################################################################################
# Robot
################################################################################

class Talos(Robot):
    def __init__(self, simulator, q=None, verbose=True, useFixedBase=True):
        
        
        
        self.urdf = "src/talos_description/robots/talos_reduced.urdf"
        self.path_meshes = "src/talos_description/meshes/../.."
        
        self.wrapper = RobotWrapper.BuildFromURDF(
            self.urdf,
            self.path_meshes,
            verbose=verbose)

        self.model = self.wrapper.model
        self.data = self.wrapper.data

        print("RobotWrapper loaded with {} joints".format(self.model.nq))

        super().__init__(simulator,
                        self.urdf,
                        self.model,
                        [0, 0, 1.15],
                        [0, 0, 0, 1],
                        q=q,
                        useFixedBase=useFixedBase,
                        verbose=verbose)
        
        # Publisher will be set from outside (placeholder for injection)
        self.joint_state_publisher = None
        self.node = None
        
        
    def update(self):
        # TODO: update base class, update pinocchio robot wrapper's kinematics
        super().update()
        
        self.wrapper.forwardKinematics(self._q)
    
    def wrapper(self):
        return self._wrapper

    def data(self):
        return self._wrapper.data
    
    def publish(self):
        # TODO: publish robot state to ros
        
        if self.joint_state_publisher is None or self.node is None:
            return
            
        # Get current joint states
        q = self._q  # joint positions
        v = self._v  # joint velocities
        tau = np.zeros(len(q))  # joint efforts
        
        joint_state_msg = JointState()
        joint_state_msg.header = Header()
        joint_state_msg.header.stamp = self.node.get_clock().now().to_msg()
        joint_state_msg.name = self.actuatedJointNames()
        joint_state_msg.position = q.tolist()
        joint_state_msg.velocity = v.tolist()
        joint_state_msg.effort = tau.tolist()
        
        self.joint_state_publisher.publish(joint_state_msg)
        

################################################################################
# Controllers
################################################################################

class JointSpaceController:
    """JointSpaceController
    Tracking controller in jointspace
    """
    def __init__(self, robot, Kp, Kd):        
        # Save gains, robot ref
        None
    
    def update(self, q_r, q_r_dot, q_r_ddot):
        # Compute jointspace torque, return torque
        None
    
class CartesianSpaceController:
    """CartesianSpaceController
    Tracking controller in cartspace
    """
    def __init__(self, robot, joint_name, Kp, Kd):
        # save gains, robot ref
        None
        
    def update(self, X_r, X_dot_r, X_ddot_r):
        # compute cartesian control torque, return torque
        None

################################################################################
# Application
################################################################################
    
class Environment(Node):
    def __init__(self):
        super().__init__('robot_environment')
        
        # state
        self.cur_state = State.JOINT_SPLINE
        
        # create simulation
        self.simulator = PybulletWrapper()
        
        self.joint_state_publisher = self.create_publisher(
            JointState, 
            'joint_states', 
            10)
        
        ########################################################################
        # spawn the robot
        ########################################################################
        self.q_home = np.zeros(32)
        self.q_home[14:22] = np.array([0, +0.45, 0, -1, 0, 0, 0, 0 ])
        self.q_home[22:30] = np.array([0, -0.45, 0, -1, 0, 0, 0, 0 ])
        
        self.q_init = np.zeros(32)
        
        self.robot = Talos(self.simulator, q=self.q_init, verbose=True, useFixedBase=True)

        self.robot.joint_state_publisher = self.joint_state_publisher
        self.robot.node = self

        ########################################################################
        # joint space spline: init -> home
        ########################################################################

        # TODO: create a joint spline 
        # TODO: create a joint controller
        
        ########################################################################
        # cart space: hand motion
        ########################################################################

        # TODO: create a cartesian controller
        
        ########################################################################
        # logging
        ########################################################################
        
        # TODO: publish robot state every 0.01 s to ROS
        self.t_publish = 0.0
        self.publish_period = 0.01
        
    def update(self, t, dt):
        
        # TODO: update the robot and model
        self.robot.update()

        # update the controllers
        # TODO: Do inital jointspace, switch to cartesianspace control
        
        # command the robot
        # self.robot.setActuatedJointTorques(tau)
            
        if t - self.t_publish >= self.publish_period:
            self.robot.publish()
            self.t_publish = t

        
def main():
    rclpy.init()  
    env = Environment()
    
    try:
        while rclpy.ok():
            t = env.simulator.simTime()
            dt = env.simulator.stepTime()
            
            env.update(t, dt)
            
            env.simulator.debug()
            env.simulator.step()
            
            # Spin ROS callbacks
            rclpy.spin_once(env, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        env.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__': 
    main()
    
