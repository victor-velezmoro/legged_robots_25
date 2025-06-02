import pybullet as pb
import numpy as np
from simulator.pybullet_wrapper import PybulletWrapper
from simulator.robot import Robot
import pinocchio as pin

# For REEM-C robot
#urdf = "src/reemc_description/robots/reemc.urdf"
#path_meshes = "src/reemc_description/meshes/../.."

# For Talos robot
urdf = "src/talos_description/robots/talos_reduced.urdf"
path_meshes = "src/talos_description/meshes/../.."

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

# Initial condition for the simulator an model
z_init = 1.15
q_actuated_home = np.zeros(32)
q_actuated_home[:6] = np.array([0, 0, 0, 0, 0, 0])
q_actuated_home[6:12] = np.array([0, 0, 0, 0, 0, 0])
q_actuated_home[14:22] = np.array([0, 0, 0, 0, 0, 0, 0, 0 ])
q_actuated_home[22:30] = np.array([0, 0, 0, 0, 0, 0, 0, 0 ])

# Initialization position including floating base
q_home = np.hstack([np.array([0, 0, z_init, 0, 0, 0, 1]), q_actuated_home])
# p = position folating base, Q quaternion (floaing base  orientation), q full sytem position state (actuated joints position)

# setup the task stack
modelWrap = pin.RobotWrapper.BuildFromURDF(urdf,                        # Model description
                                           path_meshes,                 # Model geometry descriptors 
                                           pin.JointModelFreeFlyer(),   # Floating base model. Use "None" if fixed
                                           True,                        # Printout model details
                                           None)                        # Load meshes different from the descripor
# Get model from wrapper
model = modelWrap.model



# setup the simulator
simulator = PybulletWrapper(sim_rate=1000)

#Create Pybullet-Pinocchio map
robot = Robot(simulator,            # The Pybullet wrapper
              urdf,                 # Robot descriptor
              model,                # Pinocchio model
              [0, 0, z_init],       # Floating base initial position
              [0,0,0,1],            # Floating base initial orientation [x,y,z,w]
              q=q_home,             # Initial state
              useFixedBase=False,   # Fixed base or not
              verbose=True)         # Printout details

#for q_home vector = robot.q()
#for velocity vector = robot.v()
data = robot._model.createData()
M = pin.crba(robot._model, data,robot._q )



pin.ccrba(robot._model, data, robot._q, robot._v )
hg = data.hg
Ag = data.Ag
com = data.com[0]

nle = pin.nonLinearEffects(robot._model, data, robot._q, robot._v)





###############
## PD CONTROLLER
###############

kp_base = 70.0  
kd_base = 3.0   

# Create diagonal gain matrices
Kp = np.eye(32)
Kd = np.eye(32)

for i in range(0, 12):
    Kp[i, i] = 3.5 * kp_base  
    Kd[i, i] = 2 * kd_base  
# Torso (indices 12-13)
for i in range(12, 14):
    Kp[i, i] = 3.0 * kp_base
    Kd[i, i] = 2.5 * kd_base
# Arms (indices 14-29)
for i in range(14, 30):
    Kp[i, i] = 0.8 * kp_base
    Kd[i, i] = 1.0 * kd_base

# Head (indices 30-31):
for i in range(30, 32):
    Kp[i, i] = 0.6 * kp_base
    Kd[i, i] = 1.0 * kd_base


q_desired = np.zeros(32)

#Needed for compatibility
simulator.addLinkDebugFrame(-1,-1)

# Setup pybullet camera
pb.resetDebugVisualizerCamera(
    cameraDistance=1.2,
    cameraYaw=90,
    cameraPitch=-20,
    cameraTargetPosition=[0.0, 0.0, 0.8])

# Joint command vector
#tau = q_actuated_home*0

tau = np.zeros(32)


done = False
while not done:
    # update the simulator and the robot
    simulator.step()
    simulator.debug()
    robot.update()
    
    q_current = robot._q[7:]
    v_current = robot._v[6:]
    
    position_error = q_desired - q_current
    tau = Kp @ position_error - Kd @ (v_current)

    
    
    # command to the robot
    robot.setActuatedJointTorques(tau)