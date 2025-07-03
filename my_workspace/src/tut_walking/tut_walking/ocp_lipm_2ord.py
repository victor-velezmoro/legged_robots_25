"""Task2: Linear inverted pendulum Trajectory planning

The goal of this file is to formulate the optimal control problem (OCP)
in equation 12. 

In this case we will solve the trajectory planning over the entire footstep plan
(= horizon) in one go.

Our state will be the position and velocity of the pendulum in the 2d plane.
x = [cx, vx, cy, vy]
And your control the ZMP position in the 2d plane
u = [px, py]

You will need to fill in the TODO to solve the task.
"""

import numpy as np

from pydrake.all import MathematicalProgram, Solve, eq

import matplotlib.pyplot as plt
plt.style.use('seaborn-dark')

################################################################################
# settings
################################################################################

# Robot Parameters:
# --------------

h           = 0.80   # fixed CoM height (assuming walking on a flat terrain)
g           = 9.81   # norm of the gravity vector
foot_length = 0.10   # foot size in the x-direction
foot_width  = 0.06   # foot size in the y-direciton

# OCP Parameters:
# --------------
T                     = 0.1                                # fixed sampling time interval of computing the ocp in [s]
STEP_TIME             = 0.8                                # fixed time needed for every foot step [s]

NO_SAMPLES_PER_STEP   = int(round(STEP_TIME/T))            # number of ocp samples per step

NO_STEPS              = 10                                 # total number of foot steps in the plan
TOTAL_NO_SAMPLES      = NO_SAMPLES_PER_STEP*NO_STEPS       # total number of ocp samples over the complete plan (= Horizon)

# Cost Parameters:
# ---------------
alpha       = 10**(-1)                                      # ZMP error squared cost weight (= tracking cost)
gamma       = 10**(-3)                                      # CoM velocity error squared cost weight (= smoothing cost)

################################################################################
# helper function for visualization and dynamics
################################################################################

def generate_foot_steps(foot_step_0, step_size_x, no_steps):
    """Write a function that generates footstep of step size = step_size_x in the 
    x direction starting from foot_step_0 located at (x0, y0).
    
    Args:
        foot_step_0 (_type_): first footstep position (x0, y0)
        step_size_x (_type_): step size in x direction
        no_steps (_type_): number of steps to take
    """

    #>>>>TODO: generate the foot step plan with no_steps
    #>>>>Hint: Check the pdf Fig.3 for inspiration
    # Assuming no step size in y direction for simplicity
    foot_steps = None
    foot_steps = np.zeros((no_steps, 2)) 
    foot_steps[0] = foot_step_0 
    for i in range(1, no_steps):
        
        foot_steps[i, 0] = foot_steps[i-1, 0] + step_size_x
        foot_steps[i, 1] = foot_steps[i-1, 1] - step_size_x * (-1)**i  

    return foot_steps


def plot_foot_steps(foot_steps, XY_foot_print, ax):
    """Write a function that plots footsteps in the xy plane using the given
    footprint (length, width)
    You can use the function ax.fill() to gerneate a colored rectanges.
    Color the left and right steps different and check if the step sequence makes sense.

    Args:
        foot_steps (_type_): the foot step plan
        XY_foot_print (_type_): the dimensions of the foot (x,y)
        ax (_type_): the axis to plot on
    """
    #>>>>TODO: Plot the the footsteps into ax 
    foot_length, foot_width = XY_foot_print
    for i, step in enumerate(foot_steps):
        center_x, center_y = step
        x_corners = [center_x - foot_length / 2, center_x + foot_length / 2,
                     center_x + foot_length / 2, center_x - foot_length / 2]
        y_corners = [center_y - foot_width / 2, center_y - foot_width / 2,
                     center_y + foot_width / 2, center_y + foot_width / 2]
        color = 'blue' if i % 2 == 0 else 'orange'  
        ax.fill(x_corners, y_corners, color=color, alpha=0.5, edgecolor='black', linewidth=1)


    ax.legend()
    ax.set_xlabel('X Position [m]')
    ax.set_ylabel('Y Position [m]')
    ax.set_title('Footstep Plan')
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')

def generate_zmp_reference(foot_steps, no_samples_per_step):
    """generate a function that computes a referecne trajecotry for the ZMP
    (We need this for the tracking cost in the cost function of eq. 12)
    Remember: Our goal is to keep the ZMP at the footstep center within each step.
    So for the # of samples a step is active the zmp_ref should be at that step.
    
    Returns a vector of size (TOTAL_NO_SAMPLES, 2)

    Args:
        foot_steps (_type_): the foot step plan
        no_samples_per_step (_type_): number of sampes per step
    """
    #>>>>TODO: Generate the ZMP reference based on given foot_steps
    total_samples = len(foot_steps) * no_samples_per_step
    zmp_ref = np.zeros((total_samples, 2))
    for i, step in enumerate(foot_steps):
        start = i * no_samples_per_step
        end = start + no_samples_per_step
        zmp_ref[start:end, 0] = step[0] 
        zmp_ref[start:end, 1] = step[1]  
        
    return zmp_ref

################################################################################
# Dynamics of the simplified walking model
################################################################################

def continious_LIP_dynamics(g, h):
    """returns the matrices A,B of the continious LIP dynamics

    Args:
        g (_type_): gravity
        h (_type_): fixed height

    Returns:
        np.array: A, B
    """
    #>>>>TODO: Generate A, B for the continous linear inverted pendulum
    #>>>>Hint: Look at Eq. 4 and rewrite as a system first order diff. eq.
    #first order diff
    #x = [cx, c_vx, cy, c_vy]
    #zmp equation
    A = np.array([[0,   1,   0,   0],
                  [g/h, 0,   0,   0],
                  [0,   0,   0,   1],
                  [0,   0,   g/h, 0]])
    
    B = np.array([[0,    0],
                  [-g/h, 0],
                  [0,    0],
                  [0,    -g/h]])
    
    return A, B

def discrete_LIP_dynamics(delta_t, g, h):
    """returns the matrices static Ad,Bd of the discretized LIP dynamics

    Args:
        delta_t (_type_): discretization steps
        g (_type_): gravity
        h (_type_): height

    Returns:
        _type_: _description_
    """
    #>>>>TODO: Generate Ad, Bd for the discretized linear inverted pendulum
    omega = np.sqrt(g/h)
    Ad_xy = np.array([[np.cosh(omega*delta_t), np.sinh(omega*delta_t)/omega],
                   [omega*np.sinh(omega*delta_t), np.cosh(omega*delta_t)]])
    Bd_xy = np.array([1-np.cosh(omega*delta_t), -omega *np.sinh(omega*delta_t)])
    
    Ad = np.zeros((4, 4))
    Ad[0:2, 0:2] = Ad_xy
    Ad[2:4, 2:4] = Ad_xy
    
    Bd = np.zeros((4, 2))
    Bd[0:2, 0] = Bd_xy.flatten()
    Bd[2:4, 1] = Bd_xy.flatten()
    
    return Ad, Bd

################################################################################
# setup the plan references and system matrices
################################################################################

# inital state in x0 = [px0, vx0]
x_0 = np.array([0.0, 0.0])
# inital state in y0 = [py0, vy0]
y_0 = np.array([-0.09, 0.0])

# footprint
footprint = np.array([foot_length, foot_width])

# generate the footsteps
step_size = 0.2
#>>>>TODO: 1. generate the foot step plan using generate_foot_steps
first_foot_step = np.array([x_0[0], y_0[0]])  # Starting footstep position
foot_steps = generate_foot_steps(first_foot_step, step_size, NO_STEPS)

# zmp reference trajecotry
#>>>>TODO: 2. generate the ZMP reference using generate_zmp_reference
zmp_ref = generate_zmp_reference(foot_steps, NO_SAMPLES_PER_STEP)

#>>>>Note: At this point you can already start plotting things to see if they
# really make sense!
# plot the foot steps
# fig, ax = plt.subplots(figsize=(8, 6))
# plot_foot_steps(foot_steps, footprint, ax)
# ax.plot(zmp_ref[:,0], zmp_ref[:,1], '-o', label='ZMP Reference')
# ax.legend()


# discrete LIP dynamics
#>>>>TODO: get the static dynamic matrix Ad, Bd
Ad, Bd = discrete_LIP_dynamics(T, g, h)

# continous LIP dynamics
#>>>>TODO: get the static dynamic matrix A, B
A, B = continious_LIP_dynamics(g, h)

################################################################################
# problem definition
################################################################################

# Define an instance of MathematicalProgram 
prog = MathematicalProgram() 

################################################################################
# variables
nx = 4 #>>>>TODO: State dimension = ?
nu = 2 #>>>>TODO: control dimension = ?

state = prog.NewContinuousVariables(TOTAL_NO_SAMPLES, nx, 'state')
control = prog.NewContinuousVariables(TOTAL_NO_SAMPLES, nu, 'control')

# intial state
state_inital = np.array([x_0[0], x_0[1], y_0[0], y_0[1]]) 

# terminal state
state_terminal = np.array([foot_steps[-1,0], 0.0, foot_steps[-1,1], 0.0]) 

################################################################################
# constraints

# 1. intial constraint
#>>>>TODO: Add inital state constrain, Hint: prog.AddConstraint
prog.AddConstraint(eq(state[0,:], state_inital))

# 2. terminal constraint
#>>>>TODO: Add terminal state constrain, Hint: prog.AddConstraintf
prog.AddConstraint(eq(state[-1,:], state_terminal))

# 3. at each step: respect the LIP descretized dynamics
#>>>>TODO: Enforce the dynamics at every time step
for k in range(TOTAL_NO_SAMPLES - 1):
    prog.AddConstraint(eq(state[k+1,:], Ad.dot(state[k,:]) + Bd.dot(control[k,:])))

# 4. at each step: keep the ZMP within the foot sole (use the footprint and planned step position)
#>>>>TODO: Add ZMP upper and lower bound to keep the control (ZMP) within each footprints
#Hint: first compute upper and lower bound based on zmp_ref then add constraints.
#Hint: Add constraints at every time step
zmp_lb = zmp_ref - footprint/2
zmp_ub = zmp_ref + footprint/2
for k in range(TOTAL_NO_SAMPLES):
    prog.AddConstraint(control[k, 0] >= zmp_lb[k, 0])
    prog.AddConstraint(control[k, 0] <= zmp_ub[k, 0])
    prog.AddConstraint(control[k, 1] >= zmp_lb[k, 1])
    prog.AddConstraint(control[k, 1] <= zmp_ub[k, 1])


################################################################################
# stepwise cost, note that the cost function is scalar!

# setup our cost: minimize zmp error (tracking), minimize CoM velocity (smoothing)
#>>>>TODO: add the cost at each timestep, hint: prog.AddCost
for k in range(TOTAL_NO_SAMPLES):
    # ZMP error cost
    zmp_error = control[k,:] - zmp_ref[k,:]
    prog.AddCost(alpha * zmp_error.dot(zmp_error))
    
    # CoM velocity cost
    com_vel = state[k, [1,3]]
    prog.AddCost(gamma * com_vel.dot(com_vel))

################################################################################
# solve

result = Solve(prog)
if not result.is_success:
    print("failure")
print("solved")

# extract the solution
#>>>>TODO: extract your variables from the result object
state_sol = result.GetSolution(state)
control_sol = result.GetSolution(control)
t = T*np.arange(0, TOTAL_NO_SAMPLES)

# compute the acceleration
#>>>>TODO: compute the acceleration of the COM
com_acc_sol = np.zeros((TOTAL_NO_SAMPLES, 2))
for k in range(TOTAL_NO_SAMPLES):
    com_acc_sol[k,:] = (g/h) * (state_sol[k,[0,2]] - control_sol[k,:])

################################################################################
# plot something

#>>>>TODO: plot everything in x-axis
fig, ax = plt.subplots(4,1, sharex=True, figsize=(8,12))
ax[0].plot(t, state_sol[:,0], label='CoM Pos')
ax[0].plot(t, control_sol[:,0], label='ZMP Pos')
ax[0].plot(t, zmp_ref[:,0], '--', label='ZMP Ref')
ax[0].set_ylabel('position [m]')
ax[0].legend()
ax[1].plot(t, state_sol[:,1], label='CoM Vel')
ax[1].set_ylabel('velocity [m/s]')
ax[1].legend()
ax[2].plot(t, com_acc_sol[:,0], label='CoM Acc')
ax[2].set_ylabel('acceleration [m/s^2]')
ax[2].legend()
ax[3].plot(t, control_sol[:,0] - state_sol[:,0], label='ZMP-CoM Error')
ax[3].set_ylabel('error [m]')
ax[3].set_xlabel('time [s]')
ax[3].legend()
fig.suptitle('X-axis Trajectories')
fig.savefig('lipm_2ord_x_axis.png', dpi=300)

#>>>>TODO: plot everything in y-axis
fig, ax = plt.subplots(4,1, sharex=True, figsize=(8,12))
ax[0].plot(t, state_sol[:,2], label='CoM Pos')
ax[0].plot(t, control_sol[:,1], label='ZMP Pos')
ax[0].plot(t, zmp_ref[:,1], '--', label='ZMP Ref')
ax[0].plot(t, zmp_lb[:,1], '--', label='ZMP Lower Bound')
ax[0].plot(t, zmp_ub[:,1], '--', label='ZMP Upper Bound')
ax[0].set_ylabel('position [m]')
ax[0].legend()
ax[1].plot(t, state_sol[:,3], label='CoM Vel')
ax[1].set_ylabel('velocity [m/s]')
ax[1].legend()
ax[2].plot(t, com_acc_sol[:,1], label='CoM Acc')
ax[2].set_ylabel('acceleration [m/s^2]')
ax[2].legend()
ax[3].plot(t, control_sol[:,1] - state_sol[:,2], label='ZMP-CoM Error')
ax[3].set_ylabel('error [m]')
ax[3].set_xlabel('time [s]')
ax[3].legend()
fig.suptitle('Y-axis Trajectories')
fig.savefig('lipm_2ord_y_axis.png', dpi=300)


#>>>>TODO: plot everything in xy-plane
fig, ax = plt.subplots(figsize=(8, 8))
plot_foot_steps(foot_steps, footprint, ax)
ax.plot(zmp_ref[:,0], zmp_ref[:,1], '-o', label='ZMP Reference')
ax.plot(state_sol[:,0], state_sol[:,2], '-', label='CoM Trajectory')
ax.plot(control_sol[:,0], control_sol[:,1], '-', label='ZMP Trajectory')
ax.set_xlabel('x [m]')
ax.set_ylabel('y [m]')
ax.legend()
ax.set_title('XY Plane Trajectories')
ax.set_aspect('equal')
fig.savefig('lipm_2ord_xy_plane.png', dpi=300)
plt.show()
