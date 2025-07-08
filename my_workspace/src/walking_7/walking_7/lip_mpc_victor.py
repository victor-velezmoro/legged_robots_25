import numpy as np

from pydrake.all import MathematicalProgram, Solve, eq


################################################################################
# Helper fnc
################################################################################


def continious_LIP_dynamics(g, h):
    """returns the static matrices A,B of the continious LIP dynamics

    Args:
        g (_type_): gravity
        h (_type_): height

    Returns:
        np.array: A, B
    """

    #>>>>TODO: copy from previous Task 2
    A = np.array([[0,   1,   0,   0],
                  [g/h, 0,   0,   0],
                  [0,   0,   0,   1],
                  [0,   0,   g/h, 0]])
    
    B = np.array([[0,    0],
                  [-g/h, 0],
                  [0,    0],
                  [0,    -g/h]])
    
    return A, B

def discrete_LIP_dynamics( g, h, dt):
    """returns the matrices static Ad,Bd of the discretized LIP dynamics

    Args:
        dt (_type_): discretization steps
        g (_type_): gravity
        h (_type_): height

    Returns:
        _type_: _description_
    """
    #>>>>TODO: copy from previous Task 2
    omega = np.sqrt(g/h)
    Ad_xy = np.array([[np.cosh(omega*dt), np.sinh(omega*dt)/omega],
                   [omega*np.sinh(omega*dt), np.cosh(omega*dt)]])
    Bd_xy = np.array([1-np.cosh(omega*dt), -omega *np.sinh(omega*dt)])

    Ad = np.zeros((4, 4))
    Ad[0:2, 0:2] = Ad_xy
    Ad[2:4, 2:4] = Ad_xy
    
    Bd = np.zeros((4, 2))
    Bd[0:2, 0] = Bd_xy.flatten()
    Bd[2:4, 1] = Bd_xy.flatten()
    
    return Ad, Bd

################################################################################
# LIPInterpolator
################################################################################

class LIPInterpolator:
    """Integrates the linear inverted pendulum model using the 
    continous dynamics. To interpolate the solution to hight 
    """
    def __init__(self, x_inital, conf):
        self.conf = conf
        self.dt = conf.dt
        self.x = x_inital
        #>>>>TODO: Finish
        self.g = conf.g
        self.h = conf.h
        self.A, self.B = continious_LIP_dynamics(self.g, self.h)
        
    def integrate(self, u):
        #>>>>TODO: integrate with dt
        x_dot = self.A @ self.x + self.B @ u
        self.x = self.x + self.dt * x_dot

        return self.x

    def comState(self):
        #>>>>TODO: return the center of mass state
        # that is position \in R3, velocity \in R3, acceleration \in R3
        c, c_dot, c_ddot = None, None, None
        c = np.array([self.x[0], self.x[2], self.h])
        c_dot = np.array([self.x[1], self.x[3], 0])
        c_ddot = np.array([0, 0, 0])

        return c, c_dot, c_ddot
    
    def dcm(self):
        #>>>>TODO: return the computed dcm
        omega = np.sqrt(self.g / self.h)
    
        # Get 2D CoM position and velocity
        c_2d = np.array([self.x[0], self.x[2]])    # [cx, cy]
        c_dot_2d = np.array([self.x[1], self.x[3]])  # [vx, vy]
        
        dcm = c_2d + c_dot_2d / omega
        return dcm
    
    def zmp(self):
        # Get 2D CoM position and velocity
        c_x, v_x, c_y, v_y = self.x
        p_x = c_x - (self.h / self.g) * (self.A[1, 0] * c_x + self.A[1, 1] * v_x)
        p_y = c_y - (self.h / self.g) * (self.A[3, 2] * c_y + self.A[3, 3] * v_y)
        zmp = np.array([p_x, p_y])
        return zmp
        
    
################################################################################
# LIPMPC
################################################################################

class LIPMPC:
    def __init__(self, conf):
        self.conf = conf
        self.dt = conf.dt        
        self.no_samples = conf.no_mpc_samples_per_horizon
        
        # solution and references over the horizon
        self.X_k = None
        self.U_k = None
        self.ZMP_ref_k = None
        
    def buildSolveOCP(self, x_k, ZMP_ref_k, terminal_idx):
        """build and solve ocp

        Args:
            x_k (_type_): inital mpc state
            ZMP_ref_k (_type_): zmp reference over horizon
            terminal_idx (_type_): index within horizon to apply terminal constraint

        Returns:
            _type_: control
        """
        
        #>>>>TODO: build and solve the ocp
        #>>>>Note: start without terminal constraints
        prog = MathematicalProgram()
        nx = 4
        nu = 2
        
        state = prog.NewContinuousVariables(self.no_samples, nx, 'state')
        control = prog.NewContinuousVariables(self.no_samples, nu, 'control')
        
        prog.AddConstraint(state[0, :] == x_k)
        
        Ad, Bd = discrete_LIP_dynamics(self.conf.g, self.conf.h, self.dt)
        for k in range(self.no_samples - 1):
            prog.AddConstraint(state[k + 1, :] == Ad @ state[k, :] + Bd @ control[k, :])
            
        if terminal_idx < self.no_samples:
            terminal_state = np.array([ZMP_ref_k[terminal_idx, 0], 0, ZMP_ref_k[terminal_idx, 1], 0])
            for k_term in range(terminal_idx, self.no_samples):
                prog.AddConstraint(eq(state[k_term, :], terminal_state))
                
        # Cost function
        alpha = self.conf.alpha  # ZMP error cost weight
        gamma = self.conf.gamma  # CoM velocity cost weight
        for k in range(self.no_samples):
            # ZMP tracking cost
            zmp_error = control[k, :] - ZMP_ref_k[k, :]
            prog.AddCost(alpha * zmp_error.dot(zmp_error))
            
            # CoM velocity smoothing cost
            com_vel = state[k, [1, 3]]
            prog.AddCost(gamma * com_vel.dot(com_vel))

        # Solve the OCP
        result = Solve(prog)
        if not result.is_success():
            print("MPC optimization failed.")
            # Return last known good control or a zero control
            return self.U_k[0] if self.U_k is not None else np.zeros(nu)

        self.X_k = result.GetSolution(state)
        self.U_k = result.GetSolution(control)
        self.ZMP_ref_k = ZMP_ref_k
        
        return self.U_k[0]



def generate_zmp_reference(foot_steps, no_samples_per_step):
    """generate a function that computes a referecne trajecotry for the zmp.
    Our goal is to keep the ZMP at the footstep center within each step

    Args:
        foot_steps (_type_): _description_
        no_samples_per_step (_type_): _description_
    """
    zmp_ref_list = []

    for foot in foot_steps:
        pose = foot.poseInWorld()
        x = pose.translation[0]
        y = pose.translation[1]
        zmp_step = np.tile([x, y], (no_samples_per_step, 1))
        zmp_ref_list.append(zmp_step)

    zmp_ref = np.vstack(zmp_ref_list)

    return zmp_ref
