import numpy as np
import pinocchio as pin
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt

# import ndcurves, scipy, numpy, etc... to do your splines

class SwingFootTrajectory:
    """SwingFootTrajectory
    Interpolate Foot trajectory between SE3 T0 and T1
    """
    def __init__(self, T0, T1, duration, height=0.05):
        """initialize SwingFootTrajectory

        Args:
            T0 (pin.SE3): Inital foot pose
            T1 (pin.SE3): Final foot pose
            duration (float): step duration
            height (float, optional): setp height. Defaults to 0.05.
        """
        self._height = height
        self._t_elapsed = 0.0
        self._duration = duration
        self.reset(T0, T1)

    def reset(self, T0, T1):
        '''reset back to zero, update poses
        '''
        self._T0 = T0
        self._T1 = T1
        self._t_elapsed = 0.0
        
        p0 = T0.translation
        p1 = T1.translation
        
        t_points = np.array([0.0, self._duration/4, self._duration/2, 3*self._duration/4, self._duration])
        x_position = np.linspace(p0[0], p1[0], 5)
        y_position = np.linspace(p0[1], p1[1], 5)
        z_position = np.array([p0[2], p0[2] + self._height/2, p0[2] + self._height, p1[2] + self._height/2, p1[2]])

        self._spline_x = CubicSpline(t_points, x_position, bc_type='clamped')
        self._spline_y = CubicSpline(t_points, y_position, bc_type='clamped')
        self._spline_z = CubicSpline(t_points, z_position, bc_type='clamped')
        
        self._RO = T0.rotation
        self._R1 = T1.rotation

    def isDone(self):
        return self._t_elapsed >= self._duration 
    
    def evaluate(self, t):
        """evaluate at time t
        """
        t = max(0.0, min(t, self._duration))
        self._t_elapsed = t
        
        # Evaluate position and derivatives
        pos = np.array([
            self._spline_x(t),
            self._spline_y(t),
            self._spline_z(t)
        ])
        
        vel = np.array([
            self._spline_x(t, 1),
            self._spline_y(t, 1),
            self._spline_z(t, 1)
        ])
        
        acc = np.array([
            self._spline_x(t, 2),
            self._spline_y(t, 2),
            self._spline_z(t, 2)
        ])
        
        alpha = t / self._duration
        # Simple rotation interpolation (could be improved with proper SLERP)
        R = self._RO @ pin.exp3(alpha * pin.log3(self._RO.T @ self._R1))

        # Create SE3 pose
        pose = pin.SE3(R, pos)
        
        return pose, vel, acc

if __name__=="__main__":
    T0 = pin.SE3(np.eye(3), np.array([0, 0, 0]))
    T1 = pin.SE3(np.eye(3), np.array([0.2, 0, 0]))

    #create trajectory 
    duration = 1.0
    height = 0.05
    trajectory = SwingFootTrajectory(T0, T1, duration, height)
    
    #Plots 
    t_plots = np.linspace(0, duration, 100)
    positions = []
    velocities = []
    accelerations = []
    
    for t in t_plots:
        pose, vel, acc = trajectory.evaluate(t)
        positions.append(pose.translation)
        velocities.append(vel)
        accelerations.append(acc)
    
    positions = np.array(positions)
    velocities = np.array(velocities)
    accelerations = np.array(accelerations)
    
    fig, axes = plt.subplots(3, 3, figsize=(15, 1))
    
    axes[0, 0].plot(t_plots, positions[:, 0], label='X Position')
    axes[0, 0].set_title('X Position')
    axes[0,0].set_ylabel('Position in (m)')
    
    axes[0, 1].plot(t_plots, positions[:, 1], label='Y Position')
    axes[0, 1].set_title('Y Position')
   
    axes[0, 2].plot(t_plots, positions[:, 2], label='Z Position')
    axes[0, 2].set_title('Z Position')
    
    axes[1, 0].plot(t_plots, velocities[:, 0], label='X Velocity')
    axes[1, 0].set_title('X Velocity')
    axes[1,0].set_ylabel('Velocity in (m/s)')
    
    axes[1, 1].plot(t_plots, velocities[:, 1], label='Y Velocity')
    axes[1, 1].set_title('Y Velocity')
    
    axes[1, 2].plot(t_plots, velocities[:, 2], label='Z Velocity')
    axes[1, 2].set_title('Z Velocity')
    
    axes[2, 0].plot(t_plots, accelerations[:, 0], label='X Acceleration')
    axes[2, 0].set_title('X Acceleration')
    axes[2,0].set_ylabel('Acceleration in (m/s^2)')
    
    axes[2, 1].plot(t_plots, accelerations[:, 1], label='Y Acceleration')
    axes[2, 1].set_title('Y Acceleration')
    
    axes[2, 2].plot(t_plots, accelerations[:, 2], label='Z Acceleration')
    axes[2, 2].set_title('Z Acceleration')
    
    plt.tight_layout()
    plt.show()
    plt.savefig("foot_trajectory.png")
    
    # Print boundary conditions to verify
    print("Boundary conditions verification:")
    pose0, vel0, acc0 = trajectory.evaluate(0.0)
    pose1, vel1, acc1 = trajectory.evaluate(duration)
    
    print(f"Initial velocity: {vel0}")
    print(f"Initial acceleration: {acc0}")
    print(f"Final velocity: {vel1}")
    print(f"Final acceleration: {acc1}")
    print(f"Peak height at t={duration/2}: {trajectory.evaluate(duration/2)[0].translation[2]}")
    
    
