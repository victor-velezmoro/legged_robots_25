import numpy as np
import pinocchio as pin
from enum import Enum

class Side(Enum):
    """Side
    Describes which foot to use
    """
    LEFT=0
    RIGHT=1

def other_foot_id(id):
    if id == Side.LEFT:
        return Side.RIGHT
    else:
        return Side.LEFT
        
class FootStep:
    """FootStep
    Holds all information describing a single footstep
    """
    def __init__(self, pose, footprint, side=Side.LEFT):
        """inti FootStep

        Args:
            pose (pin.SE3): the pose of the footstep
            footprint (np.array): 3 by n matrix of foot vertices
            side (_type_, optional): Foot identifier. Defaults to Side.LEFT.
        """
        self.pose = pose
        self.footprint = footprint
        self.side = side
        
    def poseInWorld(self):
        return self.pose

    def plot(self, simulator):

        # Get the position and rotation from the pose
        position = self.pose.translation
        rotation_matrix = self.pose.rotation
        
        # Transform footprint vertices to world coordinates
        world_footprint = rotation_matrix @ self.footprint + position.reshape(3, 1)
        
        # Plot footprint as a rectangle using debug lines
        for i in range(len(world_footprint[0]) - 1):
            p1 = [world_footprint[0, i], world_footprint[1, i], world_footprint[2, i]]
            p2 = [world_footprint[0, i+1], world_footprint[1, i+1], world_footprint[2, i+1]]
            
            # Color based on side: red for left, blue for right
            color = [1, 0, 0] if self.side == Side.LEFT else [0, 0, 1]
            simulator.addGlobalDebugLine(p1, p2, color=color, lineWidth=3)

        # Display the side of the step with text
        side_text = "L" if self.side == Side.LEFT else "R"
        text_position = [position[0], position[1], position[2] + 0.05]
        text_color = [1, 0, 0] if self.side == Side.LEFT else [0, 0, 1]
        #simulator.addUserDebugText(side_text, text_position, text_color, 1.5)

        # Plot step target position as a sphere marker
        sphere_color = [1, 0, 0, 0.5] if self.side == Side.LEFT else [0, 0, 1, 0.5]
        simulator.addSphereMarker(position, radius=0.02, color=sphere_color)

        return None

class FootStepPlanner:
    """FootStepPlanner
    Creates footstep plans (list of right and left steps)
    """
    
    def __init__(self, conf):
        self.conf = conf
        self.steps = []
        
    def planLine(self, T_0_w, side, no_steps):
        """plan a sequence of steps in a strait line

        Args:
            T_0_w (pin.SE3): The inital starting position of the plan
            side (Side): The intial foot for starting the plan
            no_steps (_type_): The number of steps to take

        Returns:
            list: sequence of steps
        """
        
        # the displacement between steps in x and y direction
        dx = self.conf.step_size_x
        dy = 2*self.conf.step_size_y
        
        # the footprint of the robot
        lfxp, lfxn = self.conf.lfxp, self.conf.lfxn
        lfyp, lfyn = self.conf.lfyp, self.conf.lfyn
        
        # Create footprint vertices (rectangle)
        footprint = np.array([
            [lfxp, lfxp, lfxn, lfxn, lfxp],  # x coordinates
            [lfyp, lfyn, lfyn, lfyp, lfyp],  # y coordinates
            [0, 0, 0, 0, 0]                   # z coordinates
        ])
        
        steps = []
        
        if no_steps < 1:
            self.steps = steps
            return steps
        
        current_side = side
        
        # Step 1: First foot at initial position
        steps.append(FootStep(T_0_w, footprint, current_side))
        
        if no_steps == 1:
            self.steps = steps
            return steps
        
        # Step 2: Other foot parallel to first (both feet on ground)
        other_side = other_foot_id(current_side)
        if current_side == Side.LEFT:
            # If first step is left foot, right foot goes to the right
            parallel_translation = pin.SE3(np.eye(3), np.array([0, -dy/2, 0]))
        else:
            # If first step is right foot, left foot goes to the left  
            parallel_translation = pin.SE3(np.eye(3), np.array([0, dy/2, 0]))
        
        parallel_pose = T_0_w * parallel_translation
        steps.append(FootStep(parallel_pose, footprint, other_side))
        
        if no_steps == 2:
            self.steps = steps
            return steps
        
        # Steps 3 to no_steps-1: Alternating walking steps
        current_side = side  # Start alternating from original side
        for i in range(2, no_steps - 1):
            step_number = i - 1  # Steps 1, 2, 3... in walking sequence
            x_offset = step_number * dx
            
            if current_side == Side.LEFT:
                y_offset = dy/2  # Left foot offset
            else:
                y_offset = -dy/2  # Right foot offset
            
            translation = pin.SE3(np.eye(3), np.array([x_offset, y_offset, 0]))
            step_pose = T_0_w * translation
            steps.append(FootStep(step_pose, footprint, current_side))
            
            current_side = other_foot_id(current_side)
        
        # Final step (step no_steps): Parallel to second-to-last
        final_side = current_side
        last_step_pose = steps[-1].pose
        
        if final_side == Side.LEFT:
            final_translation = pin.SE3(np.eye(3), np.array([0, dy, 0]))
        else:
            final_translation = pin.SE3(np.eye(3), np.array([0, -dy, 0]))
        
        final_pose = last_step_pose * final_translation
        steps.append(FootStep(final_pose, footprint, final_side))
                                
        self.steps = steps
        return steps

    
    def plot(self, simulation):
        for step in self.steps:
            step.plot(simulation)

            
if __name__=='__main__':
    """ Test footstep planner
    """
    
    # Import the pybullet wrapper
    import sys
    import os
    sys.path.append('/workspaces/ros2_ws/my_workspace/src/simulator/simulator')
    from pybullet_wrapper import PybulletWrapper
    import time
    
    # Simple configuration class for testing
    class TestConfig:
        def __init__(self):
            self.step_size_x = 0.2  # 20cm forward steps
            self.step_size_y = 0.1  # 10cm lateral separation (total 20cm between feet)
            self.lfxp = 0.1   # foot length forward
            self.lfxn = -0.05 # foot length backward  
            self.lfyp = 0.05  # foot width left
            self.lfyn = -0.05 # foot width right
    
    # Create simulation environment
    simulator = PybulletWrapper(render=True)
    
    # Create configuration and planner
    config = TestConfig()
    planner = FootStepPlanner(config)
    
    # Create initial pose (identity transformation at origin, lifted slightly off ground)
    initial_pose = pin.SE3(np.eye(3), np.array([0, 0, 0.01]))
    
    # Test different scenarios
    print("Testing footstep planner...")
    
    # Test 1: Generate a plan with 8 steps starting with left foot
    print("\n=== Test 1: 8 steps starting with LEFT foot ===")
    steps = planner.planLine(initial_pose, Side.LEFT, 8)
    
    print(f"Generated {len(steps)} steps:")
    for i, step in enumerate(steps):
        pos = step.pose.translation
        side_str = "LEFT" if step.side == Side.LEFT else "RIGHT"
        print(f"Step {i+1}: {side_str} foot at ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
    
    # Plot the footsteps
    planner.plot(simulator)
    
    print("\nFootstep plan visualization loaded in PyBullet.")
    print("Check that the plan looks as expected:")
    print("- Steps should alternate between left (red) and right (blue)")
    print("- Robot starts and ends with both feet on ground")
    print("- Steps move forward progressively")
    print("- Press SPACE to pause/resume, 'q' to quit")
    
    # Run simulation loop
    try:
        while simulator.isRunning():
            simulator.step()
            simulator.debug()
            # Add a small delay for visualization
            if simulator.isRendering():
                time.sleep(0.01)
                
    except KeyboardInterrupt:
        print("\nShutting down simulation...")
    
    simulator.disconnect()
    print("Test completed!")