# 🤖 Robot Simulation Workspace ReadMe

Welcome! This workspace contains launch files to start visualization nodes for robot simulations. Please select the appropriate launch file for your needs.

## 👁️ `ros_visuals` Package

This package handles visualization nodes.

### `launch.py`
Use this general launch file to start various visualization nodes for the robot simulation. You'll need to select the specific node you want to launch within the file.

**🚀 Launch Command:**
```bash
ros2 launch ros_visuals launch.py
```

### `talos.launch.py`
This specific launch file starts the robot state publisher and the RViz2 node for the Talos robot.

**🚀 Launch Command:**
```bash
ros2 launch ros_visuals talos.launch.py
```

## 🎯 `bullet_sims` Package

This package contains launch files and scripts related to Bullet physics simulations, primarily from Tutorials 2 and 3.

### `launch.py`
Use this launch file to start visualization nodes for Bullet simulations. Remember to select the correct file within this launch script.

**🚀 Launch Command:**
```bash
ros2 launch bullet_sims launch.py
```

## 🤖 `tutorial_4` Package - Tutorial 4 Content

This package contains basic control implementations for the Talos robot demonstrating fundamental balancing and movement capabilities.

### `01_standing.py` - Basic Standing Control
Implements basic standing control where the robot maintains its upright posture and balance.

**🚀 Launch Command:**
```bash
ros2 run tutorial_4 01_standing
```

### `02_one_leg_stand.py` - Single Leg Balance
Demonstrates advanced balance control where the robot can balance on one leg, testing the stability and control algorithms.

**🚀 Launch Command:**
```bash
ros2 run tutorial_4 02_one_leg_stand
```

### `03_squating.py` - Squatting with Arm Movement
Implements coordinated movement where the robot can squat on one leg while simultaneously performing circular motions with the right hand, demonstrating multi-task control coordination.

**🚀 Launch Command:**
```bash
ros2 run tutorial_4 03_squating
```

## 🏃‍♂️ `tutorial_4` Package - Tutorial 5 Content

This package contains advanced control implementations for the Talos robot, including ankle and hip controllers with push disturbance testing.

### `t51.py` - Torque Control with Push Testing
Implements ankle and hip controllers using **torque control** along with a push state machine for disturbance testing. The system applies sequential push forces (RIGHT → LEFT → BACK) to test the robot's balance control capabilities.

**Features:**
- Ankle controller for CoM position regulation
- Hip controller for angular momentum control  
- Push state machine with configurable timing
- Real-time data recording and visualization
- Force visualization in PyBullet

**🚀 Launch Command:**
```bash
ros2 run tutorial_4 t51
```

### `t52.py` - Position Control with Push Testing
Similar to t51.py but implements the same ankle and hip controllers using **position control** instead of torque control. This allows comparison between different control strategies under the same disturbance conditions.

**🚀 Launch Command:**
```bash
ros2 run tutorial_4 t52
```

### 📊 Generated Plots
Both scripts automatically generate analysis plots showing:
- Center of Mass (CoM) trajectories
- Zero Moment Point (ZMP) evolution
- Centroidal Moment Pivot (CMP) tracking
- Capture Point/Divergent Component of Motion (CP/DCM) behavior

**📁 Plot Location:**
```
/workspaces/ros2_ws/my_workspace/src/tutorial_4/plots/
```

Plots are saved in both PNG and PDF formats with smoothed data visualization for better analysis.

## 📝 Manual Script Execution
If a script is not a ROS 2 node, you will need to launch it manually using standard Python execution methods (e.g., `python path/to/your/script.py`).

## ⚠️ Important Notes

*   **Interactive Marker Timing:** When testing the interactive marker, please **wait** until the `JointController` finishes its task (i.e., reaches its goal position) **before** attempting to move the cube.
*   **Push Testing:** The push sequence automatically starts after initialization. Wait for the "Push sequence started" message before expecting disturbances.
*   **Data Recording:** Both t51.py and t52.py automatically record simulation data and generate analysis plots upon completion (Ctrl+C to stop and save).
