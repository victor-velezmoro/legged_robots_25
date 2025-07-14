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

## 🤖 `tut_4_5` Package - Tutorial 4 Content

This package contains basic control implementations for the Talos robot demonstrating fundamental balancing and movement capabilities.

### `01_standing.py` - Basic Standing Control
Implements basic standing control where the robot maintains its upright posture and balance.

**🚀 Launch Command:**
```bash
ros2 run tut_4_5 01_standing
```

### `02_one_leg_stand.py` - Single Leg Balance
Demonstrates advanced balance control where the robot can balance on one leg, testing the stability and control algorithms.

**🚀 Launch Command:**
```bash
ros2 run tut_4_5 02_one_leg_stand
```

### `03_squating.py` - Squatting with Arm Movement
Implements coordinated movement where the robot can squat on one leg while simultaneously performing circular motions with the right hand, demonstrating multi-task control coordination.

**🚀 Launch Command:**
```bash
ros2 run tut_4_5 03_squating
```

## 🏃‍♂️ `tut_4_5` Package - Tutorial 5 Content

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
ros2 run tut_4_5 t51
```

### `t52.py` - Position Control with Push Testing
Similar to t51.py but implements the same ankle and hip controllers using **position control** instead of torque control. This allows comparison between different control strategies under the same disturbance conditions.

**🚀 Launch Command:**
```bash
ros2 run tut_4_5 t52
```

### 📊 Generated Plots
Both scripts automatically generate analysis plots showing:
- Center of Mass (CoM) trajectories
- Zero Moment Point (ZMP) evolution
- Centroidal Moment Pivot (CMP) tracking
- Capture Point/Divergent Component of Motion (CP/DCM) behavior

**📁 Plot Location:**
```
/workspaces/ros2_ws/my_workspace/src/tut_4_5/plots/
```

Plots are saved in both PNG and PDF formats with smoothed data visualization for better analysis.

## 🚶‍♂️ `tut_6` Package - Tutorial 6 Content

This package contains advanced walking control implementations using the Linear Inverted Pendulum Model (LIPM) with both optimal control and model predictive control approaches for trajectory planning and gait generation.

### `ocp_lipm_2ord.py` - Optimal Control Problem for LIPM
Implements trajectory planning for the Linear Inverted Pendulum Model using an Optimal Control Problem (OCP) formulation. This approach solves the entire footstep plan in one optimization over the complete horizon.

**Features:**
- Linear Inverted Pendulum Model with 2D state [cx, vx, cy, vy]
- ZMP (Zero Moment Point) control [px, py]
- Complete footstep trajectory planning
- Footstep generation with alternating left/right pattern
- ZMP tracking with footprint constraints
- CoM velocity smoothing
- Comprehensive visualization (X-axis, Y-axis, XY-plane trajectories)

**🚀 Launch Command:**
```bash
ros2 run tut_6 ocp
```

### `mpc_lipm_2ord.py` - Model Predictive Control for LIPM
Implements the same Linear Inverted Pendulum Model but using Model Predictive Control (MPC) with a receding horizon approach. The system solves trajectory planning multiple times over shorter horizons of 2 steps.

**Features:**
- Receding horizon control with 2-step lookahead
- Real-time MPC updates at 10Hz (T_MPC = 0.1s)
- High-frequency simulation at 200Hz (T_SIM = 0.005s)
- Push disturbance testing for robustness evaluation
- Dynamic footstep reference tracking
- Terminal state constraints for stability
- Real-time visualization and data recording

**🚀 Launch Command:**
```bash
ros2 run tut_6 mpc
```

### 📊 Generated Plots
Both scripts automatically generate comprehensive analysis plots including:
- **X-axis trajectories**: CoM position, ZMP tracking, ZMP bounds, velocity and acceleration
- **Y-axis trajectories**: Same metrics as X-axis for lateral movement
- **XY-plane view**: Footstep plan, ZMP reference, CoM trajectory, and ZMP trajectory
- **Disturbance analysis**: Push effects visualization (MPC only)

**📁 Plot Location:**
```
/workspaces/ros2_ws/my_workspace/src/tut_6/plots/
```

Plots are saved in PNG format with high resolution (300 DPI) for detailed analysis and comparison between OCP and MPC approaches.


## 🚶‍♂️  Tutorial 7

**This work was done in collaboration with Timo Class and can be found in his repository/delivery.**









### 🎯 Key Differences: OCP vs MPC
- **OCP**: Solves the complete walking trajectory in one optimization, optimal but computationally intensive
- **MPC**: Uses receding horizon with frequent re-optimization, more robust to disturbances and real-time capable




## 📝 Manual Script Execution
If a script is not a ROS 2 node, you will need to launch it manually using standard Python execution methods (e.g., `python path/to/your/script.py`).

## ⚠️ Important Notes

*   **Interactive Marker Timing:** When testing the interactive marker, please **wait** until the `JointController` finishes its task (i.e., reaches its goal position) **before** attempting to move the cube.
*   **Push Testing:** The push sequence automatically starts after initialization. Wait for the "Push sequence started" message before expecting disturbances.
*   **Data Recording:** Both t51.py and t52.py automatically record simulation data and generate analysis plots upon completion (Ctrl+C to stop and save).
