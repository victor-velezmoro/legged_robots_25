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

## 📝 Manual Script Execution
If a script is not a ROS 2 node, you will need to launch it manually using standard Python execution methods (e.g., `python path/to/your/script.py`).

## ⚠️ Important Notes

*   **Interactive Marker Timing:** When testing the interactive marker, please **wait** until the `JointController` finishes its task (i.e., reaches its goal position) **before** attempting to move the cube.
