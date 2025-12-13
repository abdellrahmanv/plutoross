# Mecanum Robot Control Project

This project simulates a mecanum-wheeled robot with two arms in ROS 2 Humble.

## 🚀 Quick Start

### 1. Build the Project
If you haven't built it recently:
```powershell
wsl bash -c "cd ~/nero && colcon build --packages-select my_robot --symlink-install"
```

### 2. Launch the Robot (Terminal 1)
This starts the robot simulation, controller, and RViz.
```powershell
wsl bash -c "cd ~/nero && source install/setup.bash && ros2 launch my_robot mobile_base.launch.py"
```

### 3. Control the Robot (Terminal 2)
Open a **new terminal** to run the keyboard teleop node.
```powershell
wsl bash -c "cd ~/nero && source install/setup.bash && ros2 run my_robot keyboard_teleop.py"
```

## 🎮 Controls
(Case-insensitive)

**Base Movement:**
- **W** : Move Forward
- **S** : Move Backward
- **A** : Strafe Left
- **D** : Strafe Right
- **Q** : Rotate Left (CCW)
- **E** : Rotate Right (CW)

**Arm Control:**
- **U / J** : Left Arm Up / Down
- **I / K** : Right Arm Up / Down

**General:**
- **SPACE** : Stop all movement
- **Ctrl+C** : Quit
