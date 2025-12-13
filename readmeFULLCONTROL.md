# Full Robot Control (RViz + Keyboard)

This mode allows you to control **both** the mecanum wheels and the robot arms using your keyboard, visualized in RViz.

## 1. Launch the Robot & Controller
This starts RViz and the controller that listens for your keyboard commands.

```bash
wsl bash -c "source /opt/ros/humble/setup.bash; source /home/nero/nero/install/setup.bash; ros2 launch my_robot mobile_base.launch.py"
```

## 2. Start Keyboard Control
Open a **new terminal** and run this to send commands.

```bash
wsl bash -c "source /opt/ros/humble/setup.bash; source /home/nero/nero/install/setup.bash; ros2 run my_robot keyboard_teleop.py"
```

### Controls
**Base (Wheels):**
- **W / S**: Forward / Backward
- **A / D**: Strafe Left / Right
- **Q / E**: Rotate Left / Right

**Arms:**
- **U / J**: Left Arm Up / Down
- **I / K**: Right Arm Up / Down

**General:**
- **Space**: Stop All
