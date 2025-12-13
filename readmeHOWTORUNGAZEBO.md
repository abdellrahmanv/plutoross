# How to Run Robot in Gazebo

This will launch the robot in a simulated hospital world using Gazebo physics.

### Terminal 1: Simulation (Gazebo + RViz)
This launches the Gazebo environment, spawns the robot, and opens RViz.

```powershell
wsl bash -c "cd ~/nero && source install/setup.bash && ros2 launch my_robot hospital_robot.launch.py"
```

### Terminal 2: Keyboard Teleop
Run this to control the robot's movement in the simulation.

```powershell
wsl bash -c "cd ~/nero && source install/setup.bash && ros2 run my_robot keyboard_teleop.py"
```

## Controls

- **Base Movement:**
  - `W` / `S` : Forward / Backward
  - `A` / `D` : Strafe Left / Right
  - `Q` / `E` : Rotate Left / Right

- **Arm Control:**
  - Note: Arm control is currently not connected to Gazebo physics, only base movement is active.
  - `SPACE` : Stop All
  - `Ctrl+C` : Quit
