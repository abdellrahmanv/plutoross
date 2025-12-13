# How to Run Simple Movement

You will need two separate terminals to run the robot simulation and the controller.

### Terminal 1: Robot & Controller
This launches the robot visualization, the state publisher, and the main controller.

```powershell
wsl bash -c "cd ~/nero && source install/setup.bash && ros2 launch my_robot mobile_base.launch.py"
```

### Terminal 2: Keyboard Teleop
Run this to control the robot using your keyboard.

```powershell
wsl bash -c "cd ~/nero && source install/setup.bash && ros2 run my_robot keyboard_teleop.py"
```

## Controls

- **Base Movement:**
  - `W` / `S` : Forward / Backward
  - `A` / `D` : Strafe Left / Right
  - `Q` / `E` : Rotate Left / Right

- **Arm Control:**
  - `U` / `J` : Left Arm Up / Down
  - `I` / `K` : Right Arm Up / Down

- **General:**
  - `SPACE` : Stop All
  - `Ctrl+C` : Quit
