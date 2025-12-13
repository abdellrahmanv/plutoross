# MECANUM WHEEL CONTROL - Quick Reference

## System is Running! ✅

Your robot now has mecanum wheel control for omnidirectional movement.

## Test Movement (No Keyboard Needed)

**Forward:**
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

**Backward:**
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{linear: {x: -0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

**Strafe Left:**
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.5, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

**Strafe Right:**
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: -0.5, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

**Rotate CCW:**
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"
```

**Rotate CW:**
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.0}}"
```

**Diagonal (Forward-Left):**
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5, y: 0.5, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

**Stop:**
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## Watch the wheels in RViz!

The mecanum controller converts your velocity commands into individual wheel rotations. Watch them spin in RViz2!

## Mecanum Wheel Kinematics (No-Slip)

```
Front-Left  = Vx - Vy - Wz * L
Front-Right = Vx + Vy + Wz * L  
Back-Left   = Vx + Vy - Wz * L
Back-Right  = Vx - Vy + Wz * L

Where:
- Vx = forward/backward velocity
- Vy = left/right strafe velocity  
- Wz = rotational velocity
- L = robot diagonal dimension
```

## Keyboard Control (Optional)

In a new terminal:
```bash
cd ~/nero
source install/setup.bash
ros2 run my_robot keyboard_teleop.py
```

Controls: W/S (forward/back), A/D (strafe), Q/E (rotate), SPACE (stop)
