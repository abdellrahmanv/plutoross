# How to Run NERO Robot Simulation

## Launch Commands (from PowerShell)

### Hospital Environment (Default - 40x40m with reception area)
```powershell
wsl bash -c "cd ~/nero/my_robot && source /opt/ros/humble/setup.bash ; source install/setup.bash ; ros2 launch my_robot mapping.launch.py"
```

### Warehouse Environment (60x50m with storage zones)
```powershell
wsl bash -c "cd ~/nero/my_robot && source /opt/ros/humble/setup.bash ; source install/setup.bash ; ros2 launch my_robot mapping.launch.py world:=/home/nero/nero/my_robot/worlds/different_environments/warehouse_realistic.world"
```

### Office Environment (30x30m with cubicles)
```powershell
wsl bash -c "cd ~/nero/my_robot && source /opt/ros/humble/setup.bash ; source install/setup.bash ; ros2 launch my_robot mapping.launch.py world:=/home/nero/nero/my_robot/worlds/different_environments/office_realistic.world"
```

## Control Commands (from PowerShell)

### Keyboard Control (Terminal 2)
```powershell
wsl bash -c "cd ~/nero/my_robot && source /opt/ros/humble/setup.bash ; source install/setup.bash ; ros2 run my_robot keyboard_teleop.py"
```

Controls:
- W/A/S/D - Move forward/left/backward/right
- Q/E - Rotate left/right
- U/J - Left arm up/down
- I/K - Right arm up/down
- Space - Stop

### View Camera Feed (Terminal 3)
```powershell
wsl bash -c "source /opt/ros/humble/setup.bash ; ros2 run rqt_image_view rqt_image_view /camera/image_raw"
```

## What Launches

When you run mapping.launch.py:
- ✅ Gazebo with selected world
- ✅ Robot with lidar + camera (no YOLO)
- ✅ RViz for visualization
- ✅ SLAM Toolbox for mapping
- ✅ Keyboard control ready

## Your Custom Environments

All environments are in: `~/nero/my_robot/worlds/different_environments/`

1. **hospital_realistic.world** - 40x40m hospital with reception area
2. **warehouse_realistic.world** - 60x50m warehouse with storage zones  
3. **office_realistic.world** - 30x30m office with cubicles

Each has:
- Custom floor plans with walls
- Static models (no random downloads)
- Proper collision geometry
- Person obstacles for navigation testing
