# Custom Robot in Hospital Environment - Final Setup

## Quick Start Commands (Windows CMD / PowerShell)

Run these commands directly in your Windows terminal (CMD or PowerShell).
**Note:** We explicitly source ROS 2 (`source /opt/ros/humble/setup.bash`) in every command to ensure `ros2` is found.
**Note:** These commands use `export VAR=` (empty) instead of `export VAR=""` to avoid quoting issues in Windows.

### 1. RViz Only (Test Robot Visualization)
```cmd
wsl bash -c "source /opt/ros/humble/setup.bash && cd ~/nero && source install/setup.bash && ros2 launch my_robot display.launch.py"
```

### 2. Gazebo + Robot + RViz (Full Simulation)
```cmd
wsl bash -c "source /opt/ros/humble/setup.bash && cd ~/nero && source install/setup.bash && export GAZEBO_MODEL_DATABASE_URI= && export GAZEBO_IP=127.0.0.1 && export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:/home/nero/nero/install/my_robot/share && ros2 launch my_robot gazebo_3rooms.launch.py"
```

### 3. Mapping (SLAM) - Create a Map of the World

**Terminal 1: Launch Simulation & SLAM**
```cmd
wsl bash -c "source /opt/ros/humble/setup.bash && cd ~/nero && source install/setup.bash && export GAZEBO_MODEL_DATABASE_URI= && export GAZEBO_IP=127.0.0.1 && export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:/home/nero/nero/install/my_robot/share && ros2 launch my_robot mapping.launch.py"
```

**Terminal 2: Drive the Robot**
```cmd
wsl bash -c "source /opt/ros/humble/setup.bash && cd ~/nero && source install/setup.bash && ros2 run my_robot keyboard_teleop.py"
```
*Controls: i=forward, j=left, l=right, ,=backward, k=stop*

**Terminal 3: Save the Map (When finished)**
```cmd
wsl bash -c "source /opt/ros/humble/setup.bash && cd ~/nero && source install/setup.bash && ros2 run nav2_map_server map_saver_cli -f ~/nero/my_robot/maps/my_map"
```

## Build Package (If you make changes)

```cmd
wsl bash -c "source /opt/ros/humble/setup.bash && cd ~/nero && colcon build --packages-select my_robot --symlink-install && source install/setup.bash"
```

## What Was Fixed & Added

### Critical Issues Resolved:
1.  **ROS 1 -> ROS 2 Migration**: Converted catkin -> ament_cmake
2.  **Package Paths**: Changed `package://Assem2.SLDASM` -> `package://my_robot`
3.  **Joint Limits**: Fixed zero-range joints to allow movement (3.14 rad)
4.  **Offline Gazebo**: Configured environment to skip model download timeouts
5.  **Mapping Stack**: Integrated SLAM Toolbox and Nav2 Map Saver

## Robot Specifications

- **Name**: my_robot (originally Assem2.SLDASM from SolidWorks)
- **Links**: 7 total
  - base_link (14.6 kg main body)
  - 4 wheels: front_leftwheel, front_rightwheel, back_leftwheel, back_rigtwheel
  - 2 arms: leftarm, rightarm
- **Sensors**: LiDAR (Ray Sensor) on `lidar_link`
- **Plugins**: Planar Move (Omni-drive), Joint State Publisher

## File Structure

```
~/nero/
 my_robot/                      # Your ROS 2 package
    launch/
       display.launch.py     # RViz only
       gazebo_3rooms.launch.py # Offline 3-room simulation
       mapping.launch.py     # SLAM + Gazebo + RViz
    urdf/
       Assem2_pkg.urdf      # Robot description with plugins
    worlds/
       three_rooms.world    # Offline-ready environment
    maps/                     # Saved maps go here
    rviz/
       mapping.rviz         # RViz config for SLAM
    scripts/
        keyboard_teleop.py   # Robot controller
 ROBOT_SETUP_FINAL.md         # This file
```
