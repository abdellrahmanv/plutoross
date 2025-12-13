#!/bin/bash

# Launch Gazebo with the hotel world
echo "Starting Gazebo with hotel environment..."
gazebo ~/hospital_simple.world &
GAZEBO_PID=$!

# Wait for Gazebo to start
echo "Waiting for Gazebo to initialize..."
sleep 5

# Copy files to WSL if needed
cp /mnt/c/Users/Asus/nero/lidar_robot.urdf ~/nero/ 2>/dev/null || true
cp /mnt/c/Users/Asus/nero/spawn_robot.launch.py ~/nero/ 2>/dev/null || true
cp /mnt/c/Users/Asus/nero/nav2_params.yaml ~/nero/ 2>/dev/null || true

# Source ROS 2
source /opt/ros/humble/setup.bash

# Spawn the robot in Gazebo
echo "Spawning robot at entrance (y=-11)..."
ros2 launch ~/nero/spawn_robot.launch.py x_pos:=0.0 y_pos:=-11.0 z_pos:=0.1 &
SPAWN_PID=$!

# Wait for robot to spawn
sleep 3

echo ""
echo "===================================="
echo "Gazebo and Robot are ready!"
echo "===================================="
echo ""
echo "Next steps:"
echo "1. Build a map using SLAM:"
echo "   ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True"
echo ""
echo "2. Or launch Nav2 with a pre-built map:"
echo "   ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True params_file:=~/nero/nav2_params.yaml"
echo ""
echo "3. Visualize in RViz:"
echo "   rviz2"
echo ""
echo "4. Control the robot manually:"
echo "   ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo ""
echo "Press Ctrl+C to stop all processes"
echo ""

# Keep script running
wait $GAZEBO_PID
