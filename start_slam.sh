#!/bin/bash

# SLAM mapping script for hotel environment
echo "Starting SLAM mapping..."

# Source ROS 2
source /opt/ros/humble/setup.bash

# Ensure files are in WSL
cp /mnt/c/Users/Asus/nero/nav2_params.yaml ~/nero/ 2>/dev/null || true

# Launch SLAM Toolbox
echo "Launching SLAM Toolbox..."
ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=True \
  slam_params_file:=~/nero/nav2_params.yaml &

SLAM_PID=$!

# Wait a bit for SLAM to initialize
sleep 3

echo ""
echo "===================================="
echo "SLAM Mapping Started!"
echo "===================================="
echo ""
echo "Instructions:"
echo "1. Open RViz in another terminal:"
echo "   rviz2"
echo ""
echo "2. In RViz, add these displays:"
echo "   - Add > By topic > /map > Map"
echo "   - Add > By topic > /scan > LaserScan"
echo "   - Add > RobotModel"
echo "   - Set Fixed Frame to 'map'"
echo ""
echo "3. Drive the robot around to build the map:"
echo "   ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo ""
echo "4. When done mapping, save the map:"
echo "   ros2 run nav2_map_server map_saver_cli -f ~/nero/hotel_map"
echo ""
echo "Press Ctrl+C to stop SLAM"
echo ""

# Keep running
wait $SLAM_PID
