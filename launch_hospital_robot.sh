#!/bin/bash
# Quick launch script for custom robot in Hospital Gazebo

cd ~/nero
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "========================================="
echo "Launching Hospital Environment + Robot"
echo "========================================="
echo ""
echo "Starting:"
echo "  ✓ Gazebo with hospital world"
echo "  ✓ Your custom robot (spawned at 0,0,0.5)"
echo "  ✓ RViz2 for visualization"
echo "  ✓ Joint control GUI"
echo ""
echo "Hospital floor: Ceramic (friction μ=0.3)"
echo "========================================="

ros2 launch my_robot hospital_robot.launch.py
