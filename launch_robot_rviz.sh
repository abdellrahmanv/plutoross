#!/bin/bash
# Quick launch script for custom robot in RViz

cd ~/nero
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "==================================="
echo "Launching Custom Robot in RViz"
echo "==================================="
echo ""
echo "Your robot has:"
echo "  - 1 base_link"
echo "  - 4 wheels (front/back, left/right)"
echo "  - 2 arms (left/right)"
echo ""
echo "Controls: Use joint_state_publisher_gui window"
echo "==================================="

ros2 launch my_robot display.launch.py
