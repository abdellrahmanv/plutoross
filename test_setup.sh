#!/bin/bash

echo "========================================"
echo "LiDAR Navigation System - Quick Test"
echo "========================================"
echo ""

# Source ROS 2
source /opt/ros/humble/setup.bash

echo "Checking ROS 2 installation..."
if ! command -v ros2 &> /dev/null; then
    echo "❌ ROS 2 not found"
    exit 1
fi
echo "✓ ROS 2 Humble found"

echo ""
echo "Checking Gazebo..."
if ! command -v gazebo &> /dev/null; then
    echo "❌ Gazebo not found"
    exit 1
fi
echo "✓ Gazebo found"

echo ""
echo "Checking world file..."
if [ -f ~/nero/hospital_simple.world ]; then
    echo "✓ World file exists: ~/nero/hospital_simple.world"
else
    echo "❌ World file not found"
    exit 1
fi

echo ""
echo "Checking robot URDF..."
if [ -f ~/nero/lidar_robot.urdf ]; then
    echo "✓ Robot URDF exists: ~/nero/lidar_robot.urdf"
else
    echo "❌ Robot URDF not found"
    exit 1
fi

echo ""
echo "Checking Nav2 installation..."
if ros2 pkg list | grep -q "nav2_bringup"; then
    echo "✓ Nav2 installed"
else
    echo "⚠ Nav2 not fully installed"
    echo "  Run: sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup"
fi

echo ""
echo "Checking SLAM Toolbox..."
if ros2 pkg list | grep -q "slam_toolbox"; then
    echo "✓ SLAM Toolbox installed"
else
    echo "⚠ SLAM Toolbox not installed"
    echo "  Run: sudo apt install ros-humble-slam-toolbox"
fi

echo ""
echo "========================================"
echo "Setup Status: Ready! ✓"
echo "========================================"
echo ""
echo "To start the complete system:"
echo "  bash ~/nero/start_navigation.sh"
echo ""
echo "Or start components individually:"
echo "  1. Gazebo:  gazebo ~/nero/hospital_simple.world"
echo "  2. Robot:   ros2 launch ~/nero/spawn_robot.launch.py"
echo "  3. SLAM:    bash ~/nero/start_slam.sh"
echo "  4. Teleop:  ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo ""
echo "Read full guide: ~/nero/NAVIGATION_GUIDE.md"
echo ""
