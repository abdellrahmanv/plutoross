#!/bin/bash
echo "=== ARM DIAGNOSTICS ==="
echo ""

echo "1. Checking if Gazebo is running..."
if pgrep -x "gzserver" > /dev/null; then
    echo "   ✓ Gazebo server is running"
else
    echo "   ✗ Gazebo server NOT running - please start Gazebo first!"
    exit 1
fi

echo ""
echo "2. Checking ROS2 nodes..."
ros2 node list | grep -i "arm\|gazebo"

echo ""
echo "3. Checking available topics..."
ros2 topic list | grep -i "cmd_arm\|joint"

echo ""
echo "4. Checking Gazebo services..."
ros2 service list | grep -i "joint\|effort"

echo ""
echo "5. Checking joint states..."
timeout 2 ros2 topic echo /joint_states --once 2>/dev/null | grep -A2 "name:"

echo ""
echo "6. Testing keyboard teleop topic..."
echo "   Checking if cmd_arm_vel topic exists..."
ros2 topic info /cmd_arm_vel

echo ""
echo "=== DIAGNOSTIC COMPLETE ==="
echo ""
echo "If you see issues above, share the output with me!"
