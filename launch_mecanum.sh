#!/bin/bash
# Launch mecanum control - Manual mode (run teleop separately)

cd ~/nero
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "========================================="
echo "  MECANUM WHEEL CONTROL - RViz Mode"
echo "========================================="
echo ""
echo "Starting robot visualization with mecanum controller..."
echo ""
echo "To control the robot, open another terminal and run:"
echo "  cd ~/nero && source install/setup.bash"
echo "  ros2 run my_robot keyboard_teleop.py"
echo ""
echo "Or publish directly to /cmd_vel:"
echo "  ros2 topic pub /cmd_vel geometry_msgs/Twist ..."
echo "========================================="

# Launch without keyboard teleop (start that separately)
ros2 launch my_robot mecanum_control.launch.py
