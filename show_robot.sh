#!/bin/bash
source /opt/ros/humble/setup.bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat ~/nero/my_robot/urdf/Assem2_fixed.urdf)" &
sleep 3
rviz2
