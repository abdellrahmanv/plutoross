#!/bin/bash
# Add ROS factory plugin to world file

WORLD_FILE=~/nero/hospital_simple.world

# Backup original
cp $WORLD_FILE ${WORLD_FILE}.backup

# Add plugin after opening world tag
sed -i '/<world name=/a\    <!-- Gazebo ROS Plugins -->\
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">\
      <ros><namespace>/</namespace></ros>\
      <update_rate>1.0</update_rate>\
    </plugin>\
    <plugin name="gazebo_ros_factory" filename="libgazebo_ros_factory.so"/>' $WORLD_FILE

echo "ROS plugins added to world file"
