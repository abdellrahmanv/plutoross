#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot')
    urdf_file = os.path.join(pkg_share, 'urdf', 'Assem2_pkg.urdf')
    rviz_config = os.path.join(pkg_share, 'config', 'display.rviz')
    world_file = os.path.join(os.path.expanduser('~'), 'nero', 'hospital_simple.world')

    with open(urdf_file, 'r') as file:
        robot_desc = file.read()
    
    # Fix: Remove XML declaration to prevent spawn_entity crash
    if robot_desc.strip().startswith('<?xml'):
        robot_desc = robot_desc[robot_desc.find('\n')+1:]

    return LaunchDescription([
        ExecuteProcess(cmd=['gzserver', world_file, '--verbose'], output='screen'),
        ExecuteProcess(cmd=['gzclient'], output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[{'robot_description': robot_desc}], output='screen'),
        Node(package='gazebo_ros', executable='spawn_entity.py', arguments=['-entity', 'my_robot', '-topic', 'robot_description', '-x', '0.0', '-y', '0.0', '-z', '0.5'], output='screen'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config], output='screen')
    ])
