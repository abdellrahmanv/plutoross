#!/usr/bin/env python3
"""
Launch file for mecanum wheel control in RViz2
Includes: robot visualization + mecanum controller + keyboard teleop
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Paths
    pkg_share = get_package_share_directory('my_robot')
    urdf_file = os.path.join(pkg_share, 'urdf', 'Assem2_pkg.urdf')
    rviz_config = os.path.join(pkg_share, 'config', 'display.rviz')
    
    # Read URDF
    with open(urdf_file, 'r') as file:
        robot_desc = file.read()
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )
    
    # Mecanum Controller (converts cmd_vel to joint states)
    mecanum_controller = Node(
        package='my_robot',
        executable='mecanum_controller.py',
        name='mecanum_controller',
        output='screen'
    )
    
    # Keyboard Teleop (optional, run separately if needed)
    # keyboard_teleop = Node(
    #     package='my_robot',
    #     executable='keyboard_teleop.py',
    #     name='keyboard_teleop',
    #     output='screen'
    # )
    
    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    return LaunchDescription([
        robot_state_publisher,
        mecanum_controller,
        rviz
    ])
