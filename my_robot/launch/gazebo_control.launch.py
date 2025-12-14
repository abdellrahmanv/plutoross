#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot')
    
    # 1. Launch Gazebo World (includes Robot State Publisher & Spawn Entity)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'gazebo_world.launch.py')
        )
    )
    
    # 2. Mecanum Controller (Odometry + Joint States)
    mecanum_controller = Node(
        package='my_robot',
        executable='mecanum_controller_complete.py',
        name='mecanum_controller',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # 3. RViz
    rviz_config = os.path.join(pkg_share, 'rviz', 'simple.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        gazebo_launch,
        mecanum_controller,
        rviz_node
    ])
