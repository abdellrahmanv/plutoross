#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot')
    nav2_dir = get_package_share_directory('nav2_bringup')
    
    # 1. Launch Gazebo + Robot + State Publisher
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'gazebo_world.launch.py')
        )
    )

    # 2. SLAM Toolbox (Mapping + Localization)
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'base_frame': 'base_footprint',
            'odom_frame': 'odom',
            'map_frame': 'map',
            'scan_topic': '/scan',
            'mode': 'mapping'
        }]
    )

    # 3. Nav2 Navigation (Planner + Controller + BT)
    # We use navigation_launch.py which launches the nav stack without AMCL/MapServer
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'params_file': os.path.join(nav2_dir, 'params', 'nav2_params.yaml') # Use default params
        }.items()
    )

    # 4. RViz
    rviz_config = os.path.join(pkg_share, 'rviz', 'nav2_config.rviz')
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
        slam_node,
        nav2_launch,
        rviz_node
    ])
