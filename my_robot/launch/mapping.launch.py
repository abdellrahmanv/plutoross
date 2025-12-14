#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot')
    
    # World file from command line argument
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_share, 'worlds', 'different_environments', 'hospital_realistic.world'),
        description='Path to the Gazebo world file'
    )
    
    world_file = LaunchConfiguration('world')

    return LaunchDescription([
        world_arg,
        
        # Include the 3rooms launch file with world parameter
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'gazebo_3rooms.launch.py')
            ),
            launch_arguments={'world': world_file}.items()
        ),

        # SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml'),
                {'use_sim_time': True}
            ]
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'slam.rviz')],
            parameters=[{'use_sim_time': True}]
        ),
    ])
