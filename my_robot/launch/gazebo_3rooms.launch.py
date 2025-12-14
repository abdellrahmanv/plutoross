#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot')
    urdf_file = os.path.join(pkg_share, 'urdf', 'Assem2_pkg.urdf')
    world_file = LaunchConfiguration('world')

    # Read URDF and strip XML declaration to avoid spawn_entity error
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # Robustly remove XML declaration if present
    if robot_desc.strip().startswith('<?xml'):
        index = robot_desc.find('?>')
        if index != -1:
            robot_desc = robot_desc[index+2:].strip()

    # Get gazebo_ros package path
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    # Set GAZEBO_MODEL_PATH so meshes are visible
    model_path = os.path.join(pkg_share)
    gazebo_model_path_env = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=model_path
    )
    
    # Disable online model database to prevent download check
    gazebo_model_db_env = SetEnvironmentVariable(
        name='GAZEBO_MODEL_DATABASE_URI',
        value=''
    )

    return LaunchDescription([
        # Set environment variables first
        gazebo_model_path_env,
        gazebo_model_db_env,
        
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(pkg_share, 'worlds', 'three_rooms.world'),
            description='Full path to world file to load'),
        # Launch Gazebo server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_share, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world_file, 'verbose': 'true'}.items()
        ),

        # Launch Gazebo client
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_share, 'launch', 'gzclient.launch.py')
            )
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_desc},
                {'use_sim_time': True}
            ]
        ),

        # Spawn robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'my_robot',
                '-topic', 'robot_description',
                '-x', '0',
                '-y', '0',
                '-z', '0.3'
            ],
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        # Gazebo Mecanum Controller
        Node(
            package='my_robot',
            executable='gazebo_mecanum_controller.py',
            name='gazebo_mecanum_controller',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        # Gazebo Arm Controller
        Node(
            package='my_robot',
            executable='gazebo_arm_controller.py',
            name='gazebo_arm_controller',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

    ])
