import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot')
    nav2_dir = get_package_share_directory('nav2_bringup')
    slam_dir = get_package_share_directory('slam_toolbox')
    
    urdf_file = os.path.join(pkg_share, 'urdf', 'Assem2_pkg.urdf')
    rviz_config = os.path.join(pkg_share, 'config', 'nav2.rviz')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()
        
    return LaunchDescription([
        # 1. Robot State Publisher (TF: base_link -> joints)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        
        # 2. Mecanum Controller (TF: odom -> base_footprint)
        Node(
            package='my_robot',
            executable='mecanum_controller_complete.py',
            name='mecanum_controller',
            output='screen'
        ),
        
        # 3. SLAM Toolbox (TF: map -> odom)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(slam_dir, 'launch', 'online_async_launch.py')),
            launch_arguments={'use_sim_time': 'false'}.items()
        ),
        
        # 4. Nav2 Navigation Stack (Planner, Controller, BT)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_dir, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': 'false',
                'params_file': nav2_params
            }.items()
        ),
        
        # 5. RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])
