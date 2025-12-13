from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # Get the URDF file path
    urdf_file = os.path.join(os.path.expanduser('~'), 'nero', 'lidar_robot.urdf')
    
    # Declare launch arguments
    x_pos = LaunchConfiguration('x_pos', default='0.0')
    y_pos = LaunchConfiguration('y_pos', default='-11.0')
    z_pos = LaunchConfiguration('z_pos', default='0.1')
    
    # Read URDF file
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
    
    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'lidar_robot',
            '-topic', 'robot_description',
            '-x', x_pos,
            '-y', y_pos,
            '-z', z_pos
        ],
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('x_pos', default_value='0.0'),
        DeclareLaunchArgument('y_pos', default_value='-11.0'),
        DeclareLaunchArgument('z_pos', default_value='0.1'),
        robot_state_publisher,
        spawn_entity
    ])
