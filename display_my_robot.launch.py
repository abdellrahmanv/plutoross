from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Path to fixed URDF
    urdf_file = os.path.join(os.path.expanduser('~'), 'nero', 'my_robot', 'urdf', 'Assem2_fixed.urdf')
    
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
    
    # Joint State Publisher GUI
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )
    
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz
    ])
