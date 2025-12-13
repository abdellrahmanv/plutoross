from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot')
    urdf_file = os.path.join(pkg_share, 'urdf', 'Assem2_pkg.urdf')
    rviz_config = os.path.join(pkg_share, 'rviz', 'display.rviz')
    
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )
    
    # Mecanum Controller (publishes wheel states only)
    mecanum_controller = Node(
        package='my_robot',
        executable='mecanum_controller_fixed.py',
        name='mecanum_controller',
        output='screen'
    )
    
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
