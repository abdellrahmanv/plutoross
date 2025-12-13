from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot')
    urdf_file = os.path.join(pkg_share, 'urdf', 'Assem2_pkg.urdf')
    # Use the new forced configuration that has RobotModel and Fixed Frame set correctly
    rviz_config = os.path.join(pkg_share, 'rviz', 'force_fix.rviz')
    
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()
    
    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),

        # Mecanum Mobile Base Controller (publishes all 6 joints)
        Node(
            package='my_robot',
            executable='mecanum_controller_complete.py',
            name='mecanum_controller',
            output='screen'
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])
