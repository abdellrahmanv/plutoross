#!/usr/bin/env python3
"""Add simple effort controller plugin to URDF for arm joints"""

urdf_file = "/home/nero/nero/my_robot/urdf/Assem2_pkg.urdf"

# Read the file
with open(urdf_file, 'r') as f:
    content = f.read()

# Find the closing </robot> tag and insert plugin before it
plugin_xml = '''
  <!-- Simple Gazebo Effort Controller for Arms -->
  <gazebo>
    <plugin name="gazebo_ros_joint_effort_controller" filename="libgazebo_ros_joint_effort_controller.so">
      <ros>
        <namespace>/</namespace>
      </ros>
      <joint_name>leftarmj</joint_name>
      <effort>10.0</effort>
      <pid>
        <p>100.0</p>
        <i>0.01</i>
        <d>10.0</d>
      </pid>
    </plugin>
  </gazebo>

  <gazebo>
    <plugin name="gazebo_ros_joint_effort_controller_right" filename="libgazebo_ros_joint_effort_controller.so">
      <ros>
        <namespace>/</namespace>
      </ros>
      <joint_name>rightarmj</joint_name>
      <effort>10.0</effort>
      <pid>
        <p>100.0</p>
        <i>0.01</i>
        <d>10.0</d>
      </pid>
    </plugin>
  </gazebo>

'''

# Insert before </robot>
if '</robot>' in content:
    content = content.replace('</robot>', plugin_xml + '</robot>')
    
    # Write back
    with open(urdf_file, 'w') as f:
        f.write(content)
    print("✓ Added effort controller plugins for both arms")
    print("✓ Arms will respond to effort commands")
else:
    print("✗ Could not find </robot> tag")

