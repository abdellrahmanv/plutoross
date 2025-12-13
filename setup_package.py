#!/usr/bin/env python3

pkg_xml = '''<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot</name>
  <version>1.0.0</version>
  <description>My custom robot</description>
  <maintainer email="user@example.com">user</maintainer>
  <license>TODO</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher_gui</exec_depend>
  <exec_depend>rviz2</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
'''

cmake_txt = '''cmake_minimum_required(VERSION 3.8)
project(my_robot)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch meshes urdf config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
'''

with open("/home/nero/nero/my_robot/package.xml", "w") as f:
    f.write(pkg_xml)

with open("/home/nero/nero/my_robot/CMakeLists.txt", "w") as f:
    f.write(cmake_txt)

print("Files created successfully")
