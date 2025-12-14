cmake_minimum_required(VERSION 3.8)
project(my_robot)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch meshes urdf config rviz worlds
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

# Install Python scripts
install(PROGRAMS
  scripts/mecanum_controller.py
  scripts/keyboard_teleop.py
  scripts/mecanum_controller_complete.py
  DESTINATION lib/${PROJECT_NAME}
)
