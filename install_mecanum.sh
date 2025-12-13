#!/bin/bash
# Install mecanum control scripts

echo "Installing mecanum control system..."

cd ~/nero/my_robot

# Create scripts directory
mkdir -p scripts

# Copy scripts from Windows
cp /mnt/c/Users/Asus/nero/my_robot/scripts/mecanum_controller.py scripts/
cp /mnt/c/Users/Asus/nero/my_robot/scripts/keyboard_teleop.py scripts/
cp /mnt/c/Users/Asus/nero/my_robot/launch/mecanum_control.launch.py launch/

# Make executable
chmod +x scripts/*.py

# Update CMakeLists.txt to install scripts
cat >> CMakeLists.txt << 'EOF'

# Install Python scripts
install(PROGRAMS
  scripts/mecanum_controller.py
  scripts/keyboard_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)
EOF

echo "✓ Scripts installed"
echo "✓ Launch file copied"
echo ""
echo "Now rebuild the package:"
echo "  cd ~/nero"
echo "  source /opt/ros/humble/setup.bash"
echo "  colcon build --packages-select my_robot --symlink-install"
