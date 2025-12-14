#!/bin/bash
echo "Installing YOLOv8 for ROS 2..."
python3 -m pip install ultralytics opencv-python --user
echo ""
echo "✓ Installation complete!"
echo "Test with: python3 -c 'from ultralytics import YOLO; print(\"YOLO ready!\")'"
