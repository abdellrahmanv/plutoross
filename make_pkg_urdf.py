#!/usr/bin/env python3
import re

urdf_in = "/home/nero/nero/my_robot/urdf/Assem2.SLDASM.urdf"
urdf_out = "/home/nero/nero/my_robot/urdf/Assem2_pkg.urdf"

with open(urdf_in, "r") as f:
    content = f.read()

# Use package:// protocol
content = content.replace("package://Assem2.SLDASM", "package://my_robot")

# Fix joint limits
content = re.sub(
    r'lower="0"\s+upper="0"\s+effort="0"\s+velocity="0"',
    'lower="-3.14" upper="3.14" effort="10.0" velocity="1.0"',
    content
)

with open(urdf_out, "w") as f:
    f.write(content)

print("Created package-based URDF")
