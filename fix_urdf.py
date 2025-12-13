#!/usr/bin/env python3
import re

urdf_in = "/home/nero/nero/my_robot/urdf/Assem2.SLDASM.urdf"
urdf_out = "/home/nero/nero/my_robot/urdf/Assem2_final.urdf"

with open(urdf_in, "r") as f:
    content = f.read()

# Fix mesh paths
content = content.replace("package://Assem2.SLDASM", "file:///home/nero/nero/my_robot")

# Fix joint limits for all joints with zero limits
content = re.sub(
    r'lower="0"\s+upper="0"\s+effort="0"\s+velocity="0"',
    'lower="-3.14" upper="3.14" effort="10.0" velocity="1.0"',
    content
)

with open(urdf_out, "w") as f:
    f.write(content)

print("✓ Fixed URDF created at:", urdf_out)
print("✓ Mesh paths updated to file:// protocol")
print("✓ Joint limits updated for movable joints")
