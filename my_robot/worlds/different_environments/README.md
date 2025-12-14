# Realistic Gazebo World Environments

This directory contains professionally designed Gazebo world files inspired by AWS RoboMaker environments for realistic robot navigation and SLAM testing.

## Available Environments

### 1. Hospital World (`hospital_realistic.world`)
**Description**: Multi-room hospital environment with patient rooms, nurses station, waiting area, and examination rooms.

**Features**:
- 2 Patient rooms with hospital beds and medical equipment carts
- Nurses station with reception counter
- Waiting area with 3 chairs and coffee table
- Medical supply cabinets and emergency equipment
- Examination room with exam table
- Realistic hospital lighting with ceiling lights
- Walls: 35m x 30m perimeter, 3m height

**Best For**: 
- Indoor navigation with rooms and corridors
- Obstacle avoidance around furniture
- Multi-room mapping scenarios

**Spawn Location**: Center (0, 0, 0)

---

### 2. Warehouse World (`warehouse_realistic.world`)
**Description**: Large-scale warehouse with storage racks, pallets, shipping containers, and loading dock.

**Features**:
- 4 rows of storage racks (4m x 1.2m x 3m each)
- 15+ rack units organized in aisles
- Pallet stacks in southern area
- 2 Shipping containers (6m x 2.4m x 2.4m)
- Forklift (static) and loading dock
- Industrial lighting system (5 ceiling lights)
- Walls: 55m x 50m perimeter, 4m height

**Best For**:
- Large open space navigation
- Long-distance path planning
- Narrow aisle navigation between racks
- Logistics robot testing

**Spawn Location**: Center (0, 0, 0)

---

### 3. Office World (`office_realistic.world`)
**Description**: Modern office environment with cubicles, conference room, reception area, and various office furniture.

**Features**:
- Reception area with desk and 3 waiting chairs
- 3 Cubicle dividers creating workstation areas
- 2 Office desks with chairs
- Conference room with table and 4 chairs
- File cabinets, bookshelf, water cooler
- Printer station with printer
- Decorative plants
- Office carpet floor
- Walls: 32m x 24m perimeter, 3m height

**Best For**:
- Dense obstacle environments
- Indoor navigation with furniture
- Small space maneuvering
- Office service robot testing

**Spawn Location**: Center (0, 0, 0)

---

## How to Use These Worlds

### Method 1: With Mapping Launch File

```bash
# Terminal 1 - Launch Gazebo + SLAM + RViz (choose one world)
wsl bash -c "source /opt/ros/humble/setup.bash && cd ~/nero && source install/setup.bash && export GAZEBO_MODEL_DATABASE_URI= && export GAZEBO_IP=127.0.0.1 ; export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:/home/nero/nero/install/my_robot/share ; ros2 launch my_robot mapping.launch.py world:=/home/nero/nero/my_robot/worlds/different_environments/hospital_realistic.world"

# Terminal 2 - Keyboard Teleop
wsl bash -c "source /opt/ros/humble/setup.bash && cd ~/nero && source install/setup.bash && ros2 run my_robot keyboard_teleop.py"

# Terminal 3 - Save Map (when mapping is complete)
wsl bash -c "source /opt/ros/humble/setup.bash && cd ~/nero && source install/setup.bash && ros2 run nav2_map_server map_saver_cli -f ~/my_warehouse_map"
```

### Method 2: Direct Gazebo Launch

Replace the world file path in your launch file:
```python
world_file = os.path.join(pkg_share, 'worlds', 'different_environments', 'hospital_realistic.world')
```

---

## World Comparison Table

| Feature | Hospital | Warehouse | Office |
|---------|----------|-----------|--------|
| Size | 35m x 30m | 55m x 50m | 32m x 24m |
| Wall Height | 3m | 4m | 3m |
| Obstacles | Medium | High | High |
| Open Space | Medium | Large | Small |
| Narrow Passages | Yes | Yes (aisles) | Yes (cubicles) |
| Lighting | 3 ceiling lights | 5 ceiling lights | 4 ceiling lights |
| Complexity | Medium | Low | High |
| Best Robot Size | Medium | Large | Small-Medium |

---

## Design Philosophy

These worlds are inspired by **AWS RoboMaker** professional environments and follow best practices:

1. **Realistic Dimensions**: All objects use real-world measurements
2. **Proper Lighting**: Multiple light sources for realistic shadows
3. **Strategic Obstacles**: Furniture and equipment placed to test navigation
4. **Collision Geometry**: All objects have proper collision boxes
5. **Visual Variety**: Different materials and colors for visual SLAM
6. **Open Navigation Paths**: Wide enough for robot movement (1.5m+ corridors)

---

## References

- AWS RoboMaker Hospital World: https://github.com/aws-robotics/aws-robomaker-hospital-world
- AWS RoboMaker Small Warehouse World: https://github.com/aws-robotics/aws-robomaker-small-warehouse-world
- AWS RoboMaker Bookstore World (Office reference)

---

## Troubleshooting

**Issue**: Robot falls through floor
- **Solution**: Ensure `<static>true</static>` is set for ground plane

**Issue**: Robot stuck in obstacles
- **Solution**: Increase clearance around spawn point to (0, 0, 0) with 2m radius

**Issue**: Dark environment
- **Solution**: Adjust `<ambient>` values in `<scene>` tag or add more point lights

**Issue**: Gazebo crashes
- **Solution**: Check SDF syntax with `gz sdf -k worldfile.world`

---

## Customization Tips

1. **Add More Obstacles**: Copy existing model blocks and change `<pose>` values
2. **Change Colors**: Modify `<ambient>` and `<diffuse>` RGB values (0-1 range)
3. **Adjust Lighting**: Change `<range>` and `<constant>` in `<attenuation>` blocks
4. **Resize Objects**: Modify `<size>` in `<box>` geometry
5. **Add Rooms**: Create new wall models with different pose positions

---

## Credits

World designs inspired by AWS Robotics team's professional Gazebo environments.
Created for ROS 2 Humble + Gazebo Classic 11.
