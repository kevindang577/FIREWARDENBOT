# Fire Warden Bot - World Environments

This README describes the various world environments available for drone navigation training and testing.

## Available Worlds

### 1. **Simple Trees** (`simple_trees.sdf`)
- **Purpose**: Basic navigation training
- **Features**: Minimal tree placement with wide open spaces
- **Best for**: Initial drone testing and basic obstacle avoidance
- **Trees**: 2 Oak trees, 1 Pine tree in simple layout

### 2. **Large Demo** (`large_demo.sdf`)
- **Purpose**: Complex environment demonstration
- **Features**: Forest walls, varied terrain, extensive tree coverage
- **Best for**: Advanced navigation and mapping
- **Trees**: Multiple varieties with forest boundaries

### 3. **Sparse Forest** (`sparse_forest.sdf`)
- **Purpose**: Intermediate navigation training
- **Features**: Trees spaced 8m apart in grid pattern with some random placement
- **Best for**: Learning path planning between obstacles
- **Trees**: Oak and Pine trees with good visibility between them
- **Lighting**: Bright forest lighting for clear visibility

### 4. **Dense Forest** (`dense_forest.sdf`)
- **Purpose**: Advanced navigation challenges
- **Features**: Tight tree clusters with narrow passages (2-3m gaps)
- **Best for**: Precision flying and tight space navigation
- **Trees**: Clustered Oak and Pine trees creating maze-like environment
- **Lighting**: Dimmer lighting to simulate dense canopy

### 5. **Mixed Terrain** (`mixed_terrain.sdf`)
- **Purpose**: Varied elevation and terrain navigation
- **Features**: Multiple ground planes at different heights, valley and hill trees
- **Best for**: 3D navigation training with altitude variations
- **Trees**: Pine trees on elevated areas, Oak trees in valleys
- **Terrain**: Elevated grass areas and varied ground levels

### 6. **Open Meadows** (`open_meadows.sdf`)
- **Purpose**: Long-range navigation and waypoint following
- **Features**: Large open grass areas with scattered individual trees
- **Best for**: GPS navigation, long-distance flight planning
- **Trees**: Widely spaced trees (10-12m apart) for landmark navigation
- **Lighting**: Bright meadow lighting for maximum visibility

### 7. **Obstacle Course** (`obstacle_course.sdf`)
- **Purpose**: Precision navigation training
- **Features**: Strategic tree placement forming gates, slaloms, and tight turns
- **Best for**: Advanced piloting skills and competition training
- **Layout**: 
  - Entry gate (6m wide)
  - Slalom section
  - Narrow gate (3m wide)
  - Figure-8 pattern
  - Tight turn section
- **Trees**: Precisely positioned for navigation challenges

## Usage

### For Husky Robot (from 41068_ws):
```bash
cd /home/student/git/FIREWARDENBOT/World/41068_ws
source /home/student/git/FIREWARDENBOT/World/41068_ws/install/setup.bash
ros2 launch 41068_ignition_bringup 41068_ignition.launch.py world:=<world_name>
```

### For Drone (from WorldWithDrone):
```bash
cd /home/student/git/FIREWARDENBOT/World/WorldWithDrone
source /home/student/git/FIREWARDENBOT/World/WorldWithDrone/install/setup.bash
ros2 launch 41068_ignition_bringup 41068_ignition_drone.launch.py world:=<world_name> rviz:=false nav2:=false
```

**Note**: For testing, you can disable RViz and Nav2 with `rviz:=false nav2:=false` to launch faster.

### Available world names:
- `simple_trees`
- `large_demo`
- `sparse_forest`
- `dense_forest`
- `mixed_terrain`
- `open_meadows`
- `obstacle_course`

## Navigation Training Progression

1. **Beginner**: Start with `open_meadows` for basic flying and GPS navigation
2. **Intermediate**: Progress to `sparse_forest` for obstacle avoidance
3. **Advanced**: Use `dense_forest` for tight space navigation
4. **Expert**: Challenge with `obstacle_course` for precision flying
5. **Real-world**: Test with `mixed_terrain` for varied elevation handling

## Customisation 

Each world can be modified by:
1. Editing the `.sdf` file directly
2. Adjusting tree positions by changing `<pose>` values
3. Adding more trees by including additional `<include>` blocks
4. Modifying lighting by adjusting the `<light>` parameters
5. Changing terrain by modifying ground plane positions

## Application

These worlds simulate different forest environments that a fire warden drone might encounter:
- **Sparse Forest**: Early fire detection in accessible areas
- **Dense Forest**: Deep forest monitoring where access is limited
- **Mixed Terrain**: Mountainous regions with varied topography
- **Open Meadows**: Grassland fire monitoring
- **Obstacle Course**: Emergency response navigation training

## Building and Testing

After creating or modifying worlds, rebuild the appropriate workspace:

### For Husky Robot:
```bash
cd /home/student/git/FIREWARDENBOT/World/41068_ws
colcon build
source /home/student/git/FIREWARDENBOT/World/41068_ws/install/setup.bash
ros2 launch 41068_ignition_bringup 41068_ignition.launch.py world:=your_world_name
```

### For Drone:
```bash
cd /home/student/git/FIREWARDENBOT/World/WorldWithDrone
colcon build
source /home/student/git/FIREWARDENBOT/World/WorldWithDrone/install/setup.bash
ros2 launch 41068_ignition_bringup 41068_ignition_drone.launch.py world:=your_world_name rviz:=false nav2:=false
```

**Tip**: Use `rviz:=false nav2:=false` for faster testing without visualization and navigation components.
