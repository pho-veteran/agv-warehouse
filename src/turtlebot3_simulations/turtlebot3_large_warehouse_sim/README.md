# TurtleBot3 Large Warehouse Simulation

This package provides a large warehouse simulation environment for TurtleBot3 with ROS2 Jazzy and Gazebo Harmonic.

## Overview

Based on the [MovAi Tugbot Warehouse](https://app.gazebosim.org/MovAi/worlds/tugbot_warehouse) from Gazebo Fuel, modified for:
- **Gazebo Harmonic** compatibility (SDF 1.8, updated plugins)
- **TurtleBot3 Waffle Pi** support (replaced Tugbot)
- **Walking actors** for dynamic obstacle testing

## World Features

| Feature | Description |
|---------|-------------|
| **Size** | ~35m x 50m warehouse |
| **Shelves** | 16 shelves (11 regular + 5 big) |
| **Charging Station** | 1 Tugbot-compatible station at (14.7, -10.6) |
| **Pallet Boxes** | 5 pallet boxes |
| **Cart** | 1 mobile cart |
| **Walking Actors** | 2 actors with patrol trajectories |

## Usage

### Launch the simulation

```bash
# Default spawn at origin
ros2 launch turtlebot3_large_warehouse_sim turtlebot3_large_warehouse.launch.py

# Spawn near charging station
ros2 launch turtlebot3_large_warehouse_sim turtlebot3_large_warehouse.launch.py x_pose:=12.0 y_pose:=-10.0

# Without GUI (headless)
ros2 launch turtlebot3_large_warehouse_sim turtlebot3_large_warehouse.launch.py gui:=false
```

### Recommended spawn positions

| Position | Coordinates | Description |
|----------|-------------|-------------|
| Center | `(0, 0, 0.1)` | Middle of warehouse |
| Charging | `(12, -10, 0.1)` | Near charging station |
| Loading | `(0, 12, 0.1)` | Near pallet boxes |

## Directory Structure

```
turtlebot3_large_warehouse_sim/
├── CMakeLists.txt
├── package.xml
├── README.md
├── launch/
│   └── turtlebot3_large_warehouse.launch.py
└── warehouse_assets/
    ├── models/
    │   └── Walking_actor/
    │       ├── model.config
    │       ├── model.sdf
    │       └── meshes/
    │           └── walk.dae
    ├── worlds/
    │   └── tugbot_warehouse.world
    └── maps/
        └── (generated after SLAM)
```

## Walking Actors

Two walking actors patrol the warehouse:

1. **warehouse_worker_1**: Patrols around central storage area (shelves)
   - Path: rectangular loop around Y=[-1, 8.5], X=[-4, 5.5]
   - Cycle time: ~46 seconds

2. **warehouse_worker_2**: Patrols near charging station area
   - Path: rectangular loop near (10-13, -10 to -18)
   - Cycle time: ~32 seconds

## Dependencies

- `ros_gz_sim`
- `ros_gz_bridge`
- `turtlebot3_gazebo`
- `robot_state_publisher`

## Notes

- First launch may take time to download models from Gazebo Fuel
- Ensure `TURTLEBOT3_MODEL=waffle_pi` is set
- Walking actors require the `Walking_actor` model in the models path

