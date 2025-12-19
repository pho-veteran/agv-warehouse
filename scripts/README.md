# Scripts for Phase 4

## Coordinate Extraction

### Quick Start
```bash
# 1. Start Docker container
./scripts/run_dev.sh up

# 2. Create stations file from world coordinates
python3 scripts/create_stations.py

# 3. (Optional) Launch warehouse + Nav2 to refine coordinates
./scripts/load_map.sh
```

### What `load_map.sh` does:
1. Launches Gazebo with tugbot_warehouse world
2. Spawns Tugbot robot
3. Starts Nav2 with static map (warehouse_tugbot.yaml)
4. Opens RViz2 for visualization

### Extract Coordinates in RViz2:
- **2D Pose Estimate**: Click + drag for position + orientation
- **Publish Point**: Click for position only
- **Mouse cursor**: Check status bar for coordinates

### Monitor coordinates:
```bash
docker compose -f docker/compose.yaml exec agv_sim ros2 topic echo /initialpose
```

### Files Created
- `config/warehouse_stations.yaml` - Station coordinates for navigation (20 stations)

### Scripts
- `create_stations.py` - Generate stations from world file coordinates
- `load_map.sh` - Launch Gazebo + Nav2 + RViz2 with static map

### Usage in Phase 4
The `warehouse_stations.yaml` file will be used by:
- `agv_transport` package
- `station_manager.py` module  
- Task Manager for navigation goals