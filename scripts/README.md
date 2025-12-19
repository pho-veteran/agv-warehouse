# Scripts for AGV Transport System

## Quick Start - Transport Demo

```bash
# 1. Start Docker container
./scripts/run_dev.sh up

# 2. Launch full transport demo (Gazebo + Nav2 + RViz + Transport Manager)
./scripts/launch_transport_demo.sh

# 3. In another terminal, send a transport order
./scripts/send_order.sh dock_in_1 dock_out

# 4. Monitor transport status
./scripts/send_order.sh --status
```

## Scripts Overview

### Transport Demo Scripts

| Script | Description |
|--------|-------------|
| `launch_transport_demo.sh` | Launch full system: Gazebo + Nav2 + RViz + Transport Task Manager |
| `send_order.sh` | Send transport orders, monitor status, reset manager |
| `load_map.sh` | Launch Gazebo + Nav2 + RViz (without transport manager) |

### `launch_transport_demo.sh` Options

```bash
./scripts/launch_transport_demo.sh [OPTIONS]

Options:
  --location LOC   Spawn location: center | charging | entrance (default: entrance)
  --x X_POSE       Custom X position (world coordinates)
  --y Y_POSE       Custom Y position (world coordinates)
  --headless       Run without Gazebo GUI
  --skip-build     Skip rebuilding agv_transport package
  -h, --help       Show help
```

### `send_order.sh` Usage

```bash
# Send transport order
./scripts/send_order.sh <pickup_station> <dropoff_station> [--priority N]

# List available stations
./scripts/send_order.sh --list

# Monitor transport status (live)
./scripts/send_order.sh --status

# Reset transport manager
./scripts/send_order.sh --reset

# Examples
./scripts/send_order.sh dock_in_1 dock_out
./scripts/send_order.sh dock_in_2 charging_station --priority 5
```

### Available Stations

| Station | Coordinates (x, y) | Type |
|---------|-------------------|------|
| `charging_station` | (13.157, -30.282) | charger |
| `dock_in_1` | (-0.143, -20.679) | dock |
| `dock_in_2` | (-3.756, -6.553) | dock |
| `dock_out` | (-0.181, -43.405) | dock |

## Coordinate Extraction

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

## Configuration Files

| File | Description |
|------|-------------|
| `src/agv_transport/config/warehouse_stations.yaml` | Station coordinates for navigation |
| `src/agv_transport/config/tugbot_nav2_transport_params.yaml` | Nav2 parameters for transport |

## Debugging

### Check running nodes
```bash
docker compose exec agv_sim bash -c "source /opt/ros/jazzy/setup.bash && ros2 node list"
```

### Check topics
```bash
docker compose exec agv_sim bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic list"
```

### View transport manager logs
```bash
docker compose exec agv_sim bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic echo /agv/transport_status"
```

### Reset transport manager
```bash
docker compose exec agv_sim bash -c "source /opt/ros/jazzy/setup.bash && ros2 service call /agv/reset_transport std_srvs/srv/Trigger"
```