#!/usr/bin/env bash
set -euo pipefail

# Launch Gazebo warehouse + Tugbot + Nav2 with static map for coordinate extraction
#
# Usage:
#   ./scripts/load_map.sh                    # Launch at center
#   ./scripts/load_map.sh --location charging # Launch near charging station
#   ./scripts/load_map.sh --headless         # Without Gazebo GUI

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AGV_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
DOCKER_DIR="$AGV_DIR/docker"

# Default values
LOCATION="entrance"
GUI="true"
X_POSE=""
Y_POSE=""

# Predefined spawn locations
declare -A SPAWN_X
declare -A SPAWN_Y
SPAWN_X["center"]="0.0"
SPAWN_Y["center"]="0.0"
SPAWN_X["charging"]="12.0"
SPAWN_Y["charging"]="-10.0"
SPAWN_X["entrance"]="0.0"
SPAWN_Y["entrance"]="20.0"

print_help() {
    echo "Launch Warehouse with Static Map for Coordinate Extraction"
    echo ""
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --location LOC   Spawn location: center | charging | entrance (default: entrance)"
    echo "  --x X_POSE       Custom X position"
    echo "  --y Y_POSE       Custom Y position"
    echo "  --headless       Run without Gazebo GUI"
    echo "  -h, --help       Show this help"
    echo ""
    echo "This launches:"
    echo "  1. Gazebo with tugbot_warehouse world"
    echo "  2. Tugbot robot"
    echo "  3. Nav2 with static map (warehouse_tugbot.yaml)"
    echo "  4. RViz2 for visualization and coordinate extraction"
}

# Parse arguments
while [[ $# -gt 0 ]]; do
  case "$1" in
    --location)
      LOCATION="${2:-center}"; shift 2;;
    --x)
      X_POSE="${2:-}"; shift 2;;
    --y)
      Y_POSE="${2:-}"; shift 2;;
    --headless)
      GUI="false"; shift 1;;
    -h|--help)
      print_help; exit 0;;
    *)
      echo "Unknown arg: $1" >&2
      echo "Run: $0 --help" >&2
      exit 2;;
  esac
done

# Resolve spawn coordinates
if [[ -n "$X_POSE" ]] && [[ -n "$Y_POSE" ]]; then
    FINAL_X="$X_POSE"
    FINAL_Y="$Y_POSE"
elif [[ -n "${SPAWN_X[$LOCATION]:-}" ]]; then
    FINAL_X="${SPAWN_X[$LOCATION]}"
    FINAL_Y="${SPAWN_Y[$LOCATION]}"
else
    echo "Error: Unknown location '$LOCATION'" >&2
    echo "Available: center, charging, entrance" >&2
    exit 1
fi

cd "$DOCKER_DIR"

# Check if container is running
if ! docker compose ps agv_sim | grep -q "Up"; then
    echo "❌ Docker container not running. Start it first:"
    echo "   ./scripts/run_dev.sh up"
    exit 1
fi

echo "=========================================="
echo "Launch Warehouse with Static Map"
echo "=========================================="
echo "Robot:          Tugbot"
echo "Spawn location: ($FINAL_X, $FINAL_Y)"
echo "GUI:            $GUI"
echo "Map:            warehouse_tugbot.yaml"
echo "=========================================="
echo ""

# Step 1: Launch Gazebo warehouse + Tugbot
echo "Step 1: Starting Gazebo warehouse with Tugbot..."
docker compose exec -d agv_sim bash -lc "
    source /opt/ros/jazzy/setup.bash && \
    cd /ros2_ws && \
    if [ ! -f /ros2_ws/install/turtlebot3_large_warehouse_sim/share/turtlebot3_large_warehouse_sim/package.xml ]; then \
      echo 'Building warehouse package...'; \
      colcon build --symlink-install --packages-select turtlebot3_large_warehouse_sim; \
    fi && \
    source /ros2_ws/install/setup.bash && \
    echo 'Launching Tugbot in warehouse...' && \
    ros2 launch turtlebot3_large_warehouse_sim tugbot_large_warehouse.launch.py \
      gui:=${GUI} x_pose:=${FINAL_X} y_pose:=${FINAL_Y}
"

echo "Waiting for Gazebo to initialize..."
sleep 10

# Check if topics are available
echo "Checking if robot topics are available..."
RETRIES=0
MAX_RETRIES=30
while [[ $RETRIES -lt $MAX_RETRIES ]]; do
    if docker compose exec agv_sim bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic list 2>/dev/null | grep -q '/scan'" 2>/dev/null; then
        echo "✓ Robot topics ready!"
        break
    fi
    echo "  Waiting for topics... ($((RETRIES+1))/$MAX_RETRIES)"
    sleep 2
    RETRIES=$((RETRIES+1))
done

if [[ $RETRIES -eq $MAX_RETRIES ]]; then
    echo "ERROR: Robot topics not available after $MAX_RETRIES retries"
    exit 1
fi

echo ""
echo "Step 2: Starting Nav2 with static map and AMCL..."

# Launch full Nav2 bringup with map, AMCL, and navigation
docker compose exec -d agv_sim bash -lc "
    source /opt/ros/jazzy/setup.bash && \
    source /ros2_ws/install/setup.bash 2>/dev/null || true && \
    export TURTLEBOT3_MODEL=waffle_pi && \
    echo 'Launching Nav2 bringup with static map...' && \
    ros2 launch nav2_bringup bringup_launch.py \
      use_sim_time:=true \
      autostart:=true \
      map:=/ros2_ws/data/maps/warehouse_tugbot.yaml \
      params_file:=/ros2_ws/src/agv_transport/config/tugbot_nav2_transport_params.yaml
"

echo "Waiting for Nav2 to initialize..."
sleep 15

# Check if map is available
echo "Checking if map is available..."
RETRIES=0
while [[ $RETRIES -lt $MAX_RETRIES ]]; do
    if docker compose exec agv_sim bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic list 2>/dev/null | grep -q '/map'" 2>/dev/null; then
        echo "✓ Map topic ready!"
        break
    fi
    echo "  Waiting for map... ($((RETRIES+1))/$MAX_RETRIES)"
    sleep 2
    RETRIES=$((RETRIES+1))
done

if [[ $RETRIES -eq $MAX_RETRIES ]]; then
    echo "ERROR: Map not available after $MAX_RETRIES retries"
    exit 1
fi

echo ""
echo "Step 3: Starting RViz2..."

# Start RViz2 with Nav2 config
docker compose exec -d agv_sim bash -c "
    source /opt/ros/jazzy/setup.bash && \
    source /ros2_ws/install/setup.bash 2>/dev/null || true && \
    export TURTLEBOT3_MODEL=waffle_pi && \
    ros2 launch nav2_bringup rviz_launch.py
"

sleep 3

echo ""
echo "Step 4: Setting initial pose for AMCL..."

# Map coordinates for each spawn location (determined by manual testing)
declare -A MAP_X_COORDS
declare -A MAP_Y_COORDS

# Entrance: (0, 20) world -> (0.065, 0.097) map
MAP_X_COORDS["entrance"]="0.065"
MAP_Y_COORDS["entrance"]="0.097"

# Center: (0, 0) world -> need to be determined
MAP_X_COORDS["center"]="0.0"  # TODO: determine actual map coordinates
MAP_Y_COORDS["center"]="0.0"

# Charging: (12, -10) world -> (13.157, -30.282) map (from manual testing)
MAP_X_COORDS["charging"]="13.157"
MAP_Y_COORDS["charging"]="-30.282"

# Get map coordinates for current location
if [[ -n "${MAP_X_COORDS[$LOCATION]:-}" ]]; then
    MAP_X="${MAP_X_COORDS[$LOCATION]}"
    MAP_Y="${MAP_Y_COORDS[$LOCATION]}"
else
    # Fallback: use world coordinates (may not be accurate)
    MAP_X="$FINAL_X"
    MAP_Y="$FINAL_Y"
fi

echo "Setting initial pose:"
echo "  Location: $LOCATION"
echo "  World: (${FINAL_X}, ${FINAL_Y})"
echo "  Map:   (${MAP_X}, ${MAP_Y})"

# Set initial pose to help AMCL localize the robot
docker compose exec agv_sim bash -c "
    source /opt/ros/jazzy/setup.bash
    ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{
      header: {frame_id: \"map\"},
      pose: {
        pose: {position: {x: ${MAP_X}, y: ${MAP_Y}, z: 0.0}, orientation: {w: 1.0}},
        covariance: [0.25, 0, 0, 0, 0, 0, 0, 0.25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.07]
      }
    }'
"

sleep 2
echo "✓ Initial pose set in map coordinates"

echo ""
echo "=========================================="
echo "✓ System Ready!"
echo "=========================================="
echo "Gazebo:  Running with tugbot_warehouse"
echo "Robot:   Tugbot at ($FINAL_X, $FINAL_Y)"
echo "Nav2:    Running with static map + AMCL"
echo "Map:     warehouse_tugbot.yaml"
echo ""
echo "In RViz2:"
echo "  • Use '2D Pose Estimate' to set/correct robot position"
echo "  • Use 'Publish Point' to get coordinates"
echo "  • Check status bar for mouse coordinates"
echo ""
echo "Press Ctrl+C to stop"
echo "=========================================="

wait