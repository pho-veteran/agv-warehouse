#!/usr/bin/env bash
set -euo pipefail

# Launch TurtleBot3 in Large Warehouse World (inside docker container).
#
# Usage examples:
#   ./scripts/launch_large_warehouse.sh                    # Spawn at center (default)
#   ./scripts/launch_large_warehouse.sh --location center  # Spawn at center (0, 0)
#   ./scripts/launch_large_warehouse.sh --location charging # Spawn near charging station (12, -10)
#   ./scripts/launch_large_warehouse.sh --x 5.0 --y 5.0    # Custom position
#   ./scripts/launch_large_warehouse.sh --headless         # Run without GUI
#   ./scripts/launch_large_warehouse.sh --slam             # Launch with SLAM (tuned config)
#   ./scripts/launch_large_warehouse.sh --slam --slam-default # Launch with SLAM (default config)
#
# Spawn locations:
#   center   - Center of warehouse (0, 0)
#   charging - Near charging station (12, -10)
#
# Notes:
# - Requires container running: ./scripts/run_dev.sh up

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AGV_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
DOCKER_DIR="$AGV_DIR/docker"

# Default values
LOCATION="center"
GUI="true"
X_POSE=""
Y_POSE=""
MODEL="${TURTLEBOT3_MODEL:-waffle_pi}"
LAUNCH_SLAM="false"
SLAM_CONFIG="tuned"  # "tuned" or "default"

# Predefined spawn locations
declare -A SPAWN_X
declare -A SPAWN_Y
SPAWN_X["center"]="0.0"
SPAWN_Y["center"]="0.0"
SPAWN_X["charging"]="12.0"
SPAWN_Y["charging"]="-10.0"

print_help() {
    echo "Launch TurtleBot3 in Large Warehouse World"
    echo ""
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --location LOC   Spawn location: center | charging (default: center)"
    echo "  --x X_POSE       Custom X position (overrides --location)"
    echo "  --y Y_POSE       Custom Y position (overrides --location)"
    echo "  --gui true|false Enable/disable GUI (default: true)"
    echo "  --headless       Run without GUI (same as --gui false)"
    echo "  --model MODEL    TurtleBot3 model (default: waffle_pi)"
    echo "  --slam           Launch Cartographer SLAM with world"
    echo "  --slam-default   Use default TurtleBot3 SLAM config (instead of tuned)"
    echo "  -h, --help       Show this help"
    echo ""
    echo "Spawn locations:"
    echo "  center   - Center of warehouse (0, 0)"
    echo "  charging - Near charging station (12, -10)"
    echo ""
    echo "Examples:"
    echo "  $0                           # Spawn at center"
    echo "  $0 --location charging       # Spawn near charging station"
    echo "  $0 --x 5.0 --y 5.0           # Custom position"
    echo "  $0 --headless                # Without GUI"
    echo "  $0 --slam                    # Launch with SLAM (tuned config)"
    echo "  $0 --slam --slam-default     # Launch with SLAM (default config)"
}

# Parse arguments
while [[ $# -gt 0 ]]; do
  case "$1" in
    --location)
      LOCATION="${2:-center}"; shift 2;;
    --gui)
      GUI="${2:-true}"; shift 2;;
    --headless)
      GUI="false"; shift 1;;
    --x)
      X_POSE="${2:-}"; shift 2;;
    --y)
      Y_POSE="${2:-}"; shift 2;;
    --model)
      MODEL="${2:-waffle_pi}"; shift 2;;
    --slam)
      LAUNCH_SLAM="true"; shift 1;;
    --slam-default)
      SLAM_CONFIG="default"; shift 1;;
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
    # Use custom coordinates
    FINAL_X="$X_POSE"
    FINAL_Y="$Y_POSE"
elif [[ -n "${SPAWN_X[$LOCATION]:-}" ]]; then
    # Use predefined location
    FINAL_X="${SPAWN_X[$LOCATION]}"
    FINAL_Y="${SPAWN_Y[$LOCATION]}"
else
    echo "Error: Unknown location '$LOCATION'" >&2
    echo "Available: center, charging" >&2
    exit 1
fi

cd "$DOCKER_DIR"

echo "=========================================="
echo "Launch Large Warehouse Simulation"
echo "=========================================="
echo "Spawn location: ($FINAL_X, $FINAL_Y)"
echo "GUI:            $GUI"
echo "SLAM:           $LAUNCH_SLAM"
if [[ "$LAUNCH_SLAM" == "true" ]]; then
  echo "SLAM config:    $SLAM_CONFIG"
fi
echo "=========================================="
echo ""

# Launch world in background
echo "Starting large warehouse world..."
docker compose exec -d agv_sim bash -lc "\
  source /opt/ros/jazzy/setup.bash && \
  cd /ros2_ws && \
  if [ ! -f /ros2_ws/install/turtlebot3_large_warehouse_sim/share/turtlebot3_large_warehouse_sim/package.xml ]; then \
    echo '[launch_large_warehouse] Building turtlebot3_large_warehouse_sim package...'; \
    colcon build --symlink-install --packages-select turtlebot3_large_warehouse_sim; \
  fi && \
  source /ros2_ws/install/setup.bash && \
  export TURTLEBOT3_MODEL=${MODEL} && \
  echo 'Launching simulation...' && \
  ros2 launch turtlebot3_large_warehouse_sim turtlebot3_large_warehouse.launch.py \
    gui:=${GUI} x_pose:=${FINAL_X} y_pose:=${FINAL_Y}
"

# If SLAM is requested, wait for world to be ready then launch Cartographer
if [[ "$LAUNCH_SLAM" == "true" ]]; then
  echo ""
  echo "Waiting for world to initialize..."
  sleep 10
  
  echo "Checking if topics are available..."
  RETRIES=0
  MAX_RETRIES=30
  while [[ $RETRIES -lt $MAX_RETRIES ]]; do
    if docker compose exec agv_sim bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic list 2>/dev/null | grep -q '/scan'" 2>/dev/null; then
      echo "✓ Topics ready!"
      break
    fi
    echo "  Waiting for topics... ($((RETRIES+1))/$MAX_RETRIES)"
    sleep 2
    RETRIES=$((RETRIES+1))
  done
  
  if [[ $RETRIES -eq $MAX_RETRIES ]]; then
    echo "ERROR: Topics not available after $MAX_RETRIES retries"
    echo "World may not have started correctly. Check logs."
    exit 1
  fi
  
  echo ""
  echo "Launching Cartographer SLAM..."
  
  # Determine config
  if [[ "$SLAM_CONFIG" == "default" ]]; then
    CONFIG_DIR="/opt/ros/jazzy/share/turtlebot3_cartographer/config"
    CONFIG_FILE="turtlebot3_lds_2d.lua"
    echo "Using default TurtleBot3 config"
  else
    CONFIG_DIR="/ros2_ws/config"
    CONFIG_FILE="turtlebot3_slam_tuned.lua"
    echo "Using tuned config: $CONFIG_FILE"
  fi
  
  sleep 3
  
  # Launch Cartographer in foreground (so user can see logs)
  docker compose exec agv_sim bash -lc "\
    source /opt/ros/jazzy/setup.bash && \
    source /ros2_ws/install/setup.bash 2>/dev/null || true && \
    export TURTLEBOT3_MODEL=${MODEL} && \
    echo '' && \
    echo '========================================' && \
    echo 'Cartographer SLAM Ready!' && \
    echo 'Config: ${CONFIG_DIR}/${CONFIG_FILE}' && \
    echo '========================================' && \
    echo '' && \
    ros2 launch turtlebot3_cartographer cartographer.launch.py \
      use_sim_time:=true \
      cartographer_config_dir:=${CONFIG_DIR} \
      configuration_basename:=${CONFIG_FILE}
  "
else
  echo ""
  echo "=========================================="
  echo "World launched successfully!"
  echo "To add SLAM, use: --slam flag"
  echo "=========================================="
  
  # Keep script running to maintain the world
  echo ""
  echo "Press Ctrl+C to stop the simulation"
  wait
fi
