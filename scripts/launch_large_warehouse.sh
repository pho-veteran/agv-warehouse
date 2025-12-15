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

docker compose exec agv_sim bash -lc "\
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
    gui:=${GUI} x_pose:=${FINAL_X} y_pose:=${FINAL_Y} \
"
