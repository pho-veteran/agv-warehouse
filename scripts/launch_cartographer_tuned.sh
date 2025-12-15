#!/usr/bin/env bash
# Launch Cartographer SLAM with tuned config (General for all worlds)
#
# Usage:
#   ./scripts/launch_cartographer_tuned.sh              # Default tuned config
#   ./scripts/launch_cartographer_tuned.sh --default    # Use original TurtleBot3 config
#   ./scripts/launch_cartographer_tuned.sh --rviz       # Also launch RViz2
#
# Prerequisites:
#   - Container running: ./scripts/run_dev.sh up
#   - World already running (any warehouse world)
#
# Config file: AGV_2/config/turtlebot3_slam_tuned.lua
# Supported worlds: small_warehouse, large_warehouse, turtlebot3_world, etc.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AGV_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
DOCKER_DIR="$AGV_DIR/docker"

# Default config
CONFIG_DIR="/ros2_ws/config"
CONFIG_FILE="turtlebot3_slam_tuned.lua"
LAUNCH_RVIZ="false"

# Parse arguments
while [[ $# -gt 0 ]]; do
  case "$1" in
    --default)
      # Use original TurtleBot3 config
      CONFIG_DIR="/opt/ros/jazzy/share/turtlebot3_cartographer/config"
      CONFIG_FILE="turtlebot3_lds_2d.lua"
      shift;;
    --rviz)
      LAUNCH_RVIZ="true"
      shift;;
    -h|--help)
      echo "Usage: $0 [--default] [--rviz]"
      echo ""
      echo "Options:"
      echo "  --default    Use original TurtleBot3 config instead of tuned"
      echo "  --rviz       Also launch RViz2 for visualization"
      exit 0;;
    *)
      echo "Unknown option: $1"
      exit 1;;
  esac
done

echo "=========================================="
echo "Cartographer SLAM - Tuned Config"
echo "=========================================="
echo "Config dir:  $CONFIG_DIR"
echo "Config file: $CONFIG_FILE"
echo "Launch RViz: $LAUNCH_RVIZ"
echo ""

cd "$DOCKER_DIR"

# Check if custom config file exists (only if using tuned config)
if [[ "$CONFIG_DIR" == "/ros2_ws/config" ]]; then
  docker compose exec agv_sim bash -c "
    if [ ! -f $CONFIG_DIR/$CONFIG_FILE ]; then
      echo 'ERROR: Config file not found: $CONFIG_DIR/$CONFIG_FILE'
      echo 'Make sure AGV_2/config is mounted to /ros2_ws/config'
      exit 1
    fi
  "
fi

# Build launch command
LAUNCH_CMD="ros2 launch turtlebot3_cartographer cartographer.launch.py \
  use_sim_time:=true \
  cartographer_config_dir:=$CONFIG_DIR \
  configuration_basename:=$CONFIG_FILE"

# Launch cartographer
docker compose exec agv_sim bash -lc "
  source /opt/ros/jazzy/setup.bash
  source /ros2_ws/install/setup.bash 2>/dev/null || true
  export TURTLEBOT3_MODEL=waffle_pi
  
  echo 'Starting Cartographer SLAM...'
  $LAUNCH_CMD
"

