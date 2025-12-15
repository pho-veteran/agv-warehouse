#!/usr/bin/env bash
set -euo pipefail

# Launch TurtleBot3 in the turtlebot_warehouse worlds (inside docker container).
#
# Usage examples:
#   ./scripts/launch_warehouse_world.sh
#   ./scripts/launch_warehouse_world.sh --world small_warehouse.world
#   ./scripts/launch_warehouse_world.sh --headless
#   ./scripts/launch_warehouse_world.sh --x 2.0 --y 1.0
#   ./scripts/launch_warehouse_world.sh --slam
#   ./scripts/launch_warehouse_world.sh --slam --slam-default
#
# Notes:
# - Default gui is true. Use --headless to run without GUI.
# - Requires container running: ./scripts/run_dev.sh up

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AGV_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
DOCKER_DIR="$AGV_DIR/docker"

WORLD="no_roof_small_warehouse.world"
GUI="true"
X_POSE="0.0"
Y_POSE="0.0"
MODEL="${TURTLEBOT3_MODEL:-waffle_pi}"
LAUNCH_SLAM="false"
SLAM_CONFIG="tuned"  # "tuned" or "default"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --world)
      WORLD="${2:-}"; shift 2;;
    --gui)
      GUI="${2:-true}"; shift 2;;
    --headless)
      GUI="false"; shift 1;;
    --x)
      X_POSE="${2:-0.0}"; shift 2;;
    --y)
      Y_POSE="${2:-0.0}"; shift 2;;
    --model)
      MODEL="${2:-waffle_pi}"; shift 2;;
    --slam)
      LAUNCH_SLAM="true"; shift 1;;
    --slam-default)
      SLAM_CONFIG="default"; shift 1;;
    -h|--help)
      echo "Launch TurtleBot3 Warehouse World"
      echo ""
      echo "Usage: $0 [OPTIONS]"
      echo ""
      echo "Options:"
      echo "  --world WORLD    World file (default: no_roof_small_warehouse.world)"
      echo "  --gui true|false Enable/disable GUI (default: true)"
      echo "  --headless       Run without GUI"
      echo "  --x X_POSE       X spawn position (default: 0.0)"
      echo "  --y Y_POSE       Y spawn position (default: 0.0)"
      echo "  --model MODEL    TurtleBot3 model (default: waffle_pi)"
      echo "  --slam           Launch Cartographer SLAM with world"
      echo "  --slam-default   Use default TurtleBot3 SLAM config"
      echo "  -h, --help       Show this help"
      exit 0;;
    *)
      echo "Unknown arg: $1" >&2
      echo "Run: $0 --help" >&2
      exit 2;;
  esac
done

cd "$DOCKER_DIR"

echo "=========================================="
echo "Launch Warehouse Simulation"
echo "=========================================="
echo "World:          $WORLD"
echo "Spawn position: ($X_POSE, $Y_POSE)"
echo "GUI:            $GUI"
echo "SLAM:           $LAUNCH_SLAM"
if [[ "$LAUNCH_SLAM" == "true" ]]; then
  echo "SLAM config:    $SLAM_CONFIG"
fi
echo "=========================================="
echo ""

# Launch world in background
echo "Starting warehouse world..."
docker compose exec -d agv_sim bash -lc "\
  source /opt/ros/jazzy/setup.bash && \
  cd /ros2_ws && \
  if [ ! -f /ros2_ws/install/setup.bash ]; then \
    echo \"[launch_warehouse_world] /ros2_ws/install/setup.bash not found. Building workspace...\"; \
    colcon build --symlink-install --packages-select turtlebot3_warehouse_sim --packages-ignore turtlebot3_description turtlebot3_gazebo; \
  fi && \
  source /ros2_ws/install/setup.bash && \
  export TURTLEBOT3_MODEL=${MODEL} && \
  ros2 launch turtlebot3_warehouse_sim turtlebot3_warehouse_world.launch.py \
    world:=${WORLD} gui:=${GUI} x_pose:=${X_POSE} y_pose:=${Y_POSE}
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


