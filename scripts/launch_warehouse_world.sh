#!/usr/bin/env bash
set -euo pipefail

# Launch TurtleBot3 in the turtlebot_warehouse worlds (inside docker container).
#
# Usage examples:
#   ./scripts/launch_warehouse_world.sh
#   ./scripts/launch_warehouse_world.sh --world small_warehouse.world
#   ./scripts/launch_warehouse_world.sh --headless
#   ./scripts/launch_warehouse_world.sh --x 2.0 --y 1.0
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
    -h|--help)
      sed -n '1,60p' "$0"; exit 0;;
    *)
      echo "Unknown arg: $1" >&2
      echo "Run: $0 --help" >&2
      exit 2;;
  esac
done

cd "$DOCKER_DIR"

docker compose exec agv_sim bash -lc "\
  source /opt/ros/jazzy/setup.bash && \
  cd /ros2_ws && \
  if [ ! -f /ros2_ws/install/setup.bash ]; then \
    echo \"[launch_warehouse_world] /ros2_ws/install/setup.bash not found. Building workspace...\"; \
    colcon build --symlink-install --packages-select turtlebot3_warehouse_sim --packages-ignore turtlebot3_description turtlebot3_gazebo; \
  fi && \
  source /ros2_ws/install/setup.bash && \
  export TURTLEBOT3_MODEL=${MODEL} && \
  ros2 launch turtlebot3_warehouse_sim turtlebot3_warehouse_world.launch.py \
    world:=${WORLD} gui:=${GUI} x_pose:=${X_POSE} y_pose:=${Y_POSE} \
"


