#!/usr/bin/env bash
set -euo pipefail

# Launch TurtleBot3 in TurtleBot3 World (standard test environment)
# For testing agv_auto_explore frontier exploration algorithm
#
# Usage examples:
#   ./scripts/launch_turtlebot3_world.sh                    # Basic world only
#   ./scripts/launch_turtlebot3_world.sh --slam             # With SLAM Toolbox
#   ./scripts/launch_turtlebot3_world.sh --slam --explore   # With SLAM + exploration
#   ./scripts/launch_turtlebot3_world.sh --headless         # Without GUI
#   ./scripts/launch_turtlebot3_world.sh --speed 2.0        # 2x simulation speed
#
# Notes:
# - Requires container running: ./scripts/run_dev.sh up
# - TurtleBot3 World is smaller than warehouse, good for algorithm testing

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AGV_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
DOCKER_DIR="$AGV_DIR/docker"

# Default values
GUI="true"
X_POSE="-2.0"
Y_POSE="-0.5"
MODEL="${TURTLEBOT3_MODEL:-waffle_pi}"
LAUNCH_SLAM="false"
AUTO_EXPLORE="false"
SIM_SPEED="1.0"  # Real-time factor (1.0 = real-time, 2.0 = 2x speed)
SLAM_BACKEND="slam_toolbox"  # slam_toolbox | cartographer

print_help() {
    echo "Launch TurtleBot3 in TurtleBot3 World (standard test environment)"
    echo ""
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --x X_POSE       X position (default: -2.0)"
    echo "  --y Y_POSE       Y position (default: -0.5)"
    echo "  --gui true|false Enable/disable GUI (default: true)"
    echo "  --headless       Run without GUI"
    echo "  --model MODEL    TurtleBot3 model (default: waffle_pi)"
  echo "  --slam           Launch SLAM (default: SLAM Toolbox)"
  echo "  --slam-backend B Backend: slam_toolbox | cartographer (default: slam_toolbox)"
    echo "  --explore        Launch autonomous exploration (requires --slam)"
    echo "  --speed FACTOR   Simulation speed multiplier (default: 1.0)"
    echo "                   e.g., --speed 2.0 for 2x speed"
    echo "  -h, --help       Show this help"
    echo ""
    echo "Examples:"
    echo "  $0                           # Basic world"
    echo "  $0 --slam                    # With SLAM Toolbox"
    echo "  $0 --slam --explore          # With SLAM + exploration"
    echo "  $0 --slam --explore --speed 2.0  # 2x simulation speed"
    echo "  $0 --headless --speed 3.0    # Headless + 3x speed (fastest)"
}

# Parse arguments
while [[ $# -gt 0 ]]; do
  case "$1" in
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
    --slam-backend)
      SLAM_BACKEND="${2:-slam_toolbox}"; shift 2;;
    --explore)
      AUTO_EXPLORE="true"; shift 1;;
    --speed)
      SIM_SPEED="${2:-1.0}"; shift 2;;
    -h|--help)
      print_help; exit 0;;
    *)
      echo "Unknown arg: $1" >&2
      echo "Run: $0 --help" >&2
      exit 2;;
  esac
done

cd "$DOCKER_DIR"

echo "=========================================="
echo "Launch TurtleBot3 World"
echo "=========================================="
echo "Spawn position: ($X_POSE, $Y_POSE)"
echo "GUI:            $GUI"
echo "Sim speed:      ${SIM_SPEED}x"
echo "SLAM:           $LAUNCH_SLAM"
if [[ "$LAUNCH_SLAM" == "true" ]]; then
  echo "SLAM backend:   $SLAM_BACKEND"
fi
echo "Auto-explore:   $AUTO_EXPLORE"
echo "=========================================="
echo ""

# Validate auto-explore requires SLAM
if [[ "$AUTO_EXPLORE" == "true" ]] && [[ "$LAUNCH_SLAM" != "true" ]]; then
  echo "ERROR: --explore requires --slam flag"
  echo "Usage: $0 --slam --explore"
  exit 1
fi

# Launch TurtleBot3 World
echo "Starting TurtleBot3 World..."

# Set simulation speed via Gazebo physics
docker compose exec -d agv_sim bash -lc "\
  source /opt/ros/jazzy/setup.bash && \
  export TURTLEBOT3_MODEL=${MODEL} && \
  ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py \
    x_pose:=${X_POSE} y_pose:=${Y_POSE}
"

# Wait for Gazebo to start, then set simulation speed
if [[ "$SIM_SPEED" != "1.0" ]]; then
  echo "Setting simulation speed to ${SIM_SPEED}x..."
  sleep 5
  docker compose exec -d agv_sim bash -lc "\
    source /opt/ros/jazzy/setup.bash && \
    gz service -s /world/turtlebot3_world/set_physics \
      --reqtype gz.msgs.Physics \
      --reptype gz.msgs.Boolean \
      --timeout 5000 \
      --req 'real_time_factor: ${SIM_SPEED}'
  " 2>/dev/null || echo "Note: Could not set sim speed (Gazebo Classic doesn't support this)"
fi

# If SLAM is requested
if [[ "$LAUNCH_SLAM" == "true" ]]; then
  echo ""
  echo "Waiting for world to initialize..."
  sleep 8
  
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
    exit 1
  fi
  
  sleep 2

  if [[ "$SLAM_BACKEND" == "cartographer" ]]; then
    echo ""
    echo "Launching Cartographer SLAM (${SLAM_BACKEND})..."

    CONFIG_DIR="/ros2_ws/config"
    CONFIG_FILE="turtlebot3_slam_tuned.lua"

    if [[ "$AUTO_EXPLORE" == "true" ]]; then
      echo "Launching Cartographer in background..."
      docker compose exec -d agv_sim bash -lc "\
        source /opt/ros/jazzy/setup.bash && \
        source /ros2_ws/install/setup.bash 2>/dev/null || true && \
        export TURTLEBOT3_MODEL=${MODEL} && \
        ros2 launch turtlebot3_cartographer cartographer.launch.py \
          use_sim_time:=true \
          cartographer_config_dir:=${CONFIG_DIR} \
          configuration_basename:=${CONFIG_FILE}
      "

      echo ""
      echo "Waiting for Cartographer to initialize..."
      sleep 5

      # Wait for /map topic
      echo "Checking if /map topic is available..."
      RETRIES=0
      MAX_RETRIES=30
      while [[ $RETRIES -lt $MAX_RETRIES ]]; do
        if docker compose exec agv_sim bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic list 2>/dev/null | grep -q '/map'" 2>/dev/null; then
          echo '✓ /map topic ready!'
          break
        fi
        echo "  Waiting for /map topic... ($((RETRIES+1))/$MAX_RETRIES)"
        sleep 2
        RETRIES=$((RETRIES+1))
      done

      if [[ $RETRIES -eq $MAX_RETRIES ]]; then
        echo "ERROR: /map topic not available"
        exit 1
      fi

      EXPLORE_PKG="agv_auto_explore"
      EXPLORE_LAUNCH="auto_exploration.launch.py"

      echo ""
      echo "Building ${EXPLORE_PKG} package if needed..."
      docker compose exec agv_sim bash -lc "\
        source /opt/ros/jazzy/setup.bash && \
        cd /ros2_ws && \
        if [ ! -f /ros2_ws/install/${EXPLORE_PKG}/share/${EXPLORE_PKG}/package.xml ]; then \
          echo 'Building ${EXPLORE_PKG} package...'; \
          colcon build --symlink-install --packages-select ${EXPLORE_PKG}; \
        fi
      "

      echo ""
      echo "Launching Nav2 navigation stack..."
      docker compose exec -d agv_sim bash -lc "\
        source /opt/ros/jazzy/setup.bash && \
        export TURTLEBOT3_MODEL=${MODEL} && \
        ros2 launch turtlebot3_navigation2 navigation2.launch.py \
          use_sim_time:=true
      "

      echo ""
      echo "Waiting for Nav2 to initialize..."
      sleep 8

      echo ""
      echo ""
      echo "Launching autonomous exploration (${EXPLORE_PKG})..."
      docker compose exec agv_sim bash -lc "\
        source /opt/ros/jazzy/setup.bash && \
        source /ros2_ws/install/setup.bash && \
        echo '' && \
        echo '========================================' && \
        echo 'Autonomous Exploration Started!' && \
        echo 'Package: ${EXPLORE_PKG}' && \
        echo 'SLAM: Cartographer' && \
        echo 'Nav2: Enabled' && \
        echo '========================================' && \
        echo '' && \
        ros2 launch ${EXPLORE_PKG} ${EXPLORE_LAUNCH} \
          use_sim_time:=true
      "
    else
      # Cartographer only (foreground)
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
    fi
  else
    echo ""
    echo "Launching SLAM Toolbox..."

    if [[ "$AUTO_EXPLORE" == "true" ]]; then
      # Launch SLAM Toolbox in background (async mode for online SLAM)
      echo "Launching SLAM Toolbox in background..."
      docker compose exec -d agv_sim bash -lc "\
        source /opt/ros/jazzy/setup.bash && \
        source /ros2_ws/install/setup.bash 2>/dev/null || true && \
        export TURTLEBOT3_MODEL=${MODEL} && \
        ros2 launch slam_toolbox online_async_launch.py \
          use_sim_time:=true
      "

      echo ""
      echo "Waiting for SLAM Toolbox to initialize..."
      sleep 5

      # Wait for /map topic
      echo "Checking if /map topic is available..."
      RETRIES=0
      MAX_RETRIES=30
      while [[ $RETRIES -lt $MAX_RETRIES ]]; do
        if docker compose exec agv_sim bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic list 2>/dev/null | grep -q '/map'" 2>/dev/null; then
          echo '✓ /map topic ready!'
          break
        fi
        echo "  Waiting for /map topic... ($((RETRIES+1))/$MAX_RETRIES)"
        sleep 2
        RETRIES=$((RETRIES+1))
      done

      if [[ $RETRIES -eq $MAX_RETRIES ]]; then
        echo "ERROR: /map topic not available"
        exit 1
      fi

      EXPLORE_PKG="agv_auto_explore"
      EXPLORE_LAUNCH="auto_exploration.launch.py"

      echo ""
      echo "Building ${EXPLORE_PKG} package if needed..."
      docker compose exec agv_sim bash -lc "\
        source /opt/ros/jazzy/setup.bash && \
        cd /ros2_ws && \
        if [ ! -f /ros2_ws/install/${EXPLORE_PKG}/share/${EXPLORE_PKG}/package.xml ]; then \
          echo 'Building ${EXPLORE_PKG} package...'; \
          colcon build --symlink-install --packages-select ${EXPLORE_PKG}; \
        fi
      "

      echo ""
      echo "Launching Nav2 navigation stack..."
      docker compose exec -d agv_sim bash -lc "\
        source /opt/ros/jazzy/setup.bash && \
        export TURTLEBOT3_MODEL=${MODEL} && \
        ros2 launch turtlebot3_navigation2 navigation2.launch.py \
          use_sim_time:=true
      "

      echo ""
      echo "Waiting for Nav2 to initialize..."
      sleep 8

      echo ""
      echo ""
      echo "Launching autonomous exploration (${EXPLORE_PKG})..."
      docker compose exec agv_sim bash -lc "\
        source /opt/ros/jazzy/setup.bash && \
        source /ros2_ws/install/setup.bash && \
        echo '' && \
        echo '========================================' && \
        echo 'Autonomous Exploration Started!' && \
        echo 'Package: ${EXPLORE_PKG}' && \
        echo 'SLAM: SLAM Toolbox' && \
        echo 'Nav2: Enabled' && \
        echo '========================================' && \
        echo '' && \
        ros2 launch ${EXPLORE_PKG} ${EXPLORE_LAUNCH} \
          use_sim_time:=true
      "
    else
      # Launch SLAM Toolbox in foreground
      docker compose exec agv_sim bash -lc "\
        source /opt/ros/jazzy/setup.bash && \
        source /ros2_ws/install/setup.bash 2>/dev/null || true && \
        export TURTLEBOT3_MODEL=${MODEL} && \
        echo '' && \
        echo '========================================' && \
        echo 'SLAM Toolbox Ready!' && \
        echo '========================================' && \
        echo '' && \
        ros2 launch slam_toolbox online_async_launch.py \
          use_sim_time:=true
      "
    fi
  fi
else
  echo ""
  echo "=========================================="
  echo "World launched successfully!"
  echo "To add SLAM: --slam"
  echo "To add exploration: --slam --explore"
  echo "=========================================="
  
  echo ""
  echo "Press Ctrl+C to stop the simulation"
  wait
fi
