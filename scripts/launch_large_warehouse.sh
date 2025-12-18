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
#   ./scripts/launch_large_warehouse.sh --slam --auto-explore # Launch with SLAM and autonomous exploration
#   ./scripts/launch_large_warehouse.sh --slam --auto-explore --log # Enable logging to data/logs/
#
# Spawn locations:
#   center   - Center of warehouse (0, 0)
#   charging - Near charging station (12, -10)
#
# Logs:
#   When --log is enabled, logs are saved to data/logs/<timestamp>/
#   - gazebo.log       - Gazebo simulation logs
#   - cartographer.log - SLAM logs
#   - nav2.log         - Navigation stack logs
#   - exploration.log  - Auto-exploration logs
#
# Notes:
# - Requires container running: ./scripts/run_dev.sh up

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AGV_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
DOCKER_DIR="$AGV_DIR/docker"
LOGS_DIR="$AGV_DIR/data/logs"

# Default values
LOCATION="center"
GUI="true"
X_POSE=""
Y_POSE=""
MODEL="${TURTLEBOT3_MODEL:-waffle_pi}"
ROBOT_TYPE="turtlebot3"  # "turtlebot3" or "tugbot"
LAUNCH_SLAM="false"
SLAM_CONFIG="tuned"  # "tuned" or "default"
AUTO_EXPLORE="false"
ENABLE_LOGGING="false"
LOG_SESSION_DIR=""

# Predefined spawn locations
declare -A SPAWN_X
declare -A SPAWN_Y
SPAWN_X["center"]="0.0"
SPAWN_Y["center"]="0.0"
SPAWN_X["charging"]="12.0"
SPAWN_Y["charging"]="-10.0"
SPAWN_X["shelves"]="3.4"
SPAWN_Y["shelves"]="-12.96"
SPAWN_X["entrance"]="0.0"
SPAWN_Y["entrance"]="20.0"

print_help() {
    echo "Launch Robot in Large Warehouse World"
    echo ""
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --location LOC   Spawn location: center | charging | shelves | entrance (default: center)"
    echo "  --x X_POSE       Custom X position (overrides --location)"
    echo "  --y Y_POSE       Custom Y position (overrides --location)"
    echo "  --gui true|false Enable/disable GUI (default: true)"
    echo "  --headless       Run without GUI (same as --gui false)"
    echo "  --model MODEL    TurtleBot3 model (default: waffle_pi)"
    echo "  --tugbot         Use Tugbot industrial AGV instead of TurtleBot3"
    echo "  --slam           Launch SLAM with world"
    echo "  --slam-default   Use default SLAM config (instead of tuned)"
    echo "  --auto-explore   Launch autonomous exploration (requires --slam)"
    echo "  --log            Enable logging to data/logs/<timestamp>/"
    echo "  -h, --help       Show this help"
    echo ""
    echo "Robot Types:"
    echo "  TurtleBot3       - Small educational robot (default)"
    echo "  Tugbot (--tugbot) - Industrial AGV for large warehouses"
    echo ""
    echo "Spawn locations:"
    echo "  center   - Center of warehouse (0, 0)"
    echo "  charging - Near charging station (12, -10)"
    echo "  shelves  - Between large shelf columns (3.4, -12.96)"
    echo "  entrance - At warehouse entrance (0, 20)"
    echo ""
    echo "Examples:"
    echo "  $0                           # TurtleBot3 at center"
    echo "  $0 --tugbot                  # Tugbot at center"
    echo "  $0 --tugbot --slam --auto-explore  # Tugbot with exploration"
    echo "  $0 --location charging       # Spawn near charging station"
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
    --tugbot)
      ROBOT_TYPE="tugbot"; shift 1;;
    --slam)
      LAUNCH_SLAM="true"; shift 1;;
    --slam-default)
      SLAM_CONFIG="default"; shift 1;;
    --auto-explore)
      AUTO_EXPLORE="true"; shift 1;;
    --log)
      ENABLE_LOGGING="true"; shift 1;;
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
    echo "Available: center, charging, shelves, entrance" >&2
    exit 1
fi

cd "$DOCKER_DIR"

# Setup logging if enabled (after coordinates are resolved)
if [[ "$ENABLE_LOGGING" == "true" ]]; then
  TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
  LOG_SESSION_DIR="$LOGS_DIR/$TIMESTAMP"
  mkdir -p "$LOG_SESSION_DIR"
  
  # Create symlink to latest logs
  rm -f "$LOGS_DIR/latest"
  ln -sf "$TIMESTAMP" "$LOGS_DIR/latest"
  
  # Save session info
  cat > "$LOG_SESSION_DIR/session_info.txt" << EOF
Session: $TIMESTAMP
Location: ($FINAL_X, $FINAL_Y) [${LOCATION}]
GUI: $GUI
Model: $MODEL
SLAM: $LAUNCH_SLAM
SLAM Config: $SLAM_CONFIG
Auto-Explore: $AUTO_EXPLORE
Started: $(date)
EOF
fi

echo "=========================================="
echo "Launch Large Warehouse Simulation"
echo "=========================================="
echo "Robot type:     $ROBOT_TYPE"
if [[ "$ROBOT_TYPE" == "turtlebot3" ]]; then
  echo "Model:          $MODEL"
fi
echo "Spawn location: ($FINAL_X, $FINAL_Y)"
echo "GUI:            $GUI"
echo "SLAM:           $LAUNCH_SLAM"
if [[ "$LAUNCH_SLAM" == "true" ]]; then
  echo "SLAM config:    $SLAM_CONFIG"
fi
echo "Auto-explore:   $AUTO_EXPLORE"
if [[ "$ENABLE_LOGGING" == "true" ]]; then
  echo "Logging:        $LOG_SESSION_DIR"
fi
echo "=========================================="
echo ""

# Validate auto-explore requires SLAM
if [[ "$AUTO_EXPLORE" == "true" ]] && [[ "$LAUNCH_SLAM" != "true" ]]; then
  echo "ERROR: --auto-explore requires --slam flag"
  echo "Usage: $0 --slam --auto-explore"
  exit 1
fi

# Launch world in background
echo "Starting large warehouse world..."

# Determine log redirect
if [[ "$ENABLE_LOGGING" == "true" ]]; then
  GAZEBO_LOG_CMD="2>&1 | tee /ros2_ws/data/logs/${TIMESTAMP}/gazebo.log"
else
  GAZEBO_LOG_CMD=""
fi

if [[ "$ROBOT_TYPE" == "tugbot" ]]; then
  # Launch with Tugbot (simple approach)
  docker compose exec -d agv_sim bash -lc "\
    source /opt/ros/jazzy/setup.bash && \
    cd /ros2_ws && \
    if [ ! -f /ros2_ws/install/turtlebot3_large_warehouse_sim/share/turtlebot3_large_warehouse_sim/package.xml ]; then \
      echo '[launch_large_warehouse] Building warehouse package...'; \
      colcon build --symlink-install --packages-select turtlebot3_large_warehouse_sim; \
    fi && \
    source /ros2_ws/install/setup.bash && \
    echo 'Launching Tugbot in warehouse...' && \
    ros2 launch turtlebot3_large_warehouse_sim tugbot_large_warehouse.launch.py \
      gui:=${GUI} x_pose:=${FINAL_X} y_pose:=${FINAL_Y} ${GAZEBO_LOG_CMD}
  "
else
  # Launch with TurtleBot3 (original)
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
      gui:=${GUI} x_pose:=${FINAL_X} y_pose:=${FINAL_Y} ${GAZEBO_LOG_CMD}
  "
fi

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
  
  # If auto-explore is enabled, launch Cartographer in background and then exploration
  if [[ "$AUTO_EXPLORE" == "true" ]]; then
    echo "Launching Cartographer SLAM in background..."
    
    # Determine log redirect for Cartographer
    if [[ "$ENABLE_LOGGING" == "true" ]]; then
      CARTO_LOG_CMD="2>&1 | tee /ros2_ws/data/logs/${TIMESTAMP}/cartographer.log"
    else
      CARTO_LOG_CMD=""
    fi
    
    # Use Cartographer for both TurtleBot3 and Tugbot
    docker compose exec -d agv_sim bash -lc "\
      source /opt/ros/jazzy/setup.bash && \
      source /ros2_ws/install/setup.bash 2>/dev/null || true && \
      export TURTLEBOT3_MODEL=${MODEL} && \
      ros2 launch turtlebot3_cartographer cartographer.launch.py \
        use_sim_time:=true \
        cartographer_config_dir:=${CONFIG_DIR} \
        configuration_basename:=${CONFIG_FILE} ${CARTO_LOG_CMD}
    "
    
    echo ""
    echo "Waiting for Cartographer and Nav2 to initialize..."
    sleep 5
    
    # Wait for /map topic to be available
    echo "Checking if /map topic is available..."
    RETRIES=0
    MAX_RETRIES=30
    while [[ $RETRIES -lt $MAX_RETRIES ]]; do
      if docker compose exec agv_sim bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic list 2>/dev/null | grep -q '/map'" 2>/dev/null; then
        echo "✓ /map topic ready!"
        break
      fi
      echo "  Waiting for /map topic... ($((RETRIES+1))/$MAX_RETRIES)"
      sleep 2
      RETRIES=$((RETRIES+1))
    done
    
    if [[ $RETRIES -eq $MAX_RETRIES ]]; then
      echo "ERROR: /map topic not available after $MAX_RETRIES retries"
      echo "Cartographer may not have started correctly. Check logs."
      exit 1
    fi
    
    echo ""
    echo "Building agv_auto_explore package..."
    docker compose exec agv_sim bash -lc "\
      source /opt/ros/jazzy/setup.bash && \
      cd /ros2_ws && \
      echo '[launch_large_warehouse] Building agv_auto_explore package...'; \
      colcon build --symlink-install --packages-select agv_auto_explore
    "
    
    echo ""
    echo "Launching Nav2 navigation stack..."
    
    # Determine log redirect for Nav2
    if [[ "$ENABLE_LOGGING" == "true" ]]; then
      NAV2_LOG_CMD="2>&1 | tee /ros2_ws/data/logs/${TIMESTAMP}/nav2.log"
    else
      NAV2_LOG_CMD=""
    fi
    
    # Use Nav2 with appropriate params for each robot type
    # Both use turtlebot3_navigation2 launch (includes RViz) with different params
    if [[ "$ROBOT_TYPE" == "tugbot" ]]; then
      # Tugbot: Use custom params with larger robot_radius
      echo "Using Tugbot Nav2 params (robot_radius=0.35m)"
      docker compose exec -d agv_sim bash -lc "\
        source /opt/ros/jazzy/setup.bash && \
        source /ros2_ws/install/setup.bash 2>/dev/null || true && \
        export TURTLEBOT3_MODEL=waffle_pi && \
        ros2 launch turtlebot3_navigation2 navigation2.launch.py \
          use_sim_time:=true \
          params_file:=/ros2_ws/src/agv_auto_explore/config/tugbot_nav2_params.yaml ${NAV2_LOG_CMD}
      "
    else
      # TurtleBot3: Use default waffle_pi params
      docker compose exec -d agv_sim bash -lc "\
        source /opt/ros/jazzy/setup.bash && \
        export TURTLEBOT3_MODEL=${MODEL} && \
        ros2 launch turtlebot3_navigation2 navigation2.launch.py \
          use_sim_time:=true ${NAV2_LOG_CMD}
      "
    fi
    
    echo ""
    echo "Waiting for Nav2 to initialize..."
    sleep 8
    
    echo ""
    echo "Launching autonomous exploration node..."
    
    # Determine log redirect for exploration
    if [[ "$ENABLE_LOGGING" == "true" ]]; then
      EXPLORE_LOG_CMD="2>&1 | tee /ros2_ws/data/logs/${TIMESTAMP}/exploration.log"
      echo "📝 Logs will be saved to: $LOG_SESSION_DIR/"
      echo "   - gazebo.log"
      echo "   - cartographer.log"
      echo "   - nav2.log"
      echo "   - exploration.log"
      echo ""
      echo "💡 Tip: tail -f $LOG_SESSION_DIR/exploration.log"
    else
      EXPLORE_LOG_CMD=""
    fi
    
    docker compose exec agv_sim bash -lc "\
      source /opt/ros/jazzy/setup.bash && \
      source /ros2_ws/install/setup.bash && \
      echo '' && \
      echo '========================================' && \
      echo 'Autonomous Exploration Started!' && \
      echo 'SLAM config: ${CONFIG_DIR}/${CONFIG_FILE}' && \
      echo 'Nav2: Enabled' && \
      echo '========================================' && \
      echo '' && \
      ros2 launch agv_auto_explore auto_exploration.launch.py \
        use_sim_time:=true ${EXPLORE_LOG_CMD}
    "
  else
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
  fi
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
