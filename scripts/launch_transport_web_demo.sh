#!/usr/bin/env bash
set -euo pipefail

# Launch AGV Transport Web Demo - Complete System
# Includes: Gazebo warehouse + Tugbot + Nav2 + Transport Task Manager + Web Dashboard
#
# Usage:
#   ./scripts/launch_transport_web_demo.sh                    # Launch at entrance with web dashboard
#   ./scripts/launch_transport_web_demo.sh --location charging # Launch near charging station
#   ./scripts/launch_transport_web_demo.sh --headless         # Without Gazebo GUI
#   ./scripts/launch_transport_web_demo.sh --skip-build       # Skip package rebuild
#   ./scripts/launch_transport_web_demo.sh --web-port 8080    # Custom web server port
#   ./scripts/launch_transport_web_demo.sh --no-web           # Skip web dashboard

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AGV_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
DOCKER_DIR="$AGV_DIR/docker"

# Default values
LOCATION="entrance"
GUI="true"
X_POSE=""
Y_POSE=""
SKIP_BUILD="false"
WEB_PORT="8000"
LAUNCH_WEB="true"
WEB_HOST="0.0.0.0"
DEBUG_MODE="false"

# Predefined spawn locations (world coordinates)
declare -A SPAWN_X
declare -A SPAWN_Y
SPAWN_X["center"]="0.0"
SPAWN_Y["center"]="0.0"
SPAWN_X["charging"]="12.0"
SPAWN_Y["charging"]="-10.0"
SPAWN_X["entrance"]="0.0"
SPAWN_Y["entrance"]="20.0"

# Map coordinates for AMCL initial pose
declare -A MAP_X_COORDS
declare -A MAP_Y_COORDS
MAP_X_COORDS["entrance"]="0.065"
MAP_Y_COORDS["entrance"]="0.097"
MAP_X_COORDS["center"]="0.0"
MAP_Y_COORDS["center"]="0.0"
MAP_X_COORDS["charging"]="13.157"
MAP_Y_COORDS["charging"]="-30.282"

print_help() {
    echo "Launch AGV Transport Web Demo - Complete System"
    echo ""
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --location LOC   Spawn location: center | charging | entrance (default: entrance)"
    echo "  --x X_POSE       Custom X position (world coordinates)"
    echo "  --y Y_POSE       Custom Y position (world coordinates)"
    echo "  --headless       Run without Gazebo GUI"
    echo "  --skip-build     Skip rebuilding packages"
    echo "  --web-port PORT  Web dashboard port (default: 8000)"
    echo "  --web-host HOST  Web dashboard host (default: 0.0.0.0)"
    echo "  --no-web         Skip launching web dashboard"
    echo "  --debug          Enable debug mode for web dashboard"
    echo "  -h, --help       Show this help"
    echo ""
    echo "This launches:"
    echo "  1. Gazebo with tugbot_warehouse world"
    echo "  2. Tugbot robot"
    echo "  3. Nav2 with static map (warehouse_tugbot.yaml)"
    echo "  4. RViz2 for visualization"
    echo "  5. Transport Task Manager node"
    echo "  6. FastAPI Web Dashboard (unless --no-web)"
    echo ""
    echo "Available Stations (for transport orders):"
    echo "  - charging_station  (13.157, -30.282)"
    echo "  - dock_in_1         (-0.143, -20.679)"
    echo "  - dock_in_2         (-3.756, -6.553)"
    echo "  - dock_out          (-0.181, -43.405)"
    echo ""
    echo "Web Dashboard will be available at: http://localhost:$WEB_PORT"
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
    --skip-build)
      SKIP_BUILD="true"; shift 1;;
    --web-port)
      WEB_PORT="${2:-8000}"; shift 2;;
    --web-host)
      WEB_HOST="${2:-0.0.0.0}"; shift 2;;
    --no-web)
      LAUNCH_WEB="false"; shift 1;;
    --debug)
      DEBUG_MODE="true"; shift 1;;
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
    MAP_X="$X_POSE"
    MAP_Y="$Y_POSE"
elif [[ -n "${SPAWN_X[$LOCATION]:-}" ]]; then
    FINAL_X="${SPAWN_X[$LOCATION]}"
    FINAL_Y="${SPAWN_Y[$LOCATION]}"
    MAP_X="${MAP_X_COORDS[$LOCATION]}"
    MAP_Y="${MAP_Y_COORDS[$LOCATION]}"
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

echo ""
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║        AGV Transport Web Demo - Complete System Launch       ║"
echo "╠══════════════════════════════════════════════════════════════╣"
echo "║  Robot:          Tugbot                                      ║"
echo "║  Spawn location: ($FINAL_X, $FINAL_Y)                        "
echo "║  GUI:            $GUI                                        "
echo "║  Map:            warehouse_tugbot.yaml                       ║"
echo "║  Web Dashboard:  $LAUNCH_WEB (Port: $WEB_PORT)               "
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""

# Function to check if a process is running
check_process_running() {
    local process_name="$1"
    local max_retries="${2:-30}"
    local retry_interval="${3:-2}"
    
    echo "Checking if $process_name is running..."
    local retries=0
    while [[ $retries -lt $max_retries ]]; do
        if docker compose exec agv_sim bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic list 2>/dev/null | grep -q '$process_name'" 2>/dev/null; then
            echo "✓ $process_name ready!"
            return 0
        fi
        echo "  Waiting for $process_name... ($((retries+1))/$max_retries)"
        sleep $retry_interval
        retries=$((retries+1))
    done
    
    echo "ERROR: $process_name not available after $max_retries retries"
    return 1
}

# Function to verify system health
verify_system_health() {
    echo ""
    echo "Verifying system health..."
    
    # Check essential topics
    local topics=("/scan" "/map" "/agv/transport_status")
    for topic in "${topics[@]}"; do
        if ! check_process_running "$topic" 10 1; then
            echo "WARNING: Essential topic $topic not available"
        fi
    done
    
    # Check if robot is localized
    echo "Checking robot localization..."
    if docker compose exec agv_sim bash -c "source /opt/ros/jazzy/setup.bash && timeout 5 ros2 topic echo /amcl_pose --once" >/dev/null 2>&1; then
        echo "✓ Robot localization active"
    else
        echo "WARNING: Robot localization may not be working"
    fi
    
    echo "✓ System health check complete"
}

# Step 1: Build packages (if not skipped)
if [[ "$SKIP_BUILD" == "false" ]]; then
    echo "Step 1: Building required packages..."
    docker compose exec agv_sim bash -lc "
        source /opt/ros/jazzy/setup.bash && \
        cd /ros2_ws && \
        colcon build --symlink-install --packages-select agv_transport agv_transport_web && \
        echo '✓ Packages built successfully'
    "
else
    echo "Step 1: Skipping build (--skip-build flag set)"
fi

# Step 2: Launch Gazebo warehouse + Tugbot
echo ""
echo "Step 2: Starting Gazebo warehouse with Tugbot..."
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

# Check if robot topics are available
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

# Step 3: Launch Nav2 with static map
echo ""
echo "Step 3: Starting Nav2 with static map and AMCL..."
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

# Check if map is available using the same method as original script
echo "Checking if map is available..."
RETRIES=0
MAX_RETRIES=30
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

# Step 4: Start RViz2
echo ""
echo "Step 4: Starting RViz2..."
docker compose exec -d agv_sim bash -c "
    source /opt/ros/jazzy/setup.bash && \
    source /ros2_ws/install/setup.bash 2>/dev/null || true && \
    export TURTLEBOT3_MODEL=waffle_pi && \
    ros2 launch nav2_bringup rviz_launch.py
"
sleep 3

# Step 5: Set initial pose for AMCL
echo ""
echo "Step 5: Setting initial pose for AMCL..."
echo "  Location: $LOCATION"
echo "  World: (${FINAL_X}, ${FINAL_Y})"
echo "  Map:   (${MAP_X}, ${MAP_Y})"

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
sleep 3
echo "✓ Initial pose set"

# Step 6: Launch Transport Task Manager
echo ""
echo "Step 6: Starting Transport Task Manager..."
docker compose exec -d agv_sim bash -lc "
    source /opt/ros/jazzy/setup.bash && \
    source /ros2_ws/install/setup.bash && \
    echo 'Launching Transport Task Manager...' && \
    ros2 launch agv_transport transport_manager.launch.py \
      use_sim_time:=true \
      config_path:=/ros2_ws/src/agv_transport/config/warehouse_stations.yaml
"

sleep 3

# Check if transport manager is running
echo "Checking if Transport Task Manager is running..."
RETRIES=0
MAX_RETRIES=10
while [[ $RETRIES -lt $MAX_RETRIES ]]; do
    if docker compose exec agv_sim bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic list 2>/dev/null | grep -q '/agv/transport_status'" 2>/dev/null; then
        echo "✓ Transport Task Manager ready!"
        break
    fi
    echo "  Waiting for transport manager... ($((RETRIES+1))/$MAX_RETRIES)"
    sleep 2
    RETRIES=$((RETRIES+1))
done

if [[ $RETRIES -eq $MAX_RETRIES ]]; then
    echo "WARNING: Transport Task Manager may not be running properly"
    echo "Check logs with: docker compose exec agv_sim bash -c 'ros2 node list'"
fi

# Step 7: Launch Web Dashboard (if enabled)
if [[ "$LAUNCH_WEB" == "true" ]]; then
    echo ""
    echo "Step 7: Starting FastAPI Web Dashboard..."
    
    # Set environment variables for web dashboard
    WEB_ENV_VARS=""
    WEB_ENV_VARS+="AGV_WEB_HOST=$WEB_HOST "
    WEB_ENV_VARS+="AGV_WEB_PORT=$WEB_PORT "
    WEB_ENV_VARS+="AGV_WEB_LOG_LEVEL=info "
    WEB_ENV_VARS+="AGV_ROS2_NODE_NAME=agv_transport_web_dashboard "
    WEB_ENV_VARS+="AGV_STATION_CONFIG_PATH=/ros2_ws/src/agv_transport/config/warehouse_stations.yaml "
    
    if [[ "$DEBUG_MODE" == "true" ]]; then
        WEB_ENV_VARS+="AGV_DEBUG_MODE=true "
        WEB_ENV_VARS+="AGV_WEB_LOG_LEVEL=debug "
    fi
    
    # Launch web dashboard
    docker compose exec -d agv_sim bash -lc "
        source /opt/ros/jazzy/setup.bash && \
        source /ros2_ws/install/setup.bash && \
        cd /ros2_ws/src/agv_transport_web && \
        echo 'Starting FastAPI Web Dashboard...' && \
        $WEB_ENV_VARS python3 -m uvicorn agv_transport_web.app:app --host $WEB_HOST --port $WEB_PORT --log-level info
    "
    
    sleep 8
    
    # Check if web dashboard is running
    echo "Checking if web dashboard is accessible..."
    web_retries=0
    web_max_retries=15
    while [[ $web_retries -lt $web_max_retries ]]; do
        if docker compose exec agv_sim bash -c "curl -s http://localhost:$WEB_PORT/health >/dev/null 2>&1"; then
            echo "✓ Web Dashboard ready at http://localhost:$WEB_PORT"
            break
        fi
        echo "  Waiting for web dashboard... ($((web_retries+1))/$web_max_retries)"
        sleep 2
        web_retries=$((web_retries+1))
    done
    
    if [[ $web_retries -eq $web_max_retries ]]; then
        echo "WARNING: Web Dashboard may not be accessible"
        echo "Checking web dashboard logs..."
        docker compose exec agv_sim bash -c "ps aux | grep python | grep -v grep" || true
        echo ""
        echo "Checking if port $WEB_PORT is in use..."
        docker compose exec agv_sim bash -c "ss -tlnp | grep :$WEB_PORT" || echo "Port $WEB_PORT appears to be free"
        echo ""
        echo "Trying to start web dashboard manually for debugging..."
        docker compose exec agv_sim bash -c "
            source /opt/ros/jazzy/setup.bash && \
            source /ros2_ws/install/setup.bash && \
            cd /ros2_ws/src/agv_transport_web && \
            $WEB_ENV_VARS python3 -c 'from agv_transport_web.app import app; print(\"App imported successfully\")'
        " || echo "Failed to import web app - check dependencies"
    fi
else
    echo ""
    echo "Step 7: Skipping web dashboard (--no-web flag set)"
fi

# Step 8: System health verification
verify_system_health

echo ""
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║                    ✓ System Ready!                           ║"
echo "╠══════════════════════════════════════════════════════════════╣"
echo "║  Gazebo:    Running with tugbot_warehouse                    ║"
echo "║  Robot:     Tugbot at ($FINAL_X, $FINAL_Y)                   "
echo "║  Nav2:      Running with static map + AMCL                   ║"
echo "║  RViz2:     Running for visualization                        ║"
echo "║  Transport: Task Manager running                             ║"
if [[ "$LAUNCH_WEB" == "true" ]]; then
echo "║  Web:       Dashboard at http://localhost:$WEB_PORT          "
fi
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""
echo "Available Stations:"
echo "  - charging_station  (13.157, -30.282)"
echo "  - dock_in_1         (-0.143, -20.679)"
echo "  - dock_in_2         (-3.756, -6.553)"
echo "  - dock_out          (-0.181, -43.405)"
echo ""

if [[ "$LAUNCH_WEB" == "true" ]]; then
    echo "🌐 Web Dashboard: http://localhost:$WEB_PORT"
    echo "   - Real-time AGV status monitoring"
    echo "   - Task queue management"
    echo "   - Create transport orders via web form"
    echo "   - Health check: http://localhost:$WEB_PORT/health/detailed"
    echo ""
fi

echo "📋 Manual Commands:"
echo "To send a transport order via ROS2:"
echo "  docker compose exec agv_sim bash -c '\\"
echo "    source /opt/ros/jazzy/setup.bash && \\"
echo "    source /ros2_ws/install/setup.bash && \\"
echo "    ros2 run agv_transport send_transport_order dock_in_1 dock_out'"
echo ""
echo "To monitor transport status:"
echo "  docker compose exec agv_sim bash -c '\\"
echo "    source /opt/ros/jazzy/setup.bash && \\"
echo "    ros2 topic echo /agv/transport_status'"
echo ""
echo "To reset transport manager:"
echo "  docker compose exec agv_sim bash -c '\\"
echo "    source /opt/ros/jazzy/setup.bash && \\"
echo "    ros2 service call /agv/reset_transport std_srvs/srv/Trigger'"
echo ""

if [[ "$LAUNCH_WEB" == "true" ]]; then
    echo "🔧 Web Dashboard Configuration:"
    echo "  Host: $WEB_HOST"
    echo "  Port: $WEB_PORT"
    echo "  Debug: $DEBUG_MODE"
    echo "  Config endpoints: /config, /config/reload, /config/validate"
    echo ""
fi

echo "Press Ctrl+C to stop all processes"
echo "══════════════════════════════════════════════════════════════"

# Function to handle cleanup on exit
cleanup() {
    echo ""
    echo "Shutting down system..."
    
    # Kill web dashboard if running
    if [[ "$LAUNCH_WEB" == "true" ]]; then
        echo "Stopping web dashboard..."
        docker compose exec agv_sim bash -c "pkill -f 'python -m agv_transport_web.app'" 2>/dev/null || true
    fi
    
    # Kill ROS2 processes
    echo "Stopping ROS2 processes..."
    docker compose exec agv_sim bash -c "pkill -f ros2" 2>/dev/null || true
    
    echo "Cleanup complete"
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

# Wait for user interrupt
wait