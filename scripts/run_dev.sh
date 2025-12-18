#!/bin/bash
# Development script for single container setup

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOCKER_DIR="$SCRIPT_DIR/../docker"

cd "$DOCKER_DIR"

case "$1" in
    build)
        docker compose build
        ;;
    up)
        docker compose up -d
        ;;
    down)
        docker compose down
        ;;
    enter|bash)
        docker compose exec agv_sim bash
        ;;
    logs)
        docker compose logs -f
        ;;
    build-ws)
        # Use login shell so ROS environment is sourced (ament_package, ros2, etc.)
        # Also ignore turtlebot3 packages in src/ to prefer /opt/ros/jazzy versions.
        # Clean build to ensure fresh compilation
        docker compose exec agv_sim bash -lc "source /opt/ros/jazzy/setup.bash && cd /ros2_ws && rm -rf build/ install/ && colcon build"
        ;;
    clean-build)
        PACKAGE_NAME="${2:-agv_auto_explore}"
        echo "Clean building package: ${PACKAGE_NAME}"
        docker compose exec agv_sim bash -lc "
            source /opt/ros/jazzy/setup.bash && 
            cd /ros2_ws && 
            rm -rf build/${PACKAGE_NAME} install/${PACKAGE_NAME} && 
            colcon build --packages-select ${PACKAGE_NAME}
        "
        ;;
    test)
        docker compose exec agv_sim bash -lc "source /opt/ros/jazzy/setup.bash && cd /ros2_ws && colcon test"
        ;;
    restart)
        docker compose restart
        ;;
    teleop)
        echo "Starting TurtleBot3 Teleop..."
        echo "Controls: w/x=linear, a/d=angular, s=stop, space=emergency stop"
        docker compose exec -it agv_sim bash -c "
            source /opt/ros/jazzy/setup.bash
            export TURTLEBOT3_MODEL=waffle_pi
            ros2 run turtlebot3_teleop teleop_keyboard
        "
        ;;
    save-map)
        MAP_NAME="${2:-map}"
        TIMESTAMP=$(date +%Y%m%d_%H%M%S)
        MAP_PATH="/ros2_ws/data/maps/${MAP_NAME}_${TIMESTAMP}"
        
        echo "=========================================="
        echo "Saving SLAM Map"
        echo "=========================================="
        echo "Map name: ${MAP_NAME}"
        echo "Timestamp: ${TIMESTAMP}"
        echo "Path: ${MAP_PATH}"kk
        echo ""
        
        # Check if /map topic is available
        echo "Checking if /map topic is available..."
        if ! docker compose exec agv_sim bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic list 2>/dev/null | grep -q '/map'"; then
            echo "ERROR: /map topic not found. Make sure SLAM is running."
            echo "Start SLAM first with: ./scripts/launch_turtlebot3_world.sh --slam  ros2 run teleop_twist_keyboard teleop_twist_keyboard  --"
            exit 1
        fi
        
        echo "✓ /map topic found. Saving map..."
        docker compose exec agv_sim bash -c "
            source /opt/ros/jazzy/setup.bash && \
            cd /ros2_ws/data/maps && \
            ros2 run nav2_map_server map_saver_cli -f ${MAP_NAME}_${TIMESTAMP}
        "
        
        # Check if files were created
        if docker compose exec agv_sim bash -c "[ -f ${MAP_PATH}.pgm ] && [ -f ${MAP_PATH}.yaml ]"; then
            echo "✓ Map saved successfully!"
            echo ""
            echo "Files created:"
            echo "  - ${MAP_NAME}_${TIMESTAMP}.pgm (map image)"
            echo "  - ${MAP_NAME}_${TIMESTAMP}.yaml (map metadata)"
            echo ""
            echo "Host path: AGV_2/data/maps/"
        else
            echo "ERROR: Map files not created. Check if SLAM is publishing /map topic."
            exit 1
        fi
        ;;
    save-map-carto)
        MAP_NAME="${2:-map_cartographer}"
        TIMESTAMP=$(date +%Y%m%d_%H%M%S)
        MAP_PATH="/ros2_ws/data/maps/${MAP_NAME}_${TIMESTAMP}"
        
        echo "=========================================="
        echo "Saving Cartographer Map + Pose Graph"
        echo "=========================================="
        echo "Map name: ${MAP_NAME}"
        echo "Timestamp: ${TIMESTAMP}"
        echo "Path: ${MAP_PATH}"
        echo ""
        
        # Check if Cartographer is running (check for write_state service)
        echo "Checking if Cartographer is running..."
        if ! docker compose exec agv_sim bash -c "source /opt/ros/jazzy/setup.bash && ros2 service list 2>/dev/null | grep -q '/write_state'"; then
            echo "ERROR: Cartographer not found. Make sure Cartographer SLAM is running."
            echo "Start Cartographer with: ./scripts/launch_turtlebot3_world.sh --slam --slam-backend cartographer"
            exit 1
        fi
        
        # Check if /map topic is available (with retry - Cartographer needs time to initialize)
        echo "Checking if /map topic is available..."
        RETRIES=0
        MAX_RETRIES=30
        MAP_READY=false
        
        while [[ $RETRIES -lt $MAX_RETRIES ]]; do
            if docker compose exec agv_sim bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic list 2>/dev/null | grep -q '/map'" 2>/dev/null; then
                # Also check if topic is actually publishing (not just listed)
                if docker compose exec agv_sim bash -c "source /opt/ros/jazzy/setup.bash && timeout 2 ros2 topic echo /map --once 2>/dev/null | head -1" 2>/dev/null | grep -q "header"; then
                    echo "✓ /map topic ready and publishing!"
                    MAP_READY=true
                    break
                fi
            fi
            echo "  Waiting for /map topic... ($((RETRIES+1))/$MAX_RETRIES)"
            sleep 2
            RETRIES=$((RETRIES+1))
        done
        
        if [[ "$MAP_READY" != "true" ]]; then
            echo ""
            echo "ERROR: /map topic not available after $MAX_RETRIES retries."
            echo "Cartographer may not be publishing map yet, or there's an issue with Cartographer."
            echo ""
            echo "Troubleshooting:"
            echo "  1. Check if Cartographer is running: docker compose exec agv_sim bash -c 'ros2 node list | grep cartographer'"
            echo "  2. Check Cartographer logs: docker compose logs agv_sim | grep cartographer"
            echo "  3. Make sure robot has moved enough for Cartographer to create first submap"
            exit 1
        fi
        
        echo ""
        echo "✓ Cartographer detected. Saving map..."
        
        # Step 1: Save occupancy grid map (.pgm/.yaml)
        echo ""
        echo "Step 1: Saving occupancy grid map..."
        docker compose exec agv_sim bash -c "
            source /opt/ros/jazzy/setup.bash && \
            cd /ros2_ws/data/maps && \
            ros2 run nav2_map_server map_saver_cli -f ${MAP_NAME}_${TIMESTAMP}
        "
        
        # Step 2: Save Cartographer pose graph state (.pbstream)
        echo ""
        echo "Step 2: Saving Cartographer pose graph state..."
        PBSTREAM_PATH="${MAP_PATH}.pbstream"
        docker compose exec agv_sim bash -c "
            source /opt/ros/jazzy/setup.bash && \
            ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
                '{filename: \"${PBSTREAM_PATH}\"}' 2>/dev/null || \
            echo 'WARN: write_state service call failed. Trying alternative method...'
        "
        
        # Alternative: use cartographer_pbstream_map_writer if service doesn't work
        if ! docker compose exec agv_sim bash -c "[ -f ${PBSTREAM_PATH} ]"; then
            echo "Trying alternative method to save pose graph..."
            docker compose exec agv_sim bash -c "
                source /opt/ros/jazzy/setup.bash && \
                ros2 run cartographer_ros cartographer_pbstream_map_writer \
                    -pbstream_filename ${PBSTREAM_PATH} \
                    -map_filestem ${MAP_PATH} 2>/dev/null || \
                echo 'WARN: Could not save pose graph state. Occupancy grid map saved.'
            "
        fi
        
        # Check if files were created
        echo ""
        if docker compose exec agv_sim bash -c "[ -f ${MAP_PATH}.pgm ] && [ -f ${MAP_PATH}.yaml ]"; then
            echo "✓ Occupancy grid map saved successfully!"
            echo ""
            echo "Files created:"
            echo "  - ${MAP_NAME}_${TIMESTAMP}.pgm (map image)"
            echo "  - ${MAP_NAME}_${TIMESTAMP}.yaml (map metadata)"
            
            if docker compose exec agv_sim bash -c "[ -f ${PBSTREAM_PATH} ]"; then
                echo "  - ${MAP_NAME}_${TIMESTAMP}.pbstream (Cartographer pose graph state)"
                echo ""
                echo "Note: .pbstream file can be loaded later to continue mapping or for localization."
            else
                echo ""
                echo "Note: Pose graph state (.pbstream) not saved. Occupancy grid map is still usable."
            fi
            
            echo ""
            echo "Host path: AGV_2/data/maps/"
        else
            echo "ERROR: Map files not created. Check if Cartographer is publishing /map topic."
            exit 1
        fi
        ;;
    fix-permissions)
        echo "Fixing permissions for workspace files..."
        docker compose exec agv_sim bash -c "
            chown -R $(id -u):$(id -g) /ros2_ws/data/ 2>/dev/null || true
            chown -R $(id -u):$(id -g) /ros2_ws/src/ 2>/dev/null || true
            chown -R $(id -u):$(id -g) /ros2_ws/config/ 2>/dev/null || true
        "
        echo "Permissions fixed!"
        ;;
    *)
        echo "Usage: $0 {build|up|down|enter|bash|logs|build-ws|clean-build|test|restart|teleop|save-map|save-map-carto|fix-permissions}"
        echo ""
        echo "Commands:"
        echo "  build              - Build Docker image"
        echo "  up                 - Start container in detached mode"
        echo "  down               - Stop and remove container"
        echo "  enter              - Enter the container (alias: bash)"
        echo "  logs               - Show container logs"
        echo "  build-ws           - Clean build entire ROS2 workspace"
        echo "  clean-build [pkg]  - Clean build specific package (default: agv_auto_explore)"
        echo "  test               - Run tests in workspace"
        echo "  restart            - Restart container"
        echo "  teleop             - Launch keyboard teleop control"
        echo "  save-map [name]    - Save current SLAM map (.pgm/.yaml) with timestamp (default: map)"
        echo "  save-map-carto [name] - Save Cartographer map + pose graph (.pgm/.yaml/.pbstream) (default: map_cartographer)"
        echo "  fix-permissions    - Fix file permissions for host access"
        exit 1
        ;;
esac
