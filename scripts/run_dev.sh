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
        docker compose exec agv_sim bash -lc "source /opt/ros/jazzy/setup.bash && cd /ros2_ws && colcon build"
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
        MAP_NAME="${2:-warehouse_map}"
        TIMESTAMP=$(date +%Y%m%d_%H%M%S)
        MAP_PATH="/ros2_ws/data/maps/${MAP_NAME}_${TIMESTAMP}"
        
        echo "Saving map to: ${MAP_PATH}"
        docker compose exec agv_sim bash -c "
            source /opt/ros/jazzy/setup.bash
            ros2 run nav2_map_server map_saver_cli -f ${MAP_PATH}
        "
        echo "Map saved!"
        echo "Files: ${MAP_PATH}.pgm and ${MAP_PATH}.yaml"
        ;;
    *)
        echo "Usage: $0 {build|up|down|enter|bash|logs|build-ws|test|restart|teleop|save-map}"
        echo ""
        echo "Commands:"
        echo "  build      - Build Docker image"
        echo "  up         - Start container in detached mode"
        echo "  down       - Stop and remove container"
        echo "  enter      - Enter the container (alias: bash)"
        echo "  logs       - Show container logs"
        echo "  build-ws   - Build ROS2 workspace in container"
        echo "  test       - Run tests in workspace"
        echo "  restart    - Restart container"
        echo "  teleop     - Launch keyboard teleop control"
        echo "  save-map   - Save current SLAM map (usage: save-map [name])"
        exit 1
        ;;
esac
