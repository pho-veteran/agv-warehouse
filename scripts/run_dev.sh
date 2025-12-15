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
    *)
        echo "Usage: $0 {build|up|down|enter|bash|logs|build-ws|test|restart}"
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
        exit 1
        ;;
esac
