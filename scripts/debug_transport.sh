#!/usr/bin/env bash
set -euo pipefail

# Debug script for AGV Transport System
# Tests individual components to identify issues

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AGV_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
DOCKER_DIR="$AGV_DIR/docker"

cd "$DOCKER_DIR"

# Check if container is running
if ! docker compose ps agv_sim | grep -q "Up"; then
    echo "❌ Docker container not running. Start it first:"
    echo "   ./scripts/run_dev.sh up"
    exit 1
fi

print_header() {
    echo ""
    echo "════════════════════════════════════════════════════════════"
    echo "  $1"
    echo "════════════════════════════════════════════════════════════"
}

case "${1:-help}" in
    nodes)
        print_header "Running ROS2 Nodes"
        docker compose exec agv_sim bash -c "
            source /opt/ros/jazzy/setup.bash && \
            ros2 node list
        "
        ;;
    
    topics)
        print_header "ROS2 Topics"
        docker compose exec agv_sim bash -c "
            source /opt/ros/jazzy/setup.bash && \
            ros2 topic list | grep -E '(agv|nav|map)' || ros2 topic list
        "
        ;;
    
    nav2-status)
        print_header "Nav2 Action Server Status"
        docker compose exec agv_sim bash -c "
            source /opt/ros/jazzy/setup.bash && \
            echo 'Checking navigate_to_pose action...' && \
            ros2 action list | grep -i navigate || echo 'No navigate actions found'
            echo '' && \
            echo 'Action server info:' && \
            ros2 action info /navigate_to_pose 2>/dev/null || echo 'navigate_to_pose action not available'
        "
        ;;
    
    send-nav-goal)
        print_header "Sending Direct Nav2 Goal to dock_in_1"
        echo "Target: dock_in_1 (-0.143, -20.679)"
        docker compose exec agv_sim bash -c "
            source /opt/ros/jazzy/setup.bash && \
            ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose '{
              pose: {
                header: {frame_id: \"map\"},
                pose: {
                  position: {x: -0.143, y: -20.679, z: 0.0},
                  orientation: {w: 1.0}
                }
              }
            }' --feedback
        "
        ;;
    
    check-transport-node)
        print_header "Transport Task Manager Node Status"
        docker compose exec agv_sim bash -c "
            source /opt/ros/jazzy/setup.bash && \
            echo '=== Node Check ===' && \
            ros2 node list | grep -i transport || echo 'Transport node NOT found!' && \
            echo '' && \
            echo '=== Topic Info ===' && \
            echo '/agv/transport_orders:' && \
            ros2 topic info /agv/transport_orders 2>/dev/null || echo '  Topic not found' && \
            echo '' && \
            echo '/agv/transport_status:' && \
            ros2 topic info /agv/transport_status 2>/dev/null || echo '  Topic not found' && \
            echo '' && \
            echo '=== Services ===' && \
            ros2 service list | grep agv || echo 'No AGV services found'
        "
        ;;
    
    test-order-topic)
        print_header "Testing Order Topic Communication"
        echo "Publishing test order directly..."
        docker compose exec agv_sim bash -c "
            source /opt/ros/jazzy/setup.bash && \
            ros2 topic pub --once /agv/transport_orders std_msgs/msg/String '{data: \"{\\\"order_id\\\": \\\"test-123\\\", \\\"pickup_station\\\": \\\"dock_in_1\\\", \\\"dropoff_station\\\": \\\"dock_out\\\", \\\"priority\\\": 0, \\\"timestamp\\\": 0.0}\"}' --qos-reliability reliable --qos-durability transient_local
        "
        echo ""
        echo "Check status now with: ./scripts/send_order.sh --status"
        ;;
    
    transport-logs)
        print_header "Transport Task Manager Logs (last 50 lines)"
        docker compose exec agv_sim bash -c "
            source /opt/ros/jazzy/setup.bash && \
            ros2 topic echo /rosout --once --no-arr | head -100
        " 2>/dev/null || echo "Could not get logs"
        ;;
    
    station-test)
        print_header "Testing Station Manager"
        docker compose exec agv_sim bash -c "
            source /opt/ros/jazzy/setup.bash && \
            source /ros2_ws/install/setup.bash && \
            python3 -c \"
from agv_transport.station_manager import StationManager
sm = StationManager('/ros2_ws/src/agv_transport/config/warehouse_stations.yaml')
print('Loaded stations:', list(sm.stations.keys()))
for name, config in sm.stations.items():
    print(f'  {name}: ({config.x}, {config.y}, yaw={config.yaw})')
pose = sm.get_station_pose('dock_in_1')
if pose:
    print(f'\\ndock_in_1 pose: x={pose.pose.position.x}, y={pose.pose.position.y}')
else:
    print('\\nERROR: Could not get dock_in_1 pose')
\"
        "
        ;;
    
    restart-transport)
        print_header "Restarting Transport Task Manager"
        echo "Killing existing transport node..."
        docker compose exec agv_sim bash -c "
            pkill -f transport_task_manager || true
        "
        sleep 2
        echo "Starting transport task manager..."
        docker compose exec -d agv_sim bash -lc "
            source /opt/ros/jazzy/setup.bash && \
            source /ros2_ws/install/setup.bash && \
            ros2 launch agv_transport transport_manager.launch.py \
              use_sim_time:=true \
              config_path:=/ros2_ws/src/agv_transport/config/warehouse_stations.yaml
        "
        sleep 3
        echo "Checking if node started..."
        docker compose exec agv_sim bash -c "
            source /opt/ros/jazzy/setup.bash && \
            ros2 node list | grep transport || echo 'Transport node NOT started!'
        "
        ;;
    
    full-test)
        print_header "Full System Test"
        
        echo "1. Checking nodes..."
        docker compose exec agv_sim bash -c "source /opt/ros/jazzy/setup.bash && ros2 node list" | head -20
        
        echo ""
        echo "2. Checking Nav2 action server..."
        docker compose exec agv_sim bash -c "source /opt/ros/jazzy/setup.bash && ros2 action list" | grep navigate || echo "   Nav2 action NOT available"
        
        echo ""
        echo "3. Checking transport topics..."
        docker compose exec agv_sim bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic list" | grep agv || echo "   No AGV topics found"
        
        echo ""
        echo "4. Checking topic subscribers..."
        docker compose exec agv_sim bash -c "
            source /opt/ros/jazzy/setup.bash && \
            echo 'Subscribers to /agv/transport_orders:' && \
            ros2 topic info /agv/transport_orders -v 2>/dev/null | grep -A5 'Subscription count' || echo '  Could not get info'
        "
        ;;
    
    help|*)
        echo "Debug Transport System"
        echo ""
        echo "Usage: $0 <command>"
        echo ""
        echo "Commands:"
        echo "  nodes              - List all running ROS2 nodes"
        echo "  topics             - List relevant ROS2 topics"
        echo "  nav2-status        - Check Nav2 action server status"
        echo "  send-nav-goal      - Send a direct Nav2 goal (bypasses transport manager)"
        echo "  check-transport-node - Check transport task manager status"
        echo "  test-order-topic   - Publish test order directly to topic"
        echo "  station-test       - Test station manager loading"
        echo "  restart-transport  - Restart the transport task manager"
        echo "  full-test          - Run full system diagnostic"
        echo "  help               - Show this help"
        ;;
esac
