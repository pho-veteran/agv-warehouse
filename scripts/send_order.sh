#!/usr/bin/env bash
set -euo pipefail

# Helper script to send transport orders from outside the container
#
# Usage:
#   ./scripts/send_order.sh <pickup_station> <dropoff_station>
#   ./scripts/send_order.sh --list
#   ./scripts/send_order.sh --status
#   ./scripts/send_order.sh --reset

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AGV_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
DOCKER_DIR="$AGV_DIR/docker"

print_help() {
    echo "Send Transport Orders to AGV"
    echo ""
    echo "Usage: $0 <pickup_station> <dropoff_station> [--priority N]"
    echo "       $0 --list       List available stations"
    echo "       $0 --status     Monitor transport status"
    echo "       $0 --reset      Reset transport manager"
    echo "       $0 -h, --help   Show this help"
    echo ""
    echo "Available Stations:"
    echo "  - charging_station  (charger)"
    echo "  - dock_in_1         (dock)"
    echo "  - dock_in_2         (dock)"
    echo "  - dock_out          (dock)"
    echo ""
    echo "Examples:"
    echo "  $0 dock_in_1 dock_out"
    echo "  $0 dock_in_2 charging_station --priority 5"
    echo "  $0 --status"
}

cd "$DOCKER_DIR"

# Check if container is running
if ! docker compose ps agv_sim | grep -q "Up"; then
    echo "❌ Docker container not running. Start it first:"
    echo "   ./scripts/run_dev.sh up"
    exit 1
fi

# Handle special commands
case "${1:-}" in
    --list|-l)
        echo "Available Stations:"
        echo "  - charging_station  (13.157, -30.282) - charger"
        echo "  - dock_in_1         (-0.143, -20.679) - dock"
        echo "  - dock_in_2         (-3.756, -6.553)  - dock"
        echo "  - dock_out          (-0.181, -43.405) - dock"
        exit 0
        ;;
    --status|-s)
        echo "Monitoring transport status (Ctrl+C to stop)..."
        docker compose exec agv_sim bash -c "
            source /opt/ros/jazzy/setup.bash && \
            ros2 topic echo /agv/transport_status
        "
        exit 0
        ;;
    --reset|-r)
        echo "Resetting transport manager..."
        docker compose exec agv_sim bash -c "
            source /opt/ros/jazzy/setup.bash && \
            ros2 service call /agv/reset_transport std_srvs/srv/Trigger
        "
        exit 0
        ;;
    -h|--help)
        print_help
        exit 0
        ;;
    "")
        print_help
        exit 1
        ;;
esac

# Parse arguments for transport order
PICKUP="${1:-}"
DROPOFF="${2:-}"
PRIORITY="0"

shift 2 || true

while [[ $# -gt 0 ]]; do
    case "$1" in
        --priority|-p)
            PRIORITY="${2:-0}"; shift 2;;
        *)
            echo "Unknown arg: $1" >&2
            exit 2;;
    esac
done

if [[ -z "$PICKUP" ]] || [[ -z "$DROPOFF" ]]; then
    echo "Error: Both pickup and dropoff stations are required"
    print_help
    exit 1
fi

echo "Sending transport order:"
echo "  Pickup:   $PICKUP"
echo "  Dropoff:  $DROPOFF"
echo "  Priority: $PRIORITY"
echo ""

docker compose exec agv_sim bash -c "
    source /opt/ros/jazzy/setup.bash && \
    source /ros2_ws/install/setup.bash && \
    ros2 run agv_transport send_transport_order $PICKUP $DROPOFF --priority $PRIORITY
"

echo ""
echo "Monitor status with: $0 --status"
