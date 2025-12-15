#!/usr/bin/env bash
set -euo pipefail

# Copy ONLY the needed warehouse assets (worlds/models/maps) from:
#   AGV_2/src/turtlebot_warehouse/aws_robomaker_small_warehouse_world/
# into the turtlebot3_simulations package:
#   AGV_2/src/turtlebot3_simulations/turtlebot3_warehouse_sim/warehouse_assets/
#
# After running this successfully, you can delete:
#   AGV_2/src/turtlebot_warehouse/
# and the launch will still work (it will prefer vendored assets).

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AGV_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

SRC_ROOT="$AGV_DIR/src/turtlebot_warehouse/aws_robomaker_small_warehouse_world"
DST_ROOT="$AGV_DIR/src/turtlebot3_simulations/turtlebot3_warehouse_sim/warehouse_assets"

if [[ ! -d "$SRC_ROOT" ]]; then
  echo "Source assets not found: $SRC_ROOT" >&2
  echo "Did you clone turtlebot_warehouse or run ./scripts/setup.sh ?" >&2
  exit 1
fi

mkdir -p "$DST_ROOT"

echo "[vendor] Copying worlds/ models/ maps/ ..."
rm -rf "$DST_ROOT/worlds" "$DST_ROOT/models" "$DST_ROOT/maps"
cp -a "$SRC_ROOT/worlds" "$DST_ROOT/"
cp -a "$SRC_ROOT/models" "$DST_ROOT/"
cp -a "$SRC_ROOT/maps" "$DST_ROOT/"

echo "[vendor] Done."
echo "[vendor] Assets now at: $DST_ROOT"
echo "[vendor] Optional next step: rebuild inside container:"
echo "  cd /ros2_ws && colcon build --symlink-install --packages-select turtlebot3_warehouse_sim"


