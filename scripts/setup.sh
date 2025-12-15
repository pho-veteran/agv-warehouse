#!/bin/bash
# Setup script for AGV Smart Warehouse Project

set -e

echo "Setting up AGV Smart Warehouse Project..."

# Create directory structure
mkdir -p {src,config,data/{logs,bags,maps},scripts}

# Clone required repositories
cd src
if [ ! -d "turtlebot3" ]; then
    git clone -b jazzy-devel https://github.com/ROBOTIS-GIT/turtlebot3.git || \
    git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3.git
fi
if [ ! -d "turtlebot3_simulations" ]; then
    git clone -b jazzy-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git || \
    git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
fi
if [ ! -d "slam_toolbox" ]; then
    git clone -b jazzy https://github.com/SteveMacenski/slam_toolbox.git
fi
if [ ! -d "navigation2" ]; then
    git clone -b jazzy https://github.com/ros-planning/navigation2.git
fi

# NOTE:
# We no longer need to keep the full turtlebot_warehouse repo in src/.
# Only the warehouse assets (worlds/models/maps) are needed, and they can be vendored into:
#   src/turtlebot3_simulations/turtlebot3_warehouse_sim/warehouse_assets
# If you want to (re)vendor assets, run from AGV_2/:
#   ./scripts/vendor_warehouse_assets.sh

# Note: Custom package agv_warehouse will be created inside container
# after workspace is built
cd ..

# Set environment variables in .env file if it doesn't exist
if [ ! -f ".env" ]; then
    cat > .env << EOF
ROS_DOMAIN_ID=0
TURTLEBOT3_MODEL=waffle_pi
DISPLAY=\${DISPLAY}
EOF
fi

echo "Setup completed!"
echo ""
echo "Next steps:"
echo "1. Review and update .env file if needed"
echo "2. Build Docker containers: cd docker && docker compose build"
echo "3. Start containers: docker compose up -d"

