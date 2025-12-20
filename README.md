# AGV Smart Warehouse System - Phase 1

Hệ thống AGV cho kho thông minh sử dụng Docker, ROS2 Jazzy và Gazebo Harmonic.

**Note:** Setup này sử dụng single container (giống AGV_1 và ROBOTIS), đơn giản và dễ sử dụng.

## Yêu cầu hệ thống

- Ubuntu 22.04+ hoặc Windows WSL2
- Docker Engine ≥ 20.10
- Docker Compose ≥ 2.20
- NVIDIA Container Toolkit (optional, cho GPU acceleration)
- X11 server (cho GUI)

## Cấu trúc dự án

```
AGV_2/
├── docker/              # Docker configuration files
│   ├── Dockerfile       # Single container Dockerfile
│   ├── entrypoint.sh    # Entrypoint script
│   ├── requirements.txt # Python dependencies
│   └── docker-compose.yml
├── src/                 # ROS2 workspace source
├── config/              # Configuration files
├── data/                # Persistent data (logs, bags, maps)
├── scripts/             # Utility scripts
└── .env                # Environment variables
```

## Quick Start

### 1. Setup ban đầu

```bash
cd AGV_2
./scripts/setup.sh
```

Script này sẽ:
- Tạo cấu trúc thư mục
- Clone các repositories cần thiết (turtlebot3, navigation2, slam_toolbox, turtlebot_warehouse)
- Tạo file .env

### 2. Build Docker container

```bash
cd docker
docker compose build
```

Hoặc sử dụng helper script:
```bash
./scripts/run_dev.sh build
```

### 3. Start container

```bash
cd docker
docker compose up -d
```

Hoặc:
```bash
./scripts/run_dev.sh up
```

### 4. Kiểm tra container

```bash
docker compose ps
```

Bạn sẽ thấy container `agv_simulation` đang chạy.

### 5. Enter container

```bash
./scripts/run_dev.sh enter
```

Hoặc:
```bash
./scripts/run_dev.sh bash
```

### 6. Build ROS2 workspace (bao gồm warehouse worlds)

Trong container:
```bash
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 7. Launch Gazebo với các world khác nhau

**Empty World:**
```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

**House World:**
```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

**Warehouse World (dùng trực tiếp world files từ `src/turtlebot_warehouse/.../worlds` và chạy trong turtlebot3_simulations):**
```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_warehouse_sim turtlebot3_warehouse_world.launch.py
```

Hoặc với các tùy chọn:
```bash
# Warehouse không có mái
ros2 launch turtlebot3_warehouse_sim turtlebot3_warehouse_world.launch.py world:=no_roof_small_warehouse.world

# Warehouse có mái
ros2 launch turtlebot3_warehouse_sim turtlebot3_warehouse_world.launch.py world:=small_warehouse.world

# Không có GUI (headless mode)
ros2 launch turtlebot3_warehouse_sim turtlebot3_warehouse_world.launch.py gui:=false

# Đặt vị trí spawn robot
ros2 launch turtlebot3_warehouse_sim turtlebot3_warehouse_world.launch.py x_pose:=2.0 y_pose:=1.0
```

Hoặc dùng helper script (chạy từ host, script sẽ `docker compose exec` vào container):
```bash
# Mặc định: gui=true, no_roof_small_warehouse.world
./scripts/launch_warehouse_world.sh

# Chọn world khác
./scripts/launch_warehouse_world.sh --world small_warehouse.world

# Headless mode
./scripts/launch_warehouse_world.sh --headless

# Đặt vị trí spawn robot
./scripts/launch_warehouse_world.sh --x 2.0 --y 1.0
```

### 8. Test với Warehouse World và Spawn Robot


Terminal 1 - Gazebo với Warehouse World:
```bash
./scripts/run_dev.sh enter
# Trong container:
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_warehouse_sim turtlebot3_warehouse_world.launch.py
```

Terminal 2 - Teleop:
```bash
./scripts/run_dev.sh enter
# Trong container:
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 run turtlebot3_teleop teleop_keyboard
```

Sử dụng các phím để điều khiển robot:
- `w`: Tiến
- `s`: Lùi
- `a`: Quay trái
- `d`: Quay phải
- `x`: Dừng

## Development Workflow

### Build ROS2 workspace

```bash
./scripts/run_dev.sh build-ws
```

### Xem logs

```bash
./scripts/run_dev.sh logs
```

### Restart container

```bash
./scripts/run_dev.sh restart
```

### Stop container

```bash
./scripts/run_dev.sh down
```

## Troubleshooting

### X11 forwarding không hoạt động

Đảm bảo DISPLAY được set đúng. Kiểm tra display hiện tại:
```bash
# Kiểm tra display đang dùng
echo $DISPLAY
ls /tmp/.X11-unix/

# Thường là :0 hoặc :1, set đúng display
export DISPLAY=:1  # hoặc :0 tùy vào hệ thống
xhost +local:docker
```

Nếu vẫn không hoạt động, có thể chạy headless mode (không cần GUI):
```bash
# Trong container
ros2 launch turtlebot_warehouse_ros2 warehouse_world.launch.py gui:=false
```

### Gazebo không hiển thị

- Kiểm tra X11 forwarding
- Thử dùng CPU rendering: thêm `LIBGL_ALWAYS_SOFTWARE=1` vào .env

### GPU không hoạt động

Đảm bảo NVIDIA Container Toolkit đã được cài đặt:
```bash
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker
```

## Warehouse World từ turtlebot_warehouse

Dự án này tích hợp [turtlebot_warehouse repository](https://github.com/stevendes/turtlebot_warehouse) để sử dụng AWS RoboMaker Small Warehouse World. Repository này cung cấp:

- **Warehouse World**: Môi trường kho hàng với kệ, tường, và các vật cản
- **Turtlebot Models**: Các model turtlebot với camera
- **Maps**: Bản đồ đã được tạo sẵn cho warehouse

### Các world có sẵn:
- `no_roof_small_warehouse.world`: Warehouse không có mái (mặc định)
- `small_warehouse.world`: Warehouse có mái
- `empty_test.world`: World trống để test

### Maps có sẵn:
Maps được lưu trong `src/turtlebot_warehouse/aws_robomaker_small_warehouse_world/maps/`:
- `002/map.yaml` và `map.pgm`
- `005/map.yaml` và `map.pgm` (có cả rotated version)

## Next Steps

Sau khi hoàn thành Phase 1, tiếp tục với:
- Phase 2: Điều hướng cơ bản 1 AGV (sử dụng warehouse world)
- Phase 4: Logistics Operations
- Phase 5: Hệ thống hoàn chỉnh

Xem [AGV_Design.md](AGV_Design.md) để biết chi tiết.

# Frequently Used CMD

./scripts/run_dev.sh clean-build agv_auto_explore
./scripts/run_dev.sh clean-build agv_transport
./scripts/run_dev.sh clean-build agv_transport_web

./scripts/launch_large_warehouse.sh --tugbot --location entrance --slam --auto-explore

./scripts/launch_transport_web_demo.sh

<!-- Transport với CLI -->
./scripts/launch_transport_demo.sh
./scripts/send_order.sh dock_in_1 dock_out
./scripts/send_order.sh dock_out charging_station
./scripts/send_order.sh dock_in_2 charging_station
./scripts/send_order.sh --status

./scripts/debug_transport.sh send-nav-goal