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
- Clone các repositories cần thiết (turtlebot3, navigation2, slam_toolbox)
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

### 6. Launch Gazebo với empty world hoặc house world

Trong container:
```bash
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

Hoặc với house world:
```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

### 7. Test với Empty World và Spawn Robot


Terminal 1 - Gazebo:
```bash
./scripts/run_dev.sh enter
# Trong container:
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

Terminal 2 - Teleop:
```bash
./scripts/run_dev.sh enter
# Trong container:
source /opt/ros/jazzy/setup.bash
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

Đảm bảo DISPLAY được set:
```bash
export DISPLAY=:0
xhost +local:docker
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

## Next Steps

Sau khi hoàn thành Phase 1, tiếp tục với:
- Phase 2: Điều hướng cơ bản 1 AGV
- Phase 4: Logistics Operations
- Phase 5: Hệ thống hoàn chỉnh

Xem [AGV_Design.md](AGV_Design.md) để biết chi tiết.
