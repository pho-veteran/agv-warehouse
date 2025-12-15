# Tích hợp turtlebot_warehouse Repository

## Tổng quan

Dự án đã được tích hợp với [turtlebot_warehouse repository](https://github.com/stevendes/turtlebot_warehouse) để sử dụng AWS RoboMaker Small Warehouse World.

## Những thay đổi đã thực hiện

### 1. Clone Repository
- Repository `turtlebot_warehouse` đã được clone vào `src/turtlebot_warehouse/`
- Cập nhật `scripts/setup.sh` để tự động clone repository này

### 2. Tạo ROS2 Package Wrapper
- Tạo package `turtlebot_warehouse_ros2` trong `src/turtlebot_warehouse_ros2/`
- Package này cung cấp:
  - Launch file ROS2 để sử dụng warehouse world
  - Tích hợp với turtlebot3_gazebo
  - Hỗ trợ các world files từ repository gốc

### 3. Launch File
- `warehouse_world.launch.py`: Launch file chính để chạy turtlebot3 trong warehouse world
- Hỗ trợ các tham số:
  - `world`: Chọn world file (no_roof_small_warehouse.world, small_warehouse.world, empty_test.world)
  - `gui`: Bật/tắt GUI (true/false)
  - `x_pose`, `y_pose`: Vị trí spawn robot

## Cấu trúc Repository

```
turtlebot_warehouse/
├── aws_robomaker_small_warehouse_world/
│   ├── worlds/          # World files (.world)
│   ├── maps/            # Pre-built maps (.pgm, .yaml)
│   ├── models/          # Gazebo models (shelves, walls, etc.)
│   └── launch/          # ROS1 launch files
├── t_gazebo/            # ROS1 gazebo launch files
├── turtlebot3/          # Turtlebot3 packages (ROS1)
└── turtlebot3_msgs/     # Turtlebot3 messages
```

## Sử dụng

### Build Package
```bash
cd /ros2_ws
colcon build --symlink-install --packages-select turtlebot_warehouse_ros2
source install/setup.bash
```

### Launch Warehouse World
```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot_warehouse_ros2 warehouse_world.launch.py
```

### Các tùy chọn
```bash
# Chọn world khác
ros2 launch turtlebot_warehouse_ros2 warehouse_world.launch.py world:=small_warehouse.world

# Headless mode
ros2 launch turtlebot_warehouse_ros2 warehouse_world.launch.py gui:=false

# Đặt vị trí spawn
ros2 launch turtlebot_warehouse_ros2 warehouse_world.launch.py x_pose:=2.0 y_pose:=1.0
```

## Lưu ý

1. **World Format**: Repository gốc sử dụng Gazebo Classic (ROS1), nhưng world files (.world) có thể tương thích với Gazebo Harmonic (ROS2 Jazzy). Nếu gặp lỗi, có thể cần chuyển đổi format.

2. **Models**: Các models trong `aws_robomaker_small_warehouse_world/models/` cần được Gazebo tìm thấy. Đảm bảo GAZEBO_MODEL_PATH được set đúng.

3. **Maps**: Maps có sẵn trong `maps/002/` và `maps/005/` có thể được sử dụng với Nav2 sau khi build workspace.

## Next Steps

1. Test warehouse world với Gazebo Harmonic
2. Tạo SLAM map mới từ warehouse world
3. Cấu hình Nav2 cho warehouse environment
4. Tích hợp vào navigation workflow

