# Chi tiết Thiết kế Map Kho - Sử dụng Tugbot Warehouse

## 1. Tổng quan

### 1.1. Thay đổi Kế hoạch

**Kế hoạch ban đầu:** Tự tạo world 30x40m từ đầu  
**Kế hoạch mới:** Sử dụng `tugbot_warehouse` từ Gazebo Fuel, chỉnh sửa cho phù hợp với tech stack hiện tại

### 1.2. Nguồn gốc Map

| Thuộc tính | Giá trị |
|------------|---------|
| **Tên** | Tugbot Warehouse |
| **Nguồn** | [Gazebo Fuel - MovAi](https://app.gazebosim.org/MovAi/worlds/tugbot_warehouse) |
| **Mô tả gốc** | A simple warehouse to simulate a pick and drop Mov.Ai controller for Tugbot robot |
| **SDF Version gốc** | 1.7 |
| **Target SDF Version** | 1.8 (Gazebo Harmonic) |

### 1.3. Tech Stack Target

| Component | Version | Notes |
|-----------|---------|-------|
| **ROS2** | Jazzy | Latest LTS |
| **Gazebo** | Harmonic | gz-sim 8.x |
| **Robot** | TurtleBot3 Waffle Pi | Thay thế Tugbot |
| **Actor** | Walking Actor | Từ Gazebo Fuel |

---

## 2. Phân tích World Gốc (`tugbot_warehouse.sdf`)

### 2.1. Cấu trúc Models trong World

```
tugbot_warehouse.sdf
├── ground_plane (built-in)
├── Warehouse (MovAi) ─────────────── Base building structure
├── Tugbot (MovAi) ────────────────── REMOVE (thay bằng TurtleBot3)
├── charging_station (MovAi) ──────── KEEP ✓
├── cart1 (cart_model_2) ──────────── KEEP ✓
├── Shelves
│   ├── shelf_big (x5) ────────────── KEEP ✓
│   └── shelf (x11) ───────────────── KEEP ✓
├── pallet_box_mobile (x5) ────────── KEEP ✓
└── pallet_box (static, x2) ───────── KEEP ✓
```

### 2.2. Vị trí các Models (Tọa độ XYZ)

```yaml
# Warehouse base
warehouse:
  pose: [0, 0, -0.09]
  size_estimate: ~35m x 50m  # Dựa vào coordinates

# Charging Station
charging_station:
  pose: [14.7, -10.6, -0.04]
  description: "Góc phải dưới warehouse"

# Tugbot (SẼ XÓA)
tugbot:
  pose: [13.9, -10.6, 0.1]
  action: REMOVE

# Cart
cart1:
  pose: [-5.73, 15, 0.25]
  
# Shelves - Big (5 units)
shelf_big_0: [-9.34, -13.56, 0]
shelf_big_1: [13.98, 15.32, 0]
shelf_big_2: [6.20, -12.96, 0]
shelf_big_3: [0.59, -12.96, 0]
shelf_big_4: [-5.36, -12.96, 0]

# Shelves - Regular (11 units)
shelf:    [-4.42, -0.69, 0]
shelf_0:  [-4.42, 2.31, 0]
shelf_1:  [-4.42, 5.31, 0]
shelf_2:  [-4.42, 8.34, 0]
shelf_3:  [5.60, 8.34, 0]
shelf_4:  [5.60, 5.31, 0]
shelf_5:  [5.60, -0.69, 0]
shelf_6:  [5.60, 2.31, 0]
shelf_7:  [13.38, -21.24, 0]
shelf_8:  [13.38, -19.00, 0]
shelf_9:  [13.38, -16.45, 0]
shelf_10: [13.38, -14.10, 0]

# Pallet Boxes
pallet_box_mobile:   [4.42, 14.70, 0.01]
pallet_box_mobile_0: [4.45, 13.62, 0.01]
pallet_box_mobile_1: [4.45, 12.23, 0.01]
pallet_box (static): [-6.12, 13.71, 0.01]
pallet_box_0 (static): [14.02, -24.34, 0.01]
```

### 2.3. Layout Visualization

```
        Y ↑
          │
    20m ──┼───────────────────────────────────────────────
          │   [cart1]     [pallet_boxes]    [shelf_big_1]
          │   (-5.7,15)   (4.4,12-15)       (14,15.3)
    15m ──┼───────────────────────────────────────────────
          │
          │
    10m ──┼───────────────────────────────────────────────
          │   [shelf_2]                [shelf_3]
          │   (-4.4,8.3)               (5.6,8.3)
          │   [shelf_1]                [shelf_4]
     5m ──┼   (-4.4,5.3)               (5.6,5.3)
          │   [shelf_0]                [shelf_6]
          │   (-4.4,2.3)               (5.6,2.3)
          │   [shelf]                  [shelf_5]
     0m ──┼───(-4.4,-0.7)──────────────(5.6,-0.7)─────────
          │
          │
   -10m ──┼───────────────────────────[charging]──────────
          │                            (14.7,-10.6)
          │   [shelf_big_0]           [shelf_10] (13.4,-14)
   -15m ──┼   (-9.3,-13.6)            [shelf_9]  (13.4,-16)
          │   [shelf_big_4,3,2]       [shelf_8]  (13.4,-19)
          │   (-5 to 6, -13)          [shelf_7]  (13.4,-21)
   -20m ──┼───────────────────────────────────────────────
          │                            [pallet_box_0]
   -25m ──┼────────────────────────────(14,-24.3)─────────
          │
          └──────┼──────┼──────┼──────┼──────┼──────┼─────→ X
               -10     -5      0      5     10     15
```

---

## 3. Thay đổi Cần Thực Hiện

### 3.1. Cập nhật Gazebo Harmonic Compatibility

#### 3.1.1. SDF Version

```xml
<!-- TỪ -->
<sdf version='1.7'>

<!-- THÀNH -->
<sdf version='1.8'>
```

#### 3.1.2. Plugin Names (Ignition → Gazebo)

```xml
<!-- TỪ (Ignition Gazebo) -->
<plugin name='ignition::gazebo::systems::Physics' filename='libignition-gazebo-physics-system.so'/>
<plugin name='ignition::gazebo::systems::UserCommands' filename='libignition-gazebo-user-commands-system.so'/>
<plugin name='ignition::gazebo::systems::SceneBroadcaster' filename='libignition-gazebo-scene-broadcaster-system.so'/>
<plugin name='ignition::gazebo::systems::Imu' filename='ignition-gazebo-imu-system'/>
<plugin name='ignition::gazebo::systems::Sensors' filename='ignition-gazebo-sensors-system'>

<!-- THÀNH (Gazebo Harmonic) -->
<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
<plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
<plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
<plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
```

#### 3.1.3. Fuel URI Updates

```xml
<!-- TỪ (deprecated) -->
<uri>https://fuel.ignitionrobotics.org/1.0/MovAi/models/Warehouse</uri>

<!-- THÀNH (current) -->
<uri>https://fuel.gazebosim.org/1.0/MovAi/models/Warehouse</uri>
```

### 3.2. Loại bỏ Tugbot

```xml
<!-- XÓA HOÀN TOÀN BLOCK NÀY -->
<include>
  <uri>https://fuel.ignitionrobotics.org/1.0/MovAi/models/Tugbot</uri>
  <name>tugbot</name>
  <pose>13.9 -10.6 0.1 0 0 0</pose>
</include>
```

**Lý do:** TurtleBot3 sẽ được spawn riêng qua launch file, không cần hardcode trong world.

### 3.3. Thêm Walking Actors

#### 3.3.1. Actor Configuration

Sử dụng `Walking actor` model đã tải về, customize trajectory cho phù hợp với warehouse layout.

```xml
<!-- Actor 1: Đi dọc hành lang chính (Y = 0 to 10) -->
<include>
  <uri>model://Walking_actor</uri>
  <name>worker_1</name>
  <pose>0 0 0 0 0 0</pose>
</include>

<!-- HOẶC inline actor với custom trajectory -->
<actor name="warehouse_worker_1">
  <skin>
    <filename>model://Walking_actor/meshes/walk.dae</filename>
    <scale>1.0</scale>
  </skin>
  <animation name="walking">
    <filename>model://Walking_actor/meshes/walk.dae</filename>
    <scale>1.0</scale>
    <interpolate_x>true</interpolate_x>
  </animation>
  <script>
    <loop>true</loop>
    <delay_start>0.0</delay_start>
    <auto_start>true</auto_start>
    <trajectory id="0" type="walking">
      <!-- Route: Đi từ khu vực shelf trái sang phải -->
      <waypoint>
        <time>0</time>
        <pose>-4.0 0 0 0 0 0</pose>
      </waypoint>
      <waypoint>
        <time>8</time>
        <pose>5.5 0 0 0 0 0</pose>
      </waypoint>
      <waypoint>
        <time>16</time>
        <pose>5.5 8 0 0 0 1.57</pose>
      </waypoint>
      <waypoint>
        <time>24</time>
        <pose>-4.0 8 0 0 0 3.14</pose>
      </waypoint>
      <waypoint>
        <time>32</time>
        <pose>-4.0 0 0 0 0 -1.57</pose>
      </waypoint>
    </trajectory>
  </script>
</actor>
```

#### 3.3.2. Proposed Actor Trajectories

```
        Y ↑
          │
    15m ──┼─────────────────────────────────────────────
          │                                              
          │                                              
    10m ──┼─────────────────────────────────────────────
          │   ←←←←←←←←←← [Worker 1] ←←←←←←←←←           
          │   ↓         (rectangular path)         ↑    
     5m ──┼   ↓                                    ↑    
          │   ↓                                    ↑    
          │   ↓                                    ↑    
     0m ──┼   →→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→↑    
          │                                              
          │                                              
   -10m ──┼─────────────[Worker 2]──────────────────────
          │             (patrol near charging station)  
          │             ○ → → → ○                       
          │             ↑       ↓                       
          │             ○ ← ← ← ○                       
   -15m ──┼─────────────────────────────────────────────
          │
          └──────┼──────┼──────┼──────┼──────┼──────┼────→ X
               -10     -5      0      5     10     15
```

---

## 4. File Structure Mới

### 4.1. Cấu trúc Package

```
AGV_2/src/turtlebot3_simulations/
├── turtlebot3_warehouse_sim/           # Package hiện có
│   ├── warehouse_assets/
│   │   ├── models/
│   │   │   ├── aws_robomaker_warehouse_*/  # Giữ nguyên (backup)
│   │   │   └── Walking_actor/              # NEW - Copy từ implements_detail
│   │   │       ├── model.config
│   │   │       ├── model.sdf
│   │   │       └── meshes/
│   │   │           └── walk.dae
│   │   └── worlds/
│   │       ├── no_roof_small_warehouse.world  # Existing
│   │       ├── small_warehouse.world          # Existing
│   │       └── tugbot_warehouse.world         # NEW - Modified version
│   └── launch/
│       ├── turtlebot3_warehouse_world.launch.py      # Existing
│       └── turtlebot3_tugbot_warehouse.launch.py     # NEW
```

### 4.2. Files Cần Tạo/Modify

| File | Action | Description |
|------|--------|-------------|
| `tugbot_warehouse.world` | CREATE | Modified SDF for Gazebo Harmonic |
| `Walking_actor/` | COPY | Copy từ `implements_detail/Walking actor/` |
| `turtlebot3_tugbot_warehouse.launch.py` | CREATE | Launch file cho world mới |

---

## 5. Chi tiết Chỉnh sửa World File

### 5.1. Header và Plugins

```xml
<?xml version='1.0' encoding='UTF-8'?>
<sdf version='1.8'>
  <world name='tugbot_warehouse'>
    <gravity>0 0 -9.8</gravity>
    
    <physics type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    
    <!-- Gazebo Harmonic Plugins -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    
    <scene>
      <ambient>1 1 1 1</ambient>
      <background>0.3 0.7 0.9 1</background>
      <shadows>0</shadows>
      <grid>false</grid>
    </scene>
    
    <!-- Content follows... -->
```

### 5.2. Models với Updated URIs

```xml
    <!-- Light -->
    <light type="directional" name="sun">
      <cast_shadows>0</cast_shadows>
      <pose>-5 -3 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>1 1 1 1</specular>
      <direction>0 0 -1</direction>
    </light>

    <!-- Ground Plane -->
    <model name='ground_plane'>
      <static>true</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0.0 0.0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name='visual'>
          <geometry>
            <plane>
              <normal>0.0 0.0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Warehouse Base Structure -->
    <include>
      <uri>https://fuel.gazebosim.org/1.0/MovAi/models/Warehouse</uri>
      <name>warehouse</name>
      <pose>0 0 -0.09 0 0 0</pose>
    </include>

    <!-- Charging Station (KEEP) -->
    <include>
      <uri>https://fuel.gazebosim.org/1.0/MovAi/models/Tugbot-charging-station</uri>
      <name>charging_station</name>
      <pose>14.7 -10.6 -0.04 0 0 0</pose>
    </include>

    <!-- NOTE: Tugbot REMOVED - TurtleBot3 will be spawned via launch file -->

    <!-- Shelves and other objects... (with updated URIs) -->
```

### 5.3. Walking Actors

```xml
    <!-- Walking Actor 1: Main aisle patrol -->
    <actor name="warehouse_worker_1">
      <skin>
        <filename>model://Walking_actor/meshes/walk.dae</filename>
        <scale>1.0</scale>
      </skin>
      <animation name="walking">
        <filename>model://Walking_actor/meshes/walk.dae</filename>
        <scale>1.0</scale>
        <interpolate_x>true</interpolate_x>
      </animation>
      <script>
        <loop>true</loop>
        <delay_start>0.0</delay_start>
        <auto_start>true</auto_start>
        <trajectory id="0" type="walking">
          <!-- Rectangular path around central shelves -->
          <waypoint>
            <time>0</time>
            <pose>-4.0 -1.0 0 0 0 0</pose>
          </waypoint>
          <waypoint>
            <time>10</time>
            <pose>5.5 -1.0 0 0 0 0</pose>
          </waypoint>
          <waypoint>
            <time>12</time>
            <pose>5.5 -1.0 0 0 0 1.57</pose>
          </waypoint>
          <waypoint>
            <time>22</time>
            <pose>5.5 8.5 0 0 0 1.57</pose>
          </waypoint>
          <waypoint>
            <time>24</time>
            <pose>5.5 8.5 0 0 0 3.14</pose>
          </waypoint>
          <waypoint>
            <time>34</time>
            <pose>-4.0 8.5 0 0 0 3.14</pose>
          </waypoint>
          <waypoint>
            <time>36</time>
            <pose>-4.0 8.5 0 0 0 -1.57</pose>
          </waypoint>
          <waypoint>
            <time>46</time>
            <pose>-4.0 -1.0 0 0 0 -1.57</pose>
          </waypoint>
        </trajectory>
      </script>
    </actor>

    <!-- Walking Actor 2: Near charging station area -->
    <actor name="warehouse_worker_2">
      <skin>
        <filename>model://Walking_actor/meshes/walk.dae</filename>
        <scale>1.0</scale>
      </skin>
      <animation name="walking">
        <filename>model://Walking_actor/meshes/walk.dae</filename>
        <scale>1.0</scale>
        <interpolate_x>true</interpolate_x>
      </animation>
      <script>
        <loop>true</loop>
        <delay_start>5.0</delay_start>
        <auto_start>true</auto_start>
        <trajectory id="0" type="walking">
          <!-- Patrol near charging station and lower shelves -->
          <waypoint>
            <time>0</time>
            <pose>10.0 -10.0 0 0 0 0</pose>
          </waypoint>
          <waypoint>
            <time>8</time>
            <pose>10.0 -18.0 0 0 0 -1.57</pose>
          </waypoint>
          <waypoint>
            <time>16</time>
            <pose>13.0 -18.0 0 0 0 0</pose>
          </waypoint>
          <waypoint>
            <time>24</time>
            <pose>13.0 -10.0 0 0 0 1.57</pose>
          </waypoint>
          <waypoint>
            <time>32</time>
            <pose>10.0 -10.0 0 0 0 3.14</pose>
          </waypoint>
        </trajectory>
      </script>
    </actor>
```

---

## 6. TurtleBot3 Spawn Position

### 6.1. Recommended Spawn Positions

| Position Name | Coordinates (x, y, z) | Description |
|---------------|----------------------|-------------|
| **Default** | `(0, 0, 0.1)` | Center of warehouse |
| **Near Charging** | `(12.0, -10.6, 0.1)` | Gần trạm sạc |
| **Inbound Area** | `(0, 12, 0.1)` | Khu vực nhận hàng |
| **Central Aisle** | `(0.5, 4, 0.1)` | Giữa hành lang chính |

### 6.2. Launch File Arguments

```python
# Trong launch file
x_pose = LaunchConfiguration('x_pose', default='0.0')
y_pose = LaunchConfiguration('y_pose', default='0.0')
z_pose = LaunchConfiguration('z_pose', default='0.1')
```

---

## 7. Points of Interest (POI) cho Navigation

### 7.1. Key Waypoints

```yaml
waypoints:
  # Charging Station
  charging_station:
    position: [14.7, -10.6, 0]
    type: charging
    description: "Trạm sạc Tugbot (compatible với TurtleBot3)"

  # Shelf Areas - Left Side
  shelf_area_left_1:
    position: [-4.4, 0, 0]
    type: pickup
    shelf_ids: [shelf, shelf_0, shelf_1, shelf_2]
    
  # Shelf Areas - Right Side
  shelf_area_right_1:
    position: [5.6, 0, 0]
    type: pickup
    shelf_ids: [shelf_5, shelf_6, shelf_3, shelf_4]

  # Shelf Big Area - Bottom
  shelf_big_area_bottom:
    position: [0, -13, 0]
    type: storage
    shelf_ids: [shelf_big_0, shelf_big_2, shelf_big_3, shelf_big_4]

  # Pallet Area - Top
  pallet_pickup_area:
    position: [4.4, 13, 0]
    type: pallet_pickup
    
  # Cart Area
  cart_area:
    position: [-5.7, 15, 0]
    type: cart_pickup

  # Intersection Points
  intersection_center:
    position: [0.5, 4, 0]
    type: intersection
    
  intersection_bottom:
    position: [0.5, -8, 0]
    type: intersection
```

### 7.2. Navigation Zones

```yaml
zones:
  zone_storage_left:
    bounds: [[-6, -2], [-2, 10]]  # [x_min, x_max], [y_min, y_max]
    type: storage
    
  zone_storage_right:
    bounds: [[4, 7], [-2, 10]]
    type: storage
    
  zone_charging:
    bounds: [[12, 16], [-12, -8]]
    type: charging
    
  zone_loading:
    bounds: [[-8, 6], [12, 17]]
    type: loading
    
  zone_storage_bottom:
    bounds: [[-10, 8], [-15, -11]]
    type: bulk_storage
```

---

## 8. Implementation Checklist

### Phase 1: Chuẩn bị Files (1 ngày)

- [ ] **Task 1.1:** Copy `Walking actor` folder sang `warehouse_assets/models/Walking_actor/`
  ```bash
  cp -r "AGV_2/implements_detail/Walking actor" \
        "AGV_2/src/turtlebot3_simulations/turtlebot3_warehouse_sim/warehouse_assets/models/Walking_actor"
  ```

- [ ] **Task 1.2:** Tạo `tugbot_warehouse.world` với các modifications
  - Update SDF version to 1.8
  - Update plugin names
  - Update Fuel URIs
  - Remove Tugbot model
  - Add Walking actors

### Phase 2: Launch File (0.5 ngày)

- [ ] **Task 2.1:** Tạo `turtlebot3_tugbot_warehouse.launch.py`
  - Copy từ `turtlebot3_warehouse_world.launch.py`
  - Update world file path
  - Update model paths
  - Add TurtleBot3 spawn

### Phase 3: Testing (1 ngày)

- [ ] **Task 3.1:** Test world loads trong Gazebo Harmonic
  ```bash
  gz sim tugbot_warehouse.world
  ```

- [ ] **Task 3.2:** Test với TurtleBot3 spawn
  ```bash
  ros2 launch turtlebot3_warehouse_sim turtlebot3_tugbot_warehouse.launch.py
  ```

- [ ] **Task 3.3:** Test Walking actors hoạt động
  - Verify trajectory paths
  - Check collision detection

- [ ] **Task 3.4:** Test teleop và sensor data
  ```bash
  ros2 run turtlebot3_teleop teleop_keyboard
  ros2 topic echo /scan
  ```

### Phase 4: SLAM & Map (1 ngày)

- [ ] **Task 4.1:** Run SLAM trong tugbot_warehouse
- [ ] **Task 4.2:** Save map
- [ ] **Task 4.3:** Configure Nav2 waypoints

---

## 9. Potential Issues & Solutions

### 9.1. Fuel Model Download

**Issue:** Models từ Fuel có thể không tự động download.

**Solution:**
```bash
# Pre-download models
gz fuel download -u "https://fuel.gazebosim.org/1.0/MovAi/models/Warehouse"
gz fuel download -u "https://fuel.gazebosim.org/1.0/MovAi/models/Tugbot-charging-station"
gz fuel download -u "https://fuel.gazebosim.org/1.0/MovAi/models/shelf"
gz fuel download -u "https://fuel.gazebosim.org/1.0/MovAi/models/shelf_big"
# ... etc
```

### 9.2. Walking Actor Mesh Path

**Issue:** `model://Walking_actor` có thể không resolve đúng.

**Solution:** Ensure `GZ_SIM_RESOURCE_PATH` includes model directory:
```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/path/to/warehouse_assets/models
```

### 9.3. TurtleBot3 Collision với Warehouse

**Issue:** Robot có thể spawn inside obstacles.

**Solution:** Use safe spawn position `(0, 0, 0.1)` hoặc `(12, -10.6, 0.1)` (near charging station).

---

## 10. So sánh Layout

### 10.1. Tugbot Warehouse vs Original Plan

| Feature | Original Plan (30x40m) | Tugbot Warehouse (~35x50m) |
|---------|------------------------|----------------------------|
| Size | 30m x 40m | ~35m x 50m (larger) |
| Shelves | 24 custom placed | 16 (11 small + 5 big) |
| Charging Station | 3 custom | 1 (Tugbot-charging-station) |
| Actors | 3 custom | 2 Walking actors |
| Complexity | Medium | Medium |
| Setup Time | 5-7 days | 2-3 days |

### 10.2. Advantages của Tugbot Warehouse

1. ✅ **Tiết kiệm thời gian:** Không cần tạo world từ đầu
2. ✅ **Tested models:** Các models từ Fuel đã được verify
3. ✅ **Realistic layout:** Warehouse layout thực tế
4. ✅ **Charging station:** Có sẵn, phù hợp use case
5. ✅ **Flexible:** Dễ thêm/bớt objects

### 10.3. Limitations

1. ⚠️ **Chỉ 1 charging station:** Có thể thêm nếu cần
2. ⚠️ **Fixed warehouse structure:** Không thể thay đổi building
3. ⚠️ **Fuel dependency:** Cần internet để download models lần đầu

---

## 11. References

- [MovAi Tugbot Warehouse - Gazebo Fuel](https://app.gazebosim.org/MovAi/worlds/tugbot_warehouse)
- [Walking Actor - Gazebo Fuel](https://app.gazebosim.org/OpenRobotics/models/Walking%20actor)
- [Gazebo Harmonic Migration Guide](https://gazebosim.org/docs/harmonic/migration)
- [SDF 1.8 Specification](http://sdformat.org/spec?ver=1.8)

---

## Changelog

| Date | Version | Changes |
|------|---------|---------|
| 2024-XX-XX | 0.1 | Initial design (custom 30x40m world) |
| 2024-XX-XX | 0.2 | **Changed to Tugbot Warehouse approach** |
|  |  | - Use existing world from Gazebo Fuel |
|  |  | - Add Walking actor support |
|  |  | - Update for Gazebo Harmonic compatibility |
