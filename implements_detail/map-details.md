# Chi tiết Thiết kế Bản đồ Kho Thông minh (Large Warehouse World)

## 1. Tổng quan

### 1.1. Mục tiêu
Tạo một world kho hàng mới với diện tích lớn hơn, hỗ trợ đầy đủ các use case được định nghĩa trong `AGV_Design.md`:
- Vận chuyển hàng đơn lẻ (A → B navigation)
- Tránh vật cản động/tĩnh
- Định vị & bám sát lộ trình
- Sạc pin tự động
- Nhận diện & tương tác với kệ hàng
- Xử lý lỗi & phục hồi

### 1.2. So sánh với World Hiện tại

| Thuộc tính | `no_roof_small_warehouse.world` | **Large Warehouse (Mới)** |
|------------|--------------------------------|---------------------------|
| Kích thước | ~15m x 22m | **30m x 40m** |
| Số kệ hàng | 7 kệ (ShelfD, E, F) | **24+ kệ** (tổ chức theo zones) |
| Trạm sạc | Không có | **2-3 trạm sạc** |
| Khu vực chức năng | Không rõ ràng | **6 zones** (nhập, xuất, lưu trữ, sạc, v.v.) |
| Actors động | Không có | **3-4 actors** (người/forklift ảo) |
| AR Tags | Không có | **Đánh dấu vị trí kệ** |
| Độ phức tạp navigation | Thấp | Trung bình - Cao |

---

## 2. Layout Kho Hàng

### 2.1. Sơ đồ Tổng thể (30m x 40m)

```
    ┌──────────────────────────────────────────────────────────────┐
    │                    KHU NHẬP HÀNG (INBOUND)                   │  Y = 35m → 40m
    │  ┌─────┐  ┌─────┐  ┌─────┐  ┌─────┐  ┌─────┐                 │
    │  │ D1  │  │ D2  │  │ D3  │  │ D4  │  │ D5  │   [Dock Area]   │
    │  └─────┘  └─────┘  └─────┘  └─────┘  └─────┘                 │
    ├──────────────────────────────────────────────────────────────┤
    │                                                              │
    │     ═══════════════ MAIN AISLE ═══════════════               │  Y = 30m
    │                                                              │
    ├────────────────┬─────────────────────────┬───────────────────┤
    │                │                         │                   │
    │  ZONE A        │      ZONE B             │     ZONE C        │
    │  KỆ LƯU TRỮ    │      KỆ LƯU TRỮ         │     KỆ LƯU TRỮ    │
    │                │                         │                   │
    │  ┌───┐ ┌───┐   │  ┌───┐ ┌───┐ ┌───┐     │   ┌───┐ ┌───┐     │  Y = 12m → 28m
    │  │A1 │ │A2 │   │  │B1 │ │B2 │ │B3 │     │   │C1 │ │C2 │     │
    │  ├───┤ ├───┤   │  ├───┤ ├───┤ ├───┤     │   ├───┤ ├───┤     │
    │  │A3 │ │A4 │   │  │B4 │ │B5 │ │B6 │     │   │C3 │ │C4 │     │
    │  ├───┤ ├───┤   │  ├───┤ ├───┤ ├───┤     │   ├───┤ ├───┤     │
    │  │A5 │ │A6 │   │  │B7 │ │B8 │ │B9 │     │   │C5 │ │C6 │     │
    │  └───┘ └───┘   │  └───┘ └───┘ └───┘     │   └───┘ └───┘     │
    │                │                         │                   │
    ├────────────────┴─────────────────────────┴───────────────────┤
    │                                                              │
    │     ═══════════════ SECONDARY AISLE ═══════════════          │  Y = 10m
    │                                                              │
    ├──────────────────────────────────────────────────────────────┤
    │                                                              │
    │  ┌──────────┐                              ┌───────────────┐ │
    │  │ CHARGING │   KHU VỰC ĐỖ XE AGV          │  KHU XUẤT     │ │  Y = 0m → 8m
    │  │ STATION  │   & BẢO TRÌ                  │  (OUTBOUND)   │ │
    │  │  [⚡1]   │                              │  ┌───┐ ┌───┐  │ │
    │  │  [⚡2]   │   ○ AGV Parking              │  │O1 │ │O2 │  │ │
    │  │  [⚡3]   │   ○ Maintenance              │  └───┘ └───┘  │ │
    │  └──────────┘                              └───────────────┘ │
    │                                                              │
    └──────────────────────────────────────────────────────────────┘
         X = 0m                    X = 15m                 X = 30m
```

### 2.2. Định nghĩa các Khu vực (Zones)

| Zone | Mục đích | Vị trí (x, y) | Kích thước |
|------|----------|---------------|------------|
| **Inbound** | Khu nhập hàng, dock tiếp nhận | (0-30, 35-40) | 30m x 5m |
| **Zone A** | Khu lưu trữ nguyên liệu | (0-8, 12-28) | 8m x 16m |
| **Zone B** | Khu lưu trữ chính (bulk storage) | (10-20, 12-28) | 10m x 16m |
| **Zone C** | Khu hàng thành phẩm | (22-30, 12-28) | 8m x 16m |
| **Charging** | Trạm sạc AGV | (0-5, 0-8) | 5m x 8m |
| **Outbound** | Khu xuất hàng | (22-30, 0-8) | 8m x 8m |
| **AGV Area** | Đỗ xe và bảo trì | (6-20, 0-8) | 14m x 8m |

### 2.3. Lối đi (Aisles)

```yaml
aisles:
  main_aisle:
    description: "Lối đi chính ngang, nối Inbound với tất cả zones"
    y_position: 30-32m
    width: 2m
    
  secondary_aisle:
    description: "Lối đi phụ, nối các zones với Charging và Outbound"
    y_position: 10-12m
    width: 2m
    
  vertical_aisle_1:
    description: "Lối đi dọc giữa Zone A và B"
    x_position: 8-10m
    width: 2m
    
  vertical_aisle_2:
    description: "Lối đi dọc giữa Zone B và C"
    x_position: 20-22m
    width: 2m
    
  inner_aisles:
    description: "Lối đi giữa các dãy kệ"
    width: 1.5m
```

---

## 3. Chi tiết Models Sử dụng

### 3.1. Reuse từ `turtlebot3_warehouse_sim`

| Model | Số lượng | Mục đích | Vị trí đặt |
|-------|----------|----------|------------|
| `aws_robomaker_warehouse_WallB_01` | 1 (scaled) | Tường bao quanh | Perimeter |
| `aws_robomaker_warehouse_GroundB_01` | 1 (scaled/tiled) | Nền nhà kho | Toàn bộ |
| `aws_robomaker_warehouse_ShelfE_01` | 12 | Kệ cao (bulk storage) | Zone B |
| `aws_robomaker_warehouse_ShelfD_01` | 8 | Kệ trung bình | Zone A, C |
| `aws_robomaker_warehouse_ShelfF_01` | 4 | Kệ đặc biệt | Inbound, Outbound |
| `aws_robomaker_warehouse_DeskC_01` | 2 | Bàn làm việc | AGV Area |
| `aws_robomaker_warehouse_Bucket_01` | 6 | Thùng chứa nhỏ | Rải rác |
| `aws_robomaker_warehouse_ClutteringA_01` | 4 | Vật cản tĩnh | Corners |
| `aws_robomaker_warehouse_ClutteringC_01` | 4 | Pallet hàng | Near shelves |
| `aws_robomaker_warehouse_ClutteringD_01` | 2 | Hàng hóa ngẫu nhiên | Random |
| `aws_robomaker_warehouse_TrashCanC_01` | 3 | Thùng rác | Aisles |
| `aws_robomaker_warehouse_PalletJackB_01` | 2 | Xe đẩy pallet | Inbound, Outbound |
| `aws_robomaker_warehouse_Lamp_01` | 6 | Đèn chiếu sáng | Ceiling |

### 3.2. Models Mới Cần Tạo

#### 3.2.1. Charging Station Model

```yaml
model_name: warehouse_charging_station
description: "Trạm sạc tượng trưng cho AGV"
components:
  - base_platform:
      type: box
      size: [0.8, 0.6, 0.1]  # meters
      color: [0.2, 0.2, 0.2, 1]  # Dark gray
  - charging_dock:
      type: box
      size: [0.3, 0.1, 0.4]
      color: [0.1, 0.5, 0.1, 1]  # Green
  - indicator_light:
      type: cylinder
      radius: 0.05
      height: 0.1
      color: [0, 1, 0, 1]  # Bright green LED
  - charging_marker:
      type: visual_only
      description: "ArUco/QR marker để AGV nhận diện"
      tag_id: 100-102  # Unique IDs for each station
```

#### 3.2.2. Floor Markers / Navigation Lines

```yaml
model_name: warehouse_floor_marker
types:
  - lane_line:
      description: "Vạch kẻ lane cho AGV"
      color: yellow
      width: 0.1m
  - zone_marker:
      description: "Đánh dấu ranh giới zone"
      color: blue
      width: 0.15m
  - stop_line:
      description: "Vạch dừng tại các giao lộ"
      color: red
      width: 0.2m
```

#### 3.2.3. Waypoint Markers (Visual Reference)

```yaml
model_name: warehouse_waypoint
description: "Điểm tham chiếu cho navigation"
visual:
  type: cylinder
  radius: 0.15
  height: 0.02
  color: [0, 0.7, 0.7, 0.7]  # Semi-transparent cyan
placement:
  - Key intersection points
  - In front of each shelf
  - At charging stations
  - At dock areas
```

---

## 4. Actors (Dynamic Obstacles)

### 4.1. Định nghĩa Actors

Gazebo Harmonic hỗ trợ actors với scripted trajectories. Đây là các "người ảo" hoặc phương tiện di chuyển theo đường định sẵn.

#### Actor 1: Warehouse Worker (Person)

```xml
<actor name="warehouse_worker_1">
  <skin>
    <filename>walk.dae</filename>  <!-- Gazebo default walking animation -->
    <scale>1.0</scale>
  </skin>
  <animation name="walking">
    <filename>walk.dae</filename>
    <scale>1.0</scale>
    <interpolate_x>true</interpolate_x>
  </animation>
  <script>
    <loop>true</loop>
    <delay_start>0.0</delay_start>
    <auto_start>true</auto_start>
    <trajectory id="0" type="walking">
      <!-- Đi từ Zone A → Zone B → Outbound → Zone A -->
      <waypoint>
        <time>0</time>
        <pose>4 20 0 0 0 0</pose>  <!-- Zone A -->
      </waypoint>
      <waypoint>
        <time>10</time>
        <pose>15 20 0 0 0 0</pose>  <!-- Zone B -->
      </waypoint>
      <waypoint>
        <time>20</time>
        <pose>25 4 0 0 0 -1.57</pose>  <!-- Outbound -->
      </waypoint>
      <waypoint>
        <time>35</time>
        <pose>4 20 0 0 0 3.14</pose>  <!-- Back to Zone A -->
      </waypoint>
    </trajectory>
  </script>
</actor>
```

#### Actor 2: Forklift Operator

```xml
<actor name="forklift_path_1">
  <skin>
    <filename>model://forklift_visual/mesh.dae</filename>
    <scale>1.0</scale>
  </skin>
  <script>
    <loop>true</loop>
    <trajectory id="0" type="linear">
      <!-- Route: Inbound → Zone B → Inbound -->
      <waypoint>
        <time>0</time>
        <pose>15 38 0 0 0 -1.57</pose>
      </waypoint>
      <waypoint>
        <time>15</time>
        <pose>15 20 0 0 0 -1.57</pose>
      </waypoint>
      <waypoint>
        <time>25</time>
        <pose>15 20 0 0 0 1.57</pose>  <!-- Turn around -->
      </waypoint>
      <waypoint>
        <time>40</time>
        <pose>15 38 0 0 0 1.57</pose>
      </waypoint>
    </trajectory>
  </script>
</actor>
```

#### Actor 3: Random Cart (Pushed by worker)

```xml
<actor name="moving_cart_1">
  <link name="cart_link">
    <visual name="cart_visual">
      <geometry>
        <box><size>0.8 0.5 0.8</size></box>
      </geometry>
      <material>
        <ambient>0.5 0.5 0.5 1</ambient>
      </material>
    </visual>
    <collision name="cart_collision">
      <geometry>
        <box><size>0.8 0.5 0.8</size></box>
      </geometry>
    </collision>
  </link>
  <script>
    <loop>true</loop>
    <trajectory id="0" type="linear">
      <!-- Route ngẫu nhiên qua secondary aisle -->
      <waypoint>
        <time>0</time>
        <pose>5 11 0.4 0 0 0</pose>
      </waypoint>
      <waypoint>
        <time>20</time>
        <pose>25 11 0.4 0 0 0</pose>
      </waypoint>
      <waypoint>
        <time>40</time>
        <pose>5 11 0.4 0 0 3.14</pose>
      </waypoint>
    </trajectory>
  </script>
</actor>
```

### 4.2. Actor Trajectories Map

```
    ┌──────────────────────────────────────────────────────────────┐
    │                         INBOUND                              │
    │                            ↓                                 │
    │                       [Forklift]                             │
    │                            ↓                                 │
    │     ═══════════════════════════════════════════════════      │
    │         ← ← [Worker 1] → → → → → →                           │
    │     ┌───┐       ↑       ┌───┐       ↓       ┌───┐            │
    │     │   │       │       │   │       │       │   │            │
    │     │   │       │       │   │       │       │   │            │
    │     └───┘       │       └───┘       ↓       └───┘            │
    │                 │                   ↓                         │
    │                 └─────────────────→ ↓                         │
    │     ═══════════════════════════════════════════════════      │
    │         → → → → [Cart] → → → → → → → → → →                   │
    │                                                              │
    │  [Charging]                                   [Outbound]     │
    │                                                    ↑         │
    │                                               Worker 1       │
    └──────────────────────────────────────────────────────────────┘
```

---

## 5. Điểm Quan trọng (POI - Points of Interest)

### 5.1. Waypoints cho Navigation

```yaml
waypoints:
  # Charging Stations
  charging_station_1:
    position: [2.0, 2.0, 0]
    type: charging
    tag_id: 100
    
  charging_station_2:
    position: [2.0, 4.0, 0]
    type: charging
    tag_id: 101
    
  charging_station_3:
    position: [2.0, 6.0, 0]
    type: charging
    tag_id: 102

  # Inbound Docks
  dock_1:
    position: [4.0, 38.0, 0]
    type: inbound_dock
    tag_id: 200
    
  dock_2:
    position: [10.0, 38.0, 0]
    type: inbound_dock
    tag_id: 201
    
  dock_3:
    position: [16.0, 38.0, 0]
    type: inbound_dock
    tag_id: 202

  # Storage Shelves (Zone A - A1 to A6)
  shelf_A1:
    position: [3.0, 25.0, 0]
    type: storage_shelf
    tag_id: 301
    zone: A
    
  shelf_A2:
    position: [6.0, 25.0, 0]
    type: storage_shelf
    tag_id: 302
    zone: A
    
  # ... (continue for all 24 shelves)

  # Outbound Docks
  outbound_1:
    position: [24.0, 4.0, 0]
    type: outbound_dock
    tag_id: 400
    
  outbound_2:
    position: [28.0, 4.0, 0]
    type: outbound_dock
    tag_id: 401
    
  # Key Intersections (for path planning)
  intersection_1:
    position: [9.0, 31.0, 0]
    type: intersection
    
  intersection_2:
    position: [21.0, 31.0, 0]
    type: intersection
    
  intersection_3:
    position: [9.0, 11.0, 0]
    type: intersection
    
  intersection_4:
    position: [21.0, 11.0, 0]
    type: intersection
```

### 5.2. Vị trí đặt Kệ Chi tiết

```yaml
shelves_placement:
  zone_a:
    rows: 3
    columns: 2
    shelf_type: ShelfD_01
    start_position: [3.0, 14.0]
    spacing: [3.5, 4.5]
    shelves:
      - name: A1
        position: [3.0, 25.0, 0]
      - name: A2
        position: [6.0, 25.0, 0]
      - name: A3
        position: [3.0, 20.5, 0]
      - name: A4
        position: [6.0, 20.5, 0]
      - name: A5
        position: [3.0, 16.0, 0]
      - name: A6
        position: [6.0, 16.0, 0]

  zone_b:
    rows: 3
    columns: 3
    shelf_type: ShelfE_01  # Taller shelves for bulk storage
    start_position: [11.0, 14.0]
    spacing: [3.5, 4.5]
    shelves:
      - name: B1
        position: [11.0, 25.0, 0]
      - name: B2
        position: [14.5, 25.0, 0]
      - name: B3
        position: [18.0, 25.0, 0]
      # ... B4-B9

  zone_c:
    rows: 3
    columns: 2
    shelf_type: ShelfD_01
    start_position: [23.0, 14.0]
    spacing: [3.5, 4.5]
    shelves:
      - name: C1
        position: [23.0, 25.0, 0]
      - name: C2
        position: [26.5, 25.0, 0]
      # ... C3-C6
```

---

## 6. Hỗ trợ Use Cases từ AGV_Design.md

### 6.1. Use Case Mapping

| Use Case | World Feature | Implementation |
|----------|---------------|----------------|
| **1. Vận chuyển hàng** | Inbound → Shelves → Outbound paths | Waypoints, clear aisles |
| **2. Tránh vật cản động** | 3-4 Actors với trajectories | Actors following waypoints |
| **3. Tránh vật cản tĩnh** | Clutter models, TrashCans, PalletJacks | Random placement obstacles |
| **4. Định vị** | AR Tags trên kệ, floor markers | Visual markers for AMCL |
| **5. Sạc pin tự động** | 3 Charging stations | Dedicated charging zone |
| **6. Nhận diện kệ hàng** | ArUco tags trên mỗi shelf | Tag IDs 301-324 |
| **7. Xử lý lỗi** | Narrow passages, intersections | Recovery areas designed |
| **8. WMS Integration** | Logical zone division | Zone-based task assignment |

### 6.2. Test Scenarios được hỗ trợ

```yaml
test_scenarios:
  scenario_1_basic_navigation:
    description: "AGV di chuyển từ Charging → Shelf A1 → Outbound"
    waypoints: [charging_station_1, intersection_3, shelf_A1, intersection_1, outbound_1]
    expected_distance: ~45m
    expected_time: ~90s
    
  scenario_2_obstacle_avoidance:
    description: "AGV tránh forklift đang di chuyển"
    route: [dock_1, intersection_1, shelf_B5]
    dynamic_obstacle: forklift_path_1
    
  scenario_3_narrow_passage:
    description: "AGV đi qua lối đi hẹp giữa 2 kệ"
    route: [shelf_B2, shelf_B5]  # Đi qua inner aisle
    passage_width: 1.5m
    
  scenario_4_low_battery_return:
    description: "AGV tự động về trạm sạc khi pin thấp"
    trigger: battery < 20%
    destination: nearest_charging_station
    
  scenario_5_multi_stop_delivery:
    description: "AGV đi nhiều điểm: D1 → A1 → B3 → C2 → Outbound"
    waypoints: [dock_1, shelf_A1, shelf_B3, shelf_C2, outbound_1]
    
  scenario_6_recovery_from_stuck:
    description: "AGV bị kẹt tại intersection, recovery"
    stuck_position: intersection_2
    recovery_action: spin + backup + replan
```

---

## 7. File Structure mới

```
AGV_2/src/turtlebot3_simulations/turtlebot3_warehouse_sim/
├── warehouse_assets/
│   ├── models/
│   │   ├── aws_robomaker_warehouse_*/     # Existing models
│   │   ├── warehouse_charging_station/    # NEW
│   │   │   ├── model.config
│   │   │   ├── model.sdf
│   │   │   └── materials/
│   │   ├── warehouse_floor_marker/        # NEW
│   │   │   ├── model.config
│   │   │   └── model.sdf
│   │   └── warehouse_waypoint/            # NEW
│   │       ├── model.config
│   │       └── model.sdf
│   ├── worlds/
│   │   ├── small_warehouse.world          # Existing
│   │   ├── no_roof_small_warehouse.world  # Existing
│   │   └── large_warehouse.world          # NEW - Main target
│   └── maps/
│       ├── 002/                           # Existing
│       ├── 005/                           # Existing
│       └── large_warehouse/               # NEW (after SLAM)
│           ├── map.pgm
│           ├── map.yaml
│           └── waypoints.yaml
├── launch/
│   ├── turtlebot3_warehouse_world.launch.py
│   └── turtlebot3_large_warehouse.launch.py  # NEW
└── config/
    └── large_warehouse_waypoints.yaml     # NEW
```

---

## 8. Implementation Steps (Phân chia công việc)

### Phase 1: Chuẩn bị Models (2-3 ngày)

- [ ] **Task 1.1:** Tạo model `warehouse_charging_station`
  - Tạo `model.config` và `model.sdf`
  - Test load trong Gazebo

- [ ] **Task 1.2:** Tạo model `warehouse_floor_marker` (optional visual)
  - Lane lines
  - Zone boundaries

- [ ] **Task 1.3:** Scale/tile `GroundB_01` cho 30x40m
  - Hoặc tạo ground mới bằng simple plane

### Phase 2: Tạo World File (2-3 ngày)

- [ ] **Task 2.1:** Tạo `large_warehouse.world` base structure
  - Ground plane 30x40m
  - Boundary walls (có thể dùng simple boxes)
  - Lighting setup (multiple ceiling lights)

- [ ] **Task 2.2:** Đặt shelves theo layout
  - Zone A: 6 ShelfD_01
  - Zone B: 9 ShelfE_01
  - Zone C: 6 ShelfD_01

- [ ] **Task 2.3:** Đặt charging stations
  - 3 stations tại Charging Zone

- [ ] **Task 2.4:** Đặt các obstacles/clutter
  - TrashCans, Buckets, PalletJacks
  - Random positions for variety

### Phase 3: Thêm Actors (1-2 ngày)

- [ ] **Task 3.1:** Thêm Actor 1 (warehouse worker)
  - Trajectory: Zone A → Zone B → Outbound → Zone A

- [ ] **Task 3.2:** Thêm Actor 2 (forklift path)
  - Trajectory: Inbound → Zone B → Inbound

- [ ] **Task 3.3:** Thêm Actor 3 (moving cart)
  - Trajectory: Along secondary aisle

### Phase 4: Launch File & Testing (1-2 ngày)

- [ ] **Task 4.1:** Tạo `turtlebot3_large_warehouse.launch.py`
  - Copy và modify từ existing launch

- [ ] **Task 4.2:** Test world loads correctly
  - Robot spawns
  - All models visible
  - Actors moving

- [ ] **Task 4.3:** Create waypoints config file
  - `large_warehouse_waypoints.yaml`

### Phase 5: SLAM & Map Generation (1-2 ngày)

- [ ] **Task 5.1:** Run SLAM trong large warehouse
- [ ] **Task 5.2:** Save map files
- [ ] **Task 5.3:** Validate map quality

---

## 9. Technical Notes

### 9.1. Gazebo Harmonic Compatibility

```xml
<!-- World file header for Gazebo Harmonic -->
<?xml version='1.0' encoding='utf-8'?>
<sdf version="1.8">  <!-- Use SDF 1.8 for Harmonic -->
  <world name="large_warehouse">
    <!-- Physics config -->
    <physics name="default_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    
    <!-- Required plugins for ros_gz_bridge -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    
    <!-- World content -->
  </world>
</sdf>
```

### 9.2. Model URI Resolution

```
# Model paths sẽ được resolve theo thứ tự:
1. model://model_name  → GZ_SIM_RESOURCE_PATH
2. file:///absolute/path/to/model
3. Relative path từ world file location
```

### 9.3. Actor Animation Notes

```xml
<!-- Gazebo Harmonic actor syntax -->
<actor name="my_actor">
  <!-- For simple box/cylinder actors (no mesh needed) -->
  <link name="link">
    <visual name="visual">
      <geometry><box><size>0.5 0.5 1.8</size></box></geometry>
    </visual>
  </link>
  
  <plugin filename="gz-sim-actor-system" name="gz::sim::systems::Actor">
    <animation_name>walking</animation_name>
    <loop>true</loop>
    <trajectory>
      <waypoint><time>0</time><pose>0 0 0 0 0 0</pose></waypoint>
      <waypoint><time>10</time><pose>10 0 0 0 0 0</pose></waypoint>
    </trajectory>
  </plugin>
</actor>
```

---

## 10. Tài liệu Tham khảo

- [Gazebo Harmonic World Tutorial](https://gazebosim.org/docs/harmonic/building_robot)
- [Gazebo Actor Documentation](https://gazebosim.org/api/sim/8/actor.html)
- [AWS RoboMaker Warehouse World](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world)
- [Nav2 Waypoint Following](https://navigation.ros.org/tutorials/docs/navigation2_with_gps.html)

---

## Changelog

| Date | Version | Changes |
|------|---------|---------|
| 2025-01-XX | 0.1 | Initial design document |

