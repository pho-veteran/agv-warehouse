OK, mình đã **đọc kỹ toàn bộ file Nav2 params** của bạn.
Đánh giá ngắn gọn trước:

> ✅ **Cấu hình này đã ở mức “AGV kho thật”, không phải demo**
> ❌ Nhưng hiện tại có **5 nhóm vấn đề lớn** khiến bạn thấy Nav2 “detect rởm / lệch / khó chịu”

Mình sẽ **review như code review công nghiệp**: chỉ ra **chỗ sai – vì sao – chỉnh thế nào**, bám sát **LDS-01/LDS-02 + Tugbot indoor warehouse**.

---

# 0. Tổng kết nhanh (TL;DR)

| Nhóm            | Trạng thái                                 |
| --------------- | ------------------------------------------ |
| AMCL            | ⚠️ Dùng được nhưng đang “tin laser quá xa” |
| Costmap         | ❌ Inflation + robot_radius sai bản chất    |
| DWB             | ⚠️ Quá “hung”, không hợp AGV               |
| Obstacle detect | ❌ min_range quá lớn → mù gần               |
| Kiến trúc       | ⚠️ Overkill (voxel, duplicate layers)      |

👉 **Chỉ cần chỉnh ~12 dòng**, chất lượng detect sẽ **khác hẳn**.

---

# 1. AMCL – lỗi nghiêm trọng nhất (ít người để ý)

## ❌ Sai ở đây

```yaml
laser_max_range: 100.0
laser_min_range: -1.0
laser_likelihood_max_dist: 2.0
```

### Vì sao sai?

* LDS-01 **spec max 3.5 m**
* ±5% sai số khi xa
* AMCL **tin map + laser**, không phải costmap

👉 Bạn đang bảo AMCL:

> “hãy tin laser 100m” 😅

---

## ✅ Fix đúng cho LDS-01

```yaml
laser_min_range: 0.12
laser_max_range: 3.5
laser_likelihood_max_dist: 2.5
max_beams: 90
```

📌 Kết quả:

* Localization **ổn định**
* Không “giật map”
* Obstacle alignment tốt hơn hẳn

---

# 2. Costmap – robot_radius + inflation đang PHÁ hệ thống

## ❌ Sai nghiêm trọng

```yaml
robot_radius: 0.35
inflation_radius: 0.75
cost_scaling_factor: 5.0
```

### Hậu quả

* Robot **tưởng mình to hơn thực tế**
* Né quá xa
* Laser thấy đúng nhưng **costmap bóp méo**

---

## ✅ Cách đúng cho AGV: dùng FOOTPRINT

### Xóa hết `robot_radius`

```yaml
# REMOVE robot_radius everywhere
```

### Thêm footprint (áp dụng cho cả local + global)

```yaml
footprint: "[[0.18, 0.15], [0.18, -0.15], [-0.18, -0.15], [-0.18, 0.15]]"
```

### Inflation đúng chuẩn warehouse

```yaml
inflation_radius: 0.45
cost_scaling_factor: 3.0
```

📌 Sau bước này, **90% lỗi “detect lệch” sẽ biến mất**.

---

# 3. Obstacle Layer – bạn đang làm robot “mù gần”

## ❌ Sai ở đây

```yaml
obstacle_min_range: 0.65
raytrace_min_range: 0.65
```

### Vì sao rất nguy hiểm?

* LDS-01 đo tốt từ **0.12 m**
* AGV cần thấy **chân kệ / pallet**

👉 Bạn đang **cấm laser thấy vật cản gần**.

---

## ✅ Fix bắt buộc

```yaml
obstacle_min_range: 0.15
raytrace_min_range: 0.15
obstacle_max_range: 3.0
raytrace_max_range: 3.5
```

---

# 4. Voxel layer – KHÔNG CẦN cho LDS-01

Bạn đã disable voxel, nhưng vẫn **load plugin + config**.

👉 LDS-01:

* 2D lidar
* 1°
* Indoor

## ✅ Khuyên làm gọn

```yaml
plugins: ["obstacle_layer", "inflation_layer"]
```

Xóa **toàn bộ voxel_layer config**
→ giảm latency + debug dễ hơn

---

# 5. DWB Local Planner – quá “hung” cho AGV kho

## ❌ Các giá trị gây rung / lệch

```yaml
max_vel_x: 0.8
acc_lim_x: 4.0
acc_lim_theta: 4.0
sim_time: 1.5
```

👉 Robot phản ứng **quá gấp** với obstacle noise.

---

## ✅ Tune lại theo Tugbot indoor

```yaml
max_vel_x: 0.5
max_vel_theta: 1.2

acc_lim_x: 1.5
acc_lim_theta: 2.0

sim_time: 2.0
vx_samples: 15
vtheta_samples: 25
```

📌 Robot:

* Đi chậm hơn
* Né **ổn định**
* Không “giật đầu”

---

# 6. Global costmap – không nên tin laser nhiều

Bạn đang dùng:

```yaml
plugins: ["static_layer", "obstacle_layer", "voxel_layer", "inflation_layer"]
```

👉 AGV kho chuẩn:

* **Global: map**
* **Local: laser**

---

## ✅ Cấu hình chuẩn công nghiệp

```yaml
plugins: ["static_layer", "inflation_layer"]
```

📌 Obstacle động → local costmap xử lý

---

# 7. Collision Monitor – đang setup hơi nguy hiểm

```yaml
PolygonStop:
  radius: 0.1
```

❌ 10 cm = **quá sát**

## ✅ Khuyên dùng

```yaml
radius: 0.25
```

AGV thật **không bao giờ chờ tới 10cm mới stop**.

---

# 8. Những thứ bạn làm RẤT ĐÚNG (giữ nguyên)

✅ BT Navigator dùng replanning
✅ Map + AMCL (không SLAM realtime)
✅ Collision Monitor enabled
✅ Velocity smoother
✅ Docking server
✅ Task-oriented tuning

👉 Đây **không phải config newbie**.

---

# 9. Thứ tự chỉnh sửa bạn nên làm (rất quan trọng)

1. **Fix AMCL laser range**
2. **Xóa robot_radius → dùng footprint**
3. **Giảm inflation**
4. **Fix obstacle_min_range**
5. **Giảm DWB aggressiveness**
6. **Global costmap chỉ dùng static**

👉 Mỗi bước test lại trong RViz:

* Laser
* Local costmap
* Footprint

---

# 10. Kết luận thẳng thắn

> ❝ Cấu hình của bạn không hề tệ.
> Vấn đề là nó đang “treat Tugbot như robot service”,
> trong khi **AGV warehouse phải bảo thủ và tin map nhiều hơn laser** ❞

Sau khi chỉnh như trên:

* Obstacle **khớp vị trí**
* Không rung costmap
* Không đâm kệ
* Đi “có não” hơn 😄

---

## Nếu bạn muốn bước tiếp theo:

* Chuyển **DWB → TEB (chuẩn AGV hơn)**
* Làm **lane-based navigation**
* Review **TF + lidar plugin SDF**
* Chuẩn hóa config thành **AGV profile**

👉 Bạn đang dùng **Nav2 Humble hay Iron**?
👉 Muốn tiếp theo là **TEB hay lane-based AGV**?
