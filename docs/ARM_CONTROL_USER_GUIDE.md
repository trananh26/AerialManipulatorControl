# Changelog & User Guide

Tài liệu này mô tả các thay đổi trong nhánh `feat/x500-with-arm`.

---

## 1. Servo Motor Model – PWM range mới

### Thay đổi
| Servo | PWM 500 | PWM 1500 | PWM 2500 |
|-------|---------|----------|----------|
| **Sv9 – Shoulder** (`SERVO9 / AUX1`) | 0° **(init)** | 90° | 180° |
| **Sv10 – Elbow** (`SERVO10 / AUX2`) | 0° | 90° **(init)** | 180° |

Dải PWM trước đây là 1100–1900 (range 800), nay đổi thành **500–2500 (range 2000)** để khớp với servo thực tế.

### Constants trong code
```python
PWM_MIN    = 500    # 0°
PWM_CENTER = 1500   # 90°
PWM_MAX    = 2500   # 180°
```

### Công thức tính PWM từ góc
```
PWM = 500 + angle_deg × (2000 / 180)
```

---

## 2. State Machine – INIT_ARM_POS trước khi arm drone

### Mô tả
Thêm 2 state mới vào đầu mission để đưa arm về vị trí khởi đầu an toàn **trước khi** arm động cơ drone, tránh arm di chuyển đột ngột khi drone bắt đầu quay motor.

### Flow mới
```
WAIT_CONN → SEND_GUIDED → WAIT_GUIDED
    → INIT_ARM_POS    ← gửi servo về init position
    → WAIT_INIT_ARM   ← chờ 3 giây để arm di chuyển xong
    → SEND_ARM → WAIT_ARM → ...
```

### Vị trí init
- Sv9 (Shoulder): **500 PWM = 0°**
- Sv10 (Elbow): **1500 PWM = 90°**

### Tùy chỉnh thời gian chờ
Sửa `INIT_ARM_DURATION` trong `arm_mission.py` (mặc định 3.0 giây):
```python
self.INIT_ARM_DURATION = 3.0  # seconds
```

---

## 3. Chế độ arm_only – Điều khiển arm không cần takeoff

### Mô tả
Thêm parameter `arm_only` cho phép chạy toàn bộ arm sequence mà **không cần arm drone, takeoff, hay land**. Hữu ích khi test arm trên mặt đất.

### Flow khi `arm_only:=true`
```
WAIT_CONN → SEND_GUIDED → WAIT_GUIDED
    → INIT_ARM_POS → WAIT_INIT_ARM
    → ARM_SEQUENCE   ← bỏ qua SEND_ARM/WAIT_ARM/TAKEOFF
    → DONE           ← bỏ qua HOVER/LAND
```

### Lưu ý
- Drone vẫn cần kết nối MAVROS và ở chế độ GUIDED để nhận lệnh servo.
- Drone **không** được arm động cơ → cánh quạt không quay.

---

##  4. Cách chạy
### Mô phỏng
Terminal 1:
```bash
ros2 launch ardupilot_gz_bringup x500_with_arm_runway.launch.py
```

Terminal 2:
```bash
# Terminal riêng, chạy song song với mission node
ros2 run drone_control path_logger

# Hoặc
# Tùy chỉnh thư mục lưu log (mặc định: <workspace>/logs/trajectory_runs/)
ros2 run drone_control path_logger --ros-args -p log_dir:=~/AMLogs
```
Terminal 3:

**Chỉ điều khiển arm (không takeoff):**
```bash
ros2 run drone_control arm_mission --ros-args -p arm_only:=true
```

**Chạy mission đầy đủ (arm drone + takeoff + arm sequence + land):**
```bash
ros2 run drone_control arm_mission
# hoặc
ros2 run drone_control arm_mission --ros-args -p arm_only:=false
```
**Chạy mission mô phỏng bỏ qua giao diện và set Yaw=0:**
```bash
ros2 launch ardupilot_gz_bringup x500_with_arm_runway.launch.py use_gz_sim_gui:=false rviz:=false Y:=0
```

### Output
Mỗi lần lưu tạo thư mục mới:
```
logs/trajectory_runs/<mission_name>_YYYYMMDD_HHMMSS/
    planned_path.csv    # x, y, z, qx, qy, qz, qw theo thời gian
    actual_path.csv     # x, y, z, qx, qy, qz, qw theo thời gian
```

### Trigger thủ công
```bash
ros2 topic pub --once /mission/save_log std_msgs/String "data: 'my_test'"
```

---

## 5. arm_mission – Tất cả parameters

```bash
ros2 run drone_control arm_mission --ros-args \
    -p takeoff_alt:=5.0 \        # Độ cao takeoff (m)
    -p hover_time:=3.0 \         # Thời gian hover trước/sau arm (s)
    -p move_time_sec:=5.0 \      # Thời gian di chuyển mỗi bước arm (s)
    -p hold_time_sec:=1.0 \      # Thời gian giữ nguyên sau mỗi bước (s)
    -p max_acceleration:=50.0 \  # Gia tốc tối đa S-curve (PWM/s²)
    -p jerk_limit:=100.0 \       # Giới hạn jerk S-curve (PWM/s³)
    -p save_log:=true \          # Tự động trigger lưu log sau mission
    -p arm_only:=false           # true = chỉ chạy arm, không takeoff
```

---

## 6. Fix – AP_RobotisServo missing library

### Vấn đề
Build ArduPilot SITL thất bại với lỗi:
```
fatal error: AP_RobotisServo/AP_RobotisServo.h: No such file or directory
```

### Nguyên nhân
Thư mục `src/ardupilot/libraries/AP_RobotisServo/` tồn tại trong git tree nhưng bị thiếu nội dung trong working directory (checkout không đầy đủ).

### Fix
```bash
cd src/ardupilot
git checkout HEAD -- libraries/AP_RobotisServo/
```

---

## 7. Arm Sequence mặc định

| Bước | Shoulder (Sv9) | Elbow (Sv10) |
|------|----------------|--------------|
| Init (trước takeoff) | 500 PWM = 0° | 1500 PWM = 90° |
| Step 1 | 1500 PWM = 90° | 1500 PWM = 90° |
| Step 2 | 1500 PWM = 90° | 2000 PWM = 135° |
| Step 3 | 2000 PWM = 135° | 1000 PWM = 45° |
| Step 4 | 1000 PWM = 45° | 1500 PWM = 90° |
| Step 5 (home) | 500 PWM = 0° | 1500 PWM = 90° |
