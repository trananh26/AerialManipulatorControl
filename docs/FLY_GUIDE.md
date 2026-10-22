# Hướng dẫn bay cho X500 + arm

Tài liệu này tổng hợp quy trình bay cho hai chế độ:
- Chế độ mô phỏng: Gazebo + ArduPilot SITL + MAVROS
- Chế độ bay thực tế: PX4/ArduPilot trên Pixhawk + Raspberry Pi/companion

Mục tiêu: giúp bạn bắt đầu nhanh, an toàn, và dễ lặp lại khi chạy mô phỏng hoặc bay thực tế.

---

## 1. Môi trường làm việc bắt buộc

Mỗi terminal mới đều cần source môi trường ROS 2:

```bash
source /opt/ros/humble/setup.bash
source ~/AerialManipulatorControl/install/setup.bash
```

Nếu môi trường build gặp lỗi liên quan tới `setuptools`/`ament_python` (ví dụ `--editable`/`--uninstall` không được hỗ trợ), dùng:

```bash
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
source ~/AerialManipulatorControl/install/setup.bash
```

Đây là workaround đã được xác nhận trong repo để tránh lỗi do phiên bản `setuptools` ở user site quá mới.

---

## 2. Chế độ mô phỏng

### 2.1 Khởi động mô phỏng

Trong terminal 1:

```bash
cd ~/AerialManipulatorControl
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch ardupilot_gz_bringup x500_with_arm_runway.launch.py \
  use_gz_sim_gui:=true rviz:=false Y:=0
```

Giải thích:
- `x500_with_arm_runway.launch.py`: khởi tạo Gazebo + SITL + MAVROS
- `use_gz_sim_gui:=false`: tắt GUI Gazebo nếu muốn chạy headless
- `rviz:=false`: tắt RViz
- `Y:=0`: góc yaw ban đầu

Nếu bạn muốn mở GUI, bỏ các cờ `false` hoặc đổi thành `true`.

### 2.2 Kiểm tra trạng thái FC

Trong terminal 2:

```bash
source /opt/ros/humble/setup.bash
source ~/AerialManipulatorControl/install/setup.bash
ros2 topic echo /mavros/state
```

Bạn cần thấy:

```text
connected: true
```

Ngoài ra có thể kiểm tra vị trí local:

```bash
ros2 topic hz /mavros/local_position/pose
```

Nếu không có dữ liệu, EKF/GPS chưa ổn định, nên đợi cho drone sẵn sàng trước khi bay.

### 2.3 Chạy logger (tùy chọn)

```bash
source /opt/ros/humble/setup.bash
source ~/AerialManipulatorControl/install/setup.bash
ros2 run drone_control path_logger
```

Đây là node ghi dữ liệu quỹ đạo và các thông tin bay để sau này phân tích.

### 2.4 Chạy nhiệm vụ bay / arm

Trong terminal 3:

```bash
source /opt/ros/humble/setup.bash
source ~/AerialManipulatorControl/install/setup.bash
ros2 run drone_control arm_mission
```

Nếu muốn chỉ điều khiển cánh tay / arm mà không cất cánh:

```bash
ros2 run drone_control arm_mission --ros-args -p arm_only:=true
```

Trong chế độ mô phỏng, feedback của cánh tay thường là topic:

```text
/x500/joint_states
```

Không cần `arm_fb_bridge` trong sim.

### 2.5 Quy trình bay mô phỏng an toàn

1. Khởi động Gazebo + SITL.
2. Kiểm tra `/mavros/state` đã `connected: true`.
3. Chờ drone ổn định và có pose/local position.
4. Nếu cần, chạy `arm_mission` với `arm_only:=true` để kiểm tra servo/arm trước.
5. Khi sẵn sàng, chạy `arm_mission` đầy đủ hoặc nhiệm vụ bay mong muốn.
6. Sau khi hạ cánh hoặc dừng chế độ, kiểm tra log nếu cần.

---

## 3. Chế độ bay thực tế

### 3.1 Lưu ý quan trọng

- Đừng khởi động Gazebo khi bay thật.
- Máy bay thực tế sử dụng Pixhawk / ArduPilot, không dùng mô phỏng.
- Cánh tay được điều khiển bằng PWM ra AUX channel (`SERVO9` / `SERVO10`).
- ROS node trên Pi chỉ điều khiển logic; tín hiệu motor/servo cuối cùng vẫn ra từ flight controller.

### 3.2 Thiết lập tham số trên FC

Các tham số cần đồng bộ với map PWM trong repo:

```text
SERVO9_FUNCTION   0
SERVO9_MIN        500
SERVO9_MAX        2500
SERVO9_TRIM       500

SERVO10_FUNCTION  0
SERVO10_MIN       500
SERVO10_MAX       2500
SERVO10_TRIM      1167
```

PWM map quan trọng:

| Góc | PWM |
|-----|-----|
| 0°  | 500 |
| 90° | 1167 |
| 135°| 1500 |
| 180°| 1833 |
| 270°| 2500 |

Trong thực tế:
- Shoulder / Sv9 → AUX1 / SERVO9
- Elbow / Sv10 → AUX2 / SERVO10
- GND chung với FC

### 3.3 Khởi động MAVROS trên Pi

Với cấu hình của bạn, Pixhawk được nối vào Pi qua cổng USB Type-C, nên lệnh chạy MAVROS nên dùng cổng USB serial của Pixhawk, ví dụ:

```bash
source /opt/ros/humble/setup.bash
source ~/AerialManipulatorControl/install/setup.bash

ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:115200
```

Nếu cổng thực tế khác, kiểm tra bằng:

```bash
ls /dev/ttyACM* /dev/ttyUSB* /dev/ttyAMA* /dev/serial/by-id/
```

Rồi thay `fcu_url` tương ứng, ví dụ:

```bash
ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM1:115200
```

> Với setup của bạn, không cần dùng `telem2` nữa. Cổng Pixhawk Type-C kết nối trực tiếp vào Pi qua USB sẽ hiện lên như `/dev/ttyACM*`.

Nếu lần đầu mới cài trên máy:

```bash
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

Kiểm tra trạng thái FC:

```bash
ros2 topic echo /mavros/state
```

Cần thấy:

```text
connected: true
```

Kiểm tra local pose:

```bash
ros2 topic hz /mavros/local_position/pose
```

Nếu topic rỗng, nghĩa là hệ thống định vị chưa sẵn sàng. Với điều kiện thử cánh tay trên bàn, bạn có thể dùng `arm_only:=true`.

### 3.4 Khởi động feedback cánh tay (ESP32)

Với cấu hình của bạn, ESP32 đo ADC được nối vào cổng USB của Pi4. Vì vậy port thường là `/dev/ttyUSB0` hoặc `/dev/ttyACM1` tùy thiết bị. Kiểm tra trước:

```bash
ls /dev/ttyUSB* /dev/ttyACM*
```

Nếu dùng ESP32 trên USB Pi:

```bash
source /opt/ros/humble/setup.bash
source ~/AerialManipulatorControl/install/setup.bash

ros2 run drone_control arm_fb_bridge --ros-args \
  -p port:=/dev/ttyUSB0 -p baud:=115200
```

Nếu ESP32 hiện lên là `/dev/ttyACM1`, chạy:

```bash
ros2 run drone_control arm_fb_bridge --ros-args \
  -p port:=/dev/ttyACM1 -p baud:=115200
```

Kiểm tra topic:

```bash
ros2 topic echo /arm/joint_states
```

Nếu không dùng cảm biến thực, có thể bỏ qua bước này. Tuy nhiên, khi đó mission trên hardware không có dữ liệu joint thực tế.

### 3.5 Chạy logger (tùy chọn)

```bash
source /opt/ros/humble/setup.bash
source ~/AerialManipulatorControl/install/setup.bash
ros2 run drone_control path_logger
```

### 3.6 Chạy mission trên hardware

#### Mục đích kiểm tra arm trên bàn / props tắt

```bash
ros2 run drone_control arm_mission --ros-args \
  -p arm_only:=true \
  -p joint_states_topic:=/arm/joint_states
```

#### Bay đầy đủ (GUIDED -> cất cánh -> thực thi arm -> hạ cánh)

```bash
ros2 run drone_control arm_mission --ros-args \
  -p joint_states_topic:=/arm/joint_states
```

Lưu ý:
- Không bỏ qua `-p joint_states_topic:=/arm/joint_states` khi chạy trên hardware.
- Nếu không đặt tham số này, node sẽ mặc định lắng nghe `/x500/joint_states`, nhưng topic này tồn tại trong sim, không phải trên máy bay thật.

---

## 4. Sơ đồ nhanh

### Chế độ mô phỏng

```text
Gazebo + SITL + MAVROS
    ↓
ros2 launch ardupilot_gz_bringup x500_with_arm_runway.launch.py
    ↓
ros2 run drone_control arm_mission
```

### Chế độ máy bay thật

```text
Pixhawk (USB Type-C → Pi)
    ↓
ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM*
    ↓
ESP32 ADC (USB → Pi)
    ↓
ros2 run drone_control arm_fb_bridge --ros-args -p port:=/dev/ttyUSB0
    ↓
ros2 run drone_control arm_mission --ros-args -p arm_only:=true
```

---

## 5. Checklist trước khi bay

### Trước khi chạy mô phỏng
- [ ] Source ROS 2
- [ ] Source workspace
- [ ] Khởi động Gazebo/SITL thành công
- [ ] `/mavros/state` hiển thị `connected: true`
- [ ] Có local position / EKF ổn định
- [ ] Chạy test arm nếu cần

### Trước khi bay thật
- [ ] FC đã cấu hình đúng SERVO9/SERVO10
- [ ] Tín hiệu PWM/servo đúng dây và GND chung
- [ ] Pixhawk qua USB Type-C đang được Pi nhận dạng thành `/dev/ttyACM*`
- [ ] ESP32 qua USB của Pi đang được nhận dạng thành `/dev/ttyUSB*` hoặc `/dev/ttyACM*`
- [ ] `ros2 launch mavros apm.launch` kết nối thành công
- [ ] `/mavros/state` là `connected: true`
- [ ] `/mavros/local_position/pose` có dữ liệu hoặc biết rõ trạng thái định vị
- [ ] Nếu dùng ESP32 thì `/arm/joint_states` đang phát dữ liệu
- [ ] Props tắt nếu đang test arm ở mặt đất

---

## 6. Khắc phục sự cố nhanh

### Lỗi build/ROS 2

Nếu gặp lỗi liên quan đến `setuptools` và `ament_python`:

```bash
export PYTHONNOUSERSITE=1
```

Rồi source lại workspace.

### MAVROS không connect

- Kiểm tra Pixhawk đã xuất hiện trên Pi như `/dev/ttyACM*`
- Không cần dùng telem 2 trong cấu hình này
- Kiểm tra baud `115200` hoặc đúng baud FC
- Kiểm tra `ros2 topic echo /mavros/state`

### ESP32 không hiện port

- Vào `/dev/ttyUSB*` và `/dev/ttyACM*` kiểm tra
- Có thể cần cắm lại USB hoặc khởi động lại Pi
- Nếu hiện `/dev/ttyACM*` thay vì `/dev/ttyUSB0`, dùng port đó

### Topic `/arm/joint_states` trống

- Kiểm tra port của ESP32 đúng
- Xem `arm_fb_bridge` đang chạy hay chưa
- Nếu không cần feedback thực, dùng `arm_only:=true` và kiểm tra bằng cách bay thử ở mức an toàn

### Mission không nhận joint state trong sim

- Dùng default topic `/x500/joint_states`
- Không dùng `/arm/joint_states` khi chạy ở sim

---

## 7. Khuyến nghị thực tế với cấu hình của bạn

- Với Pixhawk qua USB Type-C vào Pi, bạn nên kiểm tra port thật bằng `ls /dev/ttyACM*` trước khi launch MAVROS.
- Với ESP32 qua USB Pi, nên kiểm tra `ls /dev/ttyUSB* /dev/ttyACM*` trước khi chạy `arm_fb_bridge`.
- Nếu cần test nhanh, hãy bắt đầu với `arm_only:=true` ở mặt đất, props tắt.
- Sau đó mới thử chế độ bay hoàn chỉnh khi kết nối FC và ESP32 đều ổn định.

---

## 8. Lệnh rút gọn cho cấu hình của bạn

### Simulation

```bash
source /opt/ros/humble/setup.bash
source ~/AerialManipulatorControl/install/setup.bash
ros2 launch ardupilot_gz_bringup x500_with_arm_runway.launch.py use_gz_sim_gui:=false rviz:=false Y:=0
ros2 run drone_control path_logger
ros2 run drone_control arm_mission
```

### Real flight (Pixhawk USB Type-C + ESP32 USB)

```bash
source /opt/ros/humble/setup.bash
source ~/AerialManipulatorControl/install/setup.bash

ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:115200
ros2 run drone_control arm_fb_bridge --ros-args -p port:=/dev/ttyUSB0 -p baud:=115200
ros2 run drone_control path_logger
ros2 run drone_control arm_mission --ros-args -p joint_states_topic:=/arm/joint_states
```

Nếu port khác, thay đúng `/dev/ttyACM*` và `/dev/ttyUSB*` tương ứng.

---

Nếu bạn muốn, tôi có thể tiếp tục chỉnh file này thành một phiên bản cực ngắn theo kiểu “bay thật: cắm gì, port nào, chạy lệnh nào” để bạn copy-paste và chạy luôn.

Nếu lần đầu mới cài trên máy:

```bash
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

Kiểm tra trạng thái FC:

```bash
ros2 topic echo /mavros/state
```

Cần thấy:

```text
connected: true
```

Kiểm tra local pose:

```bash
ros2 topic hz /mavros/local_position/pose
```

Nếu topic rỗng, nghĩa là hệ thống định vị chưa sẵn sàng. Với điều kiện thử cánh tay trên bàn, bạn có thể dùng `arm_only:=true`.

### 3.4 Khởi động feedback cánh tay (ESP32)

Nếu dùng cảm biến góc cánh tay qua ESP32:

```bash
source /opt/ros/humble/setup.bash
source ~/AerialManipulatorControl/install/setup.bash

ros2 run drone_control arm_fb_bridge --ros-args \
  -p port:=/dev/ttyUSB0 -p baud:=115200
```

Kiểm tra topic:

```bash
ros2 topic echo /arm/joint_states
```

Nếu không dùng cảm biến thực, có thể bỏ qua bước này. Tuy nhiên, khi đó mission trên hardware không có dữ liệu joint thực tế.

### 3.5 Chạy logger (tùy chọn)

```bash
source /opt/ros/humble/setup.bash
source ~/AerialManipulatorControl/install/setup.bash
ros2 run drone_control path_logger
```

### 3.6 Chạy mission trên hardware

#### Mục đích kiểm tra arm trên bàn / props tắt

```bash
ros2 run drone_control arm_mission --ros-args \
  -p arm_only:=true \
  -p joint_states_topic:=/arm/joint_states
```

#### Bay đầy đủ (GUIDED -> cất cánh -> thực thi arm -> hạ cánh)

```bash
ros2 run drone_control arm_mission --ros-args \
  -p joint_states_topic:=/arm/joint_states
```

Lưu ý:
- Không bỏ qua `-p joint_states_topic:=/arm/joint_states` khi chạy trên hardware.
- Nếu không đặt tham số này, node sẽ mặc định lắng nghe `/x500/joint_states`, nhưng topic này tồn tại trong sim, không phải trên máy bay thật.

---

## 4. Sơ đồ nhanh

### Chế độ mô phỏng

```text
Gazebo + SITL + MAVROS
    ↓
ros2 launch ardupilot_gz_bringup x500_with_arm_runway.launch.py
    ↓
ros2 run drone_control arm_mission
```

### Chế độ máy bay thật

```text
Pixhawk + ArduPilot
    ↓
ros2 launch mavros apm.launch
    ↓
ros2 run drone_control arm_fb_bridge
    ↓
ros2 run drone_control arm_mission --ros-args -p arm_only:=true
```

---

## 5. Checklist trước khi bay

### Trước khi chạy mô phỏng
- [ ] Source ROS 2
- [ ] Source workspace
- [ ] Khởi động Gazebo/SITL thành công
- [ ] `/mavros/state` hiển thị `connected: true`
- [ ] Có local position / EKF ổn định
- [ ] Chạy test arm nếu cần

### Trước khi bay thật
- [ ] FC đã cấu hình đúng SERVO9/SERVO10
- [ ] Tín hiệu PWM/servo đúng dây và GND chung
- [ ] `ros2 launch mavros apm.launch` kết nối thành công
- [ ] `/mavros/state` là `connected: true`
- [ ] `/mavros/local_position/pose` có dữ liệu hoặc biết rõ trạng thái định vị
- [ ] Nếu dùng ESP32 thì `/arm/joint_states` đang phát dữ liệu
- [ ] Props tắt nếu đang test arm ở mặt đất

---

## 6. Khắc phục sự cố nhanh

### Lỗi build/ROS 2

Nếu gặp lỗi liên quan đến `setuptools` và `ament_python`:

```bash
export PYTHONNOUSERSITE=1
```

Rồi source lại workspace.

### MAVROS không connect

- Kiểm tra đúng cổng serial
- Kiểm tra baud rate
- Kiểm tra `ros2 topic echo /mavros/state`
- Đảm bảo FC đang chạy ArduPilot đúng mode

### Topic `/arm/joint_states` trống

- Xem ESP32 đã được flash đúng firmware chưa
- Kiểm tra port `/dev/ttyUSB*` hoặc `/dev/ttyACM*`
- Kiểm tra `arm_fb_bridge` đang chạy
- Nếu không cần feedback thực, dùng `arm_only:=true` và kiểm tra hành vi servo bằng cách type rõ ràng

### Mission không nhận joint state trong sim

- Dùng default topic `/x500/joint_states`
- Không dùng `/arm/joint_states` khi chạy ở sim

---

## 7. Khuyến nghị thực tế

- Mỗi lần bay mới, nên bắt đầu bằng `arm_only:=true` để test cơ cấu arm ở mặt đất.
- Với mô phỏng, ưu tiên xét nghiệm quỹ đạo và động học cơ cấu trước khi chuyển sang bay thật.
- Với bay thật, luôn kiểm tra trạng thái GPS/EKF, UART, và điện áp trước khi cất cánh.
- Ghi log mỗi lần thử nghiệm để dễ đối chiếu dữ liệu và điều chỉnh tham số.

---

## 8. Lệnh rút gọn

### Simulation

```bash
source /opt/ros/humble/setup.bash
source ~/AerialManipulatorControl/install/setup.bash
ros2 launch ardupilot_gz_bringup x500_with_arm_runway.launch.py use_gz_sim_gui:=true rviz:=false Y:=0
ros2 run drone_control path_logger
ros2 run drone_control arm_mission
```

### Real flight

```bash
source /opt/ros/humble/setup.bash
source ~/AerialManipulatorControl/install/setup.bash
ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:115200
ros2 run drone_control arm_fb_bridge --ros-args -p port:=/dev/ttyUSB0 -p baud:=115200
ros2 run drone_control path_logger
ros2 run drone_control arm_mission --ros-args -p joint_states_topic:=/arm/joint_states
```

---

Nếu cần, tôi có thể tiếp tục viết thêm một phiên bản ngắn gọn dạng checklist “mỗi lần bay chỉ cần làm 5 bước” hoặc tạo luôn một file `README_fly.md` để gắn ở root workspace.
