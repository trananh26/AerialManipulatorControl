# Điều Chỉnh Bộ PID Drone Bằng Mạng Nơ Ron Nhân Tạo (ANN)

## Tổng Quan
Module `pid_ann_tuner.py` được tạo để tự động điều chỉnh các thông số PID (Proportional, Integral, Derivative) của drone sử dụng mạng nơ ron nhân tạo. Cách tiếp cận này sử dụng reinforcement learning để tối ưu hóa hiệu suất điều khiển dựa trên dữ liệu thời gian thực từ drone.

## Thay Đổi Được Thực Hiện

### 1. Cập Nhật Dependencies
- Thêm `tensorflow==2.15.0` vào `requirements.txt` để hỗ trợ xây dựng và chạy mạng nơ ron.

### 2. Tạo Module Mới: `pid_ann_tuner.py`
- **Class PIDANNTuner**: Node ROS2 chính để điều chỉnh PID.
- **Subscribers**: Đăng ký các topic `/mavros/local_position/pose`, `/mavros/local_position/velocity_local`, `/mavros/state` để thu thập dữ liệu vị trí, tốc độ và trạng thái drone.
- **Service Client**: Sử dụng `/mavros/param/set` để cập nhật tham số PID trong ArduPilot.

### 3. Cập Nhật Setup
- Thêm entry point `pid_ann_tuner = drone_control.pid_ann_tuner:main` vào `setup.py` để có thể chạy node như một executable.

## Kiến Trúc Mạng Nơ Ron

### Mô Hình ANN
- **Loại**: Feedforward Neural Network với 3 layers.
- **Input Layer**: 9 neurons (lỗi vị trí X, Y, Z; tốc độ X, Y, Z; lỗi attitude Roll, Pitch, Yaw).
- **Hidden Layer 1**: 128 neurons với activation ReLU.
- **Hidden Layer 2**: 64 neurons với activation ReLU.
- **Output Layer**: 6 neurons (điều chỉnh delta_P_pos, delta_I_pos, delta_D_pos, delta_P_att, delta_I_att, delta_D_att) với activation linear.
- **Optimizer**: Adam
- **Loss Function**: Mean Squared Error (MSE)

### Thuật Toán Học
- **Reinforcement Learning**: Sử dụng reward dựa trên sự giảm lỗi vị trí (prev_error - curr_error).
- **Online Learning**: Retrain model mỗi 100 samples với dữ liệu thu thập được.
- **Parameter Update**: Điều chỉnh PID gains với step size nhỏ (0.01 cho P/D, 0.001 cho I) và clamp trong khoảng [0, 2].

## Cách Sử Dụng

### Cài Đặt
```bash
pip install -r src/drone_control/requirements.txt
```

### Build và Source
```bash
colcon build --packages-select drone_control
source install/setup.bash
```

### Chạy Tuner
```bash
# Chạy tuner trong background
ros2 run drone_control pid_ann_tuner &

# Chạy mission (ví dụ arm_mission)
ros2 run drone_control arm_mission
```

## Tham Số PID Được Điều Chỉnh
Hiện tại hỗ trợ điều chỉnh cho position và attitude control:
- **Position**: `POS_X_P/I/D`, `POS_Y_P/I/D`, `POS_Z_P/I/D`
- **Attitude**: `ROLL_P/I/D`, `PITCH_P/I/D`, `YAW_P/I/D`

## Lưu Ý và Mở Rộng
- **Dữ Liệu Training**: Cần thu thập dữ liệu từ nhiều chuyến bay để ANN học hiệu quả cho cả position và attitude.
- **Mở Rộng**: Có thể thêm support cho velocity control hoặc các PID khác.
- **Tuning**: Điều chỉnh `max_samples`, learning rate, hoặc kiến trúc ANN cho hiệu suất tốt hơn.
- **Safety**: Theo dõi hiệu suất và có cơ chế fallback nếu ANN gây không ổn định.

## Yêu Cầu Hệ Thống
- ROS2 Humble hoặc tương thích
- MAVROS
- ArduPilot SITL hoặc hardware
- Python 3.8+
- TensorFlow 2.15.0