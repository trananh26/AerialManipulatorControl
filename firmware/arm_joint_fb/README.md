# Arm joint feedback firmware (ESP32 DOIT DevKit V1)

Reads two analog joint feedback channels and streams angles (or raw ADC) over
USB serial for `drone_control/arm_fb_bridge`.

## Hardware


| Signal            | ESP32 pin  | Notes                        |
| ----------------- | ---------- | ---------------------------- |
| Shoulder pot / FB | **GPIO34** | ADC1, input-only             |
| Elbow pot / FB    | **GPIO35** | ADC1, input-only             |
| Pot Vcc           | 3.3 V      | Do not use 5 V into ADC      |
| GND               | GND        | Common with pot and USB host |


Board: **esp32doit-devkit-v1** (PlatformIO).

## Serial protocol (115200 8N1)

Newline-terminated lines, same as `arm_fb_bridge.py`:

```text
S:<shoulder_deg>,E:<elbow_deg>     # default (degrees, 0…270)
A0:<raw>,A1:<raw>                  # raw 12-bit ADC 0…4095
```

Runtime toggle (send one character over serial):

- `d` — degree mode (`S:…,E:…`)
- `r` — raw ADC mode (`A0:…,A1:…`)

Default rate: **50 Hz**.

## Build / flash

### PlatformIO GUI (VS Code / Cursor)

1. Mở thư mục `firmware/arm_joint_fb` trong IDE (hoặc thêm vào workspace).
2. Thanh status bar dưới cùng → nhấn **Upload** (→) để build + flash.
3. Nhấn **Serial Monitor** (🔌) để xem output.

### CLI

```bash
cd firmware/arm_joint_fb
pio run -t upload
pio device monitor   # or: pio run -t monitor
```



## Calibration

Firmware uses the measured ESP32 ADC table (piecewise linear, ADC **falls** as angle rises):

| Angle | PWM | ESP32 ADC (mid) | V | ADS1115 (reference) |
|-------|-----|-----------------|---|---------------------|
| 0° | 500 | 3900 | 3.13 | 24208 |
| 90° | 1166 | 2550 | 2.06 | 16972 |
| 180° | 1833 | 1415 | 1.13 | 9565 |
| 270° | 2500 | 222 | 0.18 | 2214 |

GPIO34/35 currently use the **ESP32 ADC** column in `include/config.h` (`kEsp32Calib`).
The ADS1115 column is stored as `kAds1115Calib` for a later I2C reader.

To retune: flash raw mode (`r`), note ADC at 0/90/180/270, edit the table midpoints.



## ROS bridge (hardware only)

Do not run this in Gazebo — sim already publishes `/x500/joint_states`.
Full hardware steps: [docs/ARM_CONTROL_USER_GUIDE.md](../../docs/ARM_CONTROL_USER_GUIDE.md) §4.

```bash
# close pio device monitor first
ros2 run drone_control arm_fb_bridge --ros-args \
  -p port:=/dev/ttyUSB0 -p baud:=115200

ros2 run drone_control arm_mission --ros-args \
  -p arm_only:=true \
  -p joint_states_topic:=/arm/joint_states
```

Prefer **degree mode** on the MCU (`S:…,E:…`) so the bridge does not need ADC scales.
If the MCU stays in raw mode, convert with the same table on the host rather than a
single `scale/offset` (the curve is piecewise and inverted).