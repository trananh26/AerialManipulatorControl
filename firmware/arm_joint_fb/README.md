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
S:<shoulder_deg>,E:<elbow_deg>     # default (degrees, 0…180)
A0:<raw>,A1:<raw>                  # raw 12-bit ADC 0…4095
```

Runtime toggle (send one character over serial):

- `d` — degree mode (`S:…,E:…`)
- `r` — raw ADC mode (`A0:…,A1:…`)

Default rate: **50 Hz**.

## Build / flash

```bash
cd firmware/arm_joint_fb
pio run -t upload
pio device monitor   # or: pio run -t monitor
```



## Calibration

Edit `include/config.h`:

```cpp
// deg = adc * scale + offset
SHOULDER_ADC_SCALE / SHOULDER_ADC_OFFSET
ELBOW_ADC_SCALE / ELBOW_ADC_OFFSET
```

1. Flash and set raw mode (`r`).
2. Move joints to known angles; note ADC min/max.
3. Set `scale = (deg_max - deg_min) / (adc_max - adc_min)`,
  `offset = deg_min - adc_min * scale`.
4. Switch back to degree mode (`d`) and verify with the bridge.



## ROS bridge

```bash
ros2 run drone_control arm_fb_bridge --ros-args \
  -p port:=/dev/ttyUSB0 -p baud:=115200

ros2 run drone_control arm_mission --ros-args \
  -p joint_states_topic:=/arm/joint_states
```

If the MCU stays in raw mode, set bridge scales instead:

```bash
ros2 run drone_control arm_fb_bridge --ros-args \
  -p port:=/dev/ttyUSB0 \
  -p a0_scale:=0.04396 -p a0_offset:=0.0 \
  -p a1_scale:=0.04396 -p a1_offset:=0.0
```

(`180/4095 ≈ 0.04396` matches the firmware default linear map.)