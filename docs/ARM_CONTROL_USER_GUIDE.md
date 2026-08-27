# Arm Control User Guide

Canonical guide for the X500 + 2-DOF arm (`feat/x500-with-arm`).
(`docs/X500_ARM_MISSION_GUIDE.md` redirects here.)

Same mission node (`arm_mission`) for **simulation** and **hardware**.
What changes is the stack underneath: Gazebo/SITL vs Pixhawk + ESP32.

Every new terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_project/AerialManipulatorControl/install/setup.bash   # adjust path
```

---

## 1. PWM map (shared)

Bench-calibrated linear map, 0°…270°:

| Angle | PWM |
|-------|-----|
| 0° | 500 |
| 90° | 1167 |
| 135° | 1500 |
| 180° | 1833 |
| 270° | 2500 |

| Servo | PWM 500 | PWM 1167 | PWM 1833 | PWM 2500 |
|-------|---------|----------|----------|----------|
| **Sv9 – Shoulder** | 0° | 90° **(init)** | 180° | 270° |
| **Sv10 – Elbow** | 0° | 90° **(init)** | 180° | 270° |

```
PWM = 500 + angle_deg × (2000 / 270)
```

PWM 1500 is **135°**, not 90°. Code: `drone_control/arm_pwm.py`.

Set the same range on the flight controller (`SERVO9` / `SERVO10`):

```
SERVO9_FUNCTION   0
SERVO9_MIN        500
SERVO9_MAX        2500
SERVO9_TRIM       500
SERVO10_FUNCTION  0
SERVO10_MIN       500
SERVO10_MAX       2500
SERVO10_TRIM      1167
```

SITL already has this in `gazebo-x500.parm`. On hardware, set the same params in Mission Planner / MAVProxy.

Mission `INIT_ARM_POS` commands both joints to **90°** (straight) via `DO_SET_SERVO` before takeoff.
That is independent of Gazebo SDF spawn / `SERVO*_TRIM` (sim may still spawn shoulder at 0° until mission runs).

---

## 2. Sim vs hardware

```
                    arm_mission  (GUIDED + DO_SET_SERVO)
                           │
              /mavros/local_position/pose
              /mavros/state
                           │
                        MAVROS
                           │
         ┌─────────────────┴─────────────────┐
         │                                   │
   ArduCopter SITL                      Pixhawk (real)
         │                                   │
   Gazebo joints                      SERVO9 / SERVO10 PWM
   /x500/joint_states                 FB5118 + ESP32 ADC
                                             │
                                      arm_fb_bridge
                                      /arm/joint_states
```

| | Simulation | Hardware |
|--|------------|----------|
| FC | SITL (inside Gazebo launch) | Pixhawk + ArduCopter |
| MAVROS | started by `x500_with_arm_runway` | `ros2 launch mavros apm.launch` |
| Arm command | `DO_SET_SERVO` → plugin → Gazebo | `DO_SET_SERVO` → AUX PWM |
| Joint feedback | `/x500/joint_states` | `/arm/joint_states` (ESP32) |
| Pose | `/mavros/local_position/pose` (same topic) | same (needs EKF/GPS or vision) |

Do **not** launch Gazebo on the real aircraft.

---

## 3. Simulation

### 3.1 Terminal 1 — Gazebo + SITL + MAVROS

```bash
ros2 launch ardupilot_gz_bringup x500_with_arm_runway.launch.py \
  use_gz_sim_gui:=false rviz:=false Y:=0
```

Spawn pose args (`x`,`y`,`z`,`R`,`P`,`Y`) are on this launch. Default yaw is **0 rad**.

With `ARMING_CHECK=1`, wait until EKF/GPS are healthy before the mission can arm.

### 3.2 Terminal 2 — path logger (optional)

```bash
ros2 run drone_control path_logger
```

### 3.3 Terminal 3 — mission

Full flight:

```bash
ros2 run drone_control arm_mission
```

Arm only (no takeoff / land):

```bash
ros2 run drone_control arm_mission --ros-args -p arm_only:=true
```

Default feedback topic is `/x500/joint_states` (Gazebo). No ESP32, no `arm_fb_bridge`.

---

## 4. Hardware (Pi companion + Pixhawk)

PWM command still goes **Pixhawk AUX**, not the Pi. The Pi only runs ROS (MAVROS + mission + optional FB bridge).

### 4.1 FC params

On the real Pixhawk, match §1 (`SERVO9/10` `FUNCTION=0`, `MIN=500`, `MAX=2500`, elbow `TRIM=1167`).
PWM wires: shoulder → **AUX1 / SERVO9**, elbow → **AUX2 / SERVO10**. Common GND with the FC.

### 4.2 Terminal 1 — MAVROS

Use the **APM** launch (ArduPilot), not PX4.

```bash
# UART TELEM2 ↔ Pi (typical)
ros2 launch mavros apm.launch fcu_url:=/dev/ttyAMA0:921600

# or USB Pixhawk → Pi
ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:115200
```

Find the port: `ls /dev/ttyACM* /dev/ttyUSB* /dev/ttyAMA* /dev/serial/by-id/`

First time on a machine:

```bash
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

Check:

```bash
ros2 topic echo /mavros/state
# connected: true
```

Full flight also needs a healthy local pose:

```bash
ros2 topic hz /mavros/local_position/pose
```

If this is empty, EKF has no position (GPS / vision). Use `arm_only:=true` on the bench.

### 4.3 Terminal 2 — joint feedback (ESP32)

Close PlatformIO / `pio device monitor` first — only one process may own the serial port.

Flash: see [firmware/arm_joint_fb/README.md](../firmware/arm_joint_fb/README.md).
Prefer MCU **degree mode** (`S:…,E:…`).

```bash
# typical: ls /dev/ttyUSB* /dev/ttyACM*
ros2 run drone_control arm_fb_bridge --ros-args \
  -p port:=/dev/ttyUSB0 -p baud:=115200

# other terminal
ros2 topic echo /arm/joint_states
```

Skip this terminal only if you do not need actual joint angles (open-loop PWM). Planned joints still log; actual joints stay empty.

### 4.4 Terminal 3 — path logger (optional)

```bash
ros2 run drone_control path_logger
```

Same node as sim. CSV is written only if this node is running **and** the mission publishes `/mission/save_log` (normal end / disarm).

### 4.5 Terminal 4 — mission

Bench (props off, arm motion only):

```bash
ros2 run drone_control arm_mission --ros-args \
  -p arm_only:=true \
  -p joint_states_topic:=/arm/joint_states
```

Full flight (GUIDED takeoff → arm sequence → land):

```bash
ros2 run drone_control arm_mission --ros-args \
  -p joint_states_topic:=/arm/joint_states
```

Without `-p joint_states_topic:=/arm/joint_states` the mission still listens to `/x500/joint_states`, which **does not exist** on hardware.

---

## 5. Init and `arm_only`

```
WAIT_CONN → GUIDED → INIT_ARM_POS → WAIT_INIT_ARM (3 s)
  → SEND_ARM → … → takeoff → ARM_SEQUENCE → land
```

Init before takeoff (mission only): shoulder **90°**, elbow **90°** (arm straight).
Does **not** change Gazebo SDF spawn pose.

`arm_only:=true` skips arming / takeoff / land. Still needs MAVROS + GUIDED for `DO_SET_SERVO`.
Use this for bench tests on hardware and for arm-only checks in sim.

---

## 6. `arm_mission` parameters

```bash
ros2 run drone_control arm_mission --ros-args \
  -p takeoff_alt:=3.0 \
  -p hover_time:=3.0 \
  -p move_time_sec:=6.0 \
  -p hold_time_sec:=3.0 \
  -p save_log:=true \
  -p arm_only:=false \
  -p joint_states_topic:=/x500/joint_states
```

| Param | Sim | Hardware |
|-------|-----|----------|
| `joint_states_topic` | `/x500/joint_states` (default) | `/arm/joint_states` |
| `arm_only` | optional | **true** until FC + pose + props are ready |

---

## 7. Default arm sequence (degrees → PWM)

| Step | Shoulder | Elbow |
|------|----------|-------|
| Init | 90° | 90° |
| 1 | 90° | 90° |
| 2a–b | 90° | 135° → 180° |
| 3a–d | 135° → 180° → 135° → 90° | 90° → 90° → 90° → 45° |
| 4 | 90° | 90° |
| 5 | 45° | 90° |
| 6 (home) | 0° | 90° |

Defined as `arm_seq_deg` in `arm_mission.py`.

---

## 8. Path logger topics & CSV

| Topic | Type | Role |
|-------|------|------|
| `/planned_path` | `nav_msgs/Path` | Drone planned |
| `/actual_path` | `nav_msgs/Path` | Drone actual (`/mavros/local_position/pose`) |
| `/planned_joint_states` | `sensor_msgs/JointState` | Arm planned (rad) |
| `/actual_joint_states` | `sensor_msgs/JointState` | Arm actual (rad; from sim or ESP32) |
| `/mission/save_log` | `std_msgs/String` | Trigger save (`data` = mission name) |

Each save creates:

```
logs/trajectory_runs/<mission_name>_YYYYMMDD_HHMMSS/
  planned_path.csv          # time_sec,time_nsec,x,y,z,qx,qy,qz,qw
  actual_path.csv
  planned_joint_path.csv    # time_sec,time_nsec,j1,j2  (rad)
  actual_joint_path.csv
```

Default directory: **`<workspace>/logs/trajectory_runs/`**
(`-p log_dir:=…` or `AERIAL_MANIPULATOR_LOG_DIR`).

Plot:

```bash
cd Tools/plot_data && source .venv/bin/activate
python plot_trajectory.py          # latest run
python plot_trajectory.py --list
```

If hardware flight has **no CSV**: `path_logger` was not running, mission did not reach disarm, or `/mavros/local_position/pose` never published.

---

## 9. AP_RobotisServo missing (SITL build only)

```bash
cd src/ardupilot
git checkout HEAD -- libraries/AP_RobotisServo/
```
