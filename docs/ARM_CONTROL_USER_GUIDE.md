# Arm Control User Guide

Canonical guide for the X500 + 2-DOF arm stack (`feat/x500-with-arm`).
(`docs/X500_ARM_MISSION_GUIDE.md` redirects here.)

---

## 1. PWM map (SERVO9 / SERVO10)

| Servo | PWM 500 | PWM 1500 | PWM 2500 |
|-------|---------|----------|----------|
| **Sv9 – Shoulder** | 0° **(init)** | 90° | 180° |
| **Sv10 – Elbow** | 0° | 90° **(init)** | 180° |

```
PWM = 500 + angle_deg × (2000 / 180)
```

Code: `drone_control/arm_pwm.py` (`deg_to_pwm`, `pwm_to_rad`).

SITL params (`gazebo-x500.parm`): `SERVO9/10_MIN=500`, `MAX=2500`,
`SERVO9_TRIM=500`, `SERVO10_TRIM=1500`. **`ARMING_CHECK=1`** (all pre-arm checks on).

---

## 2. Init before motors

```
WAIT_CONN → GUIDED → INIT_ARM_POS → WAIT_INIT_ARM (3 s)
  → SEND_ARM → … → takeoff → ARM_SEQUENCE → land
```

Init: shoulder **0°**, elbow **90°** (Gazebo rad == mission rad).

### `arm_only`

```bash
ros2 run drone_control arm_mission --ros-args -p arm_only:=true
```

Skips arming / takeoff / land. Still needs MAVROS + GUIDED for `DO_SET_SERVO`.

---

## 3. How to run (sim)

```bash
source ~/AerialManipulatorControl/install/setup.bash   # adjust path
```

**Terminal 1 — Gazebo (headless, yaw 0):**

```bash
ros2 launch ardupilot_gz_bringup x500_with_arm_runway.launch.py \
  use_gz_sim_gui:=false rviz:=false Y:=0
```

Spawn pose args (`x`,`y`,`z`,`R`,`P`,`Y`) are declared on the runway launch and
forwarded to the robot spawn. Default yaw is **0 rad**.

**Terminal 2 — path logger:**

```bash
ros2 run drone_control path_logger
# optional: -p log_dir:=/path/to/logs
# or: export AERIAL_MANIPULATOR_LOG_DIR=/path/to/logs
```

Default directory: **`<workspace>/logs/trajectory_runs/`**
(workspace = ancestor that contains `src/drone_control`).

**Terminal 3 — mission:**

```bash
ros2 run drone_control arm_mission
# or: --ros-args -p arm_only:=true
```

With `ARMING_CHECK=1`, wait until EKF/GPS are healthy before the mission can arm.

---

## 4. Path logger topics & CSV

| Topic | Type | Role |
|-------|------|------|
| `/planned_path` | `nav_msgs/Path` | Drone planned |
| `/actual_path` | `nav_msgs/Path` | Drone actual |
| `/planned_joint_states` | `sensor_msgs/JointState` | Arm planned (rad) |
| `/actual_joint_states` | `sensor_msgs/JointState` | Arm actual (rad) |
| `/mission/save_log` | `std_msgs/String` | Trigger save (`data` = mission name) |

Each save creates:

```
logs/trajectory_runs/<mission_name>_YYYYMMDD_HHMMSS/
  planned_path.csv          # time_sec,time_nsec,x,y,z,qx,qy,qz,qw
  actual_path.csv
  planned_joint_path.csv    # time_sec,time_nsec,j1,j2  (rad; j1=shoulder, j2=elbow)
  actual_joint_path.csv
```

Plot:

```bash
cd Tools/plot_data && source .venv/bin/activate
python plot_trajectory.py          # latest run
python plot_trajectory.py --list
```

---

## 5. `arm_mission` parameters

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

Hardware feedback (ESP32 bridge): remap to `/arm/joint_states`:

```bash
ros2 run drone_control arm_fb_bridge
ros2 run drone_control arm_mission --ros-args \
  -p joint_states_topic:=/arm/joint_states
```

---

## 6. Default arm sequence (degrees → PWM)

| Step | Shoulder | Elbow |
|------|----------|-------|
| Init | 0° | 90° |
| 1 | 90° | 90° |
| 2a–b | 90° | 135° → 180° |
| 3a–d | 135° → 180° → 135° → 90° | 90° → 90° → 90° → 45° |
| 4 | 90° | 90° |
| 5 | 45° | 90° |
| 6 (home) | 0° | 90° |

Defined as `arm_seq_deg` in `arm_mission.py`.

---

## 7. AP_RobotisServo missing (SITL build)

```bash
cd src/ardupilot
git checkout HEAD -- libraries/AP_RobotisServo/
```
