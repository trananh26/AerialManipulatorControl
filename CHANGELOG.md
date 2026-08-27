# Changelog

All notable changes to this project are documented here.
Format inspired by [Keep a Changelog](https://keepachangelog.com/).

Do **not** list every git commit — only milestones useful for demos, bring-up, and release notes.
For full history use `git log`.

---

## [Unreleased]

### Changed

- Docs: submodule sync is **sim PC only**; companion Pi needs main repo only
  (README + `ROS2_Gazebo_ArduPilot_Setup.md` §7)

### Planned

- Calibrate ADC↔angle on real pots / FB and wire end-to-end on hardware

---

## [2026-08-18] — 0–270° servo calibration & sim/hardware docs

### Added

- ESP32 DOIT DevKit V1 firmware (`firmware/arm_joint_fb/`): dual ADC → serial
  `S:deg,E:deg` / `A0,A1` for `arm_fb_bridge`
- Firmware: piecewise-linear ADC→deg (`adcToDeg` + `kEsp32Calib` table)
- `docs/ARM_CONTROL_USER_GUIDE.md`: full **sim vs hardware** runbook
  (Gazebo/SITL §3, Pi + Pixhawk + ESP32 §4)

### Changed

- Arm PWM map **0°…270°** (500–2500): 90°→1167, 180°→1833, 270°→2500
- `PWM_CENTER` removed → **`PWM_90 = 1167`** (elbow init / SERVO10_TRIM)
- Gazebo arm joint upper limit **3.14 → 4.71239** (3π/2)
- `gazebo-x500.parm`: `SERVO10_TRIM` **1500 → 1167**
- Tests updated for 270° map (12 passed)
- README, firmware README (PlatformIO GUI), X500_FLIP_BUG_FIX docs aligned

---

## [2026-07-29] — Workspace layout & bring-up hardening

### Added

- Root `README.md` as project entry point
- Shared `arm_pwm.py` + unit tests for PWM map and CSV schemas
- Canonical arm guide: `docs/ARM_CONTROL_USER_GUIDE.md`



### Changed

- Runway launch forwards spawn pose (`x,y,z,R,P,Y`); default yaw **0**
- `path_logger` default dir → `<workspace>/logs/trajectory_runs/`
- `ros2_gz.repos` synced with `.gitmodules` (`feat/x500-with-arm`)
- `.gitignore` for `frames_*`, `*.tlog*`, IDE/pycache clutter



### Fixed

- Planned drone **Z** no longer draws a long diagonal (stream setpoints with pose)
- Joint plots and arm step logs use **radians**



### Security / safety (sim)

- `ARMING_CHECK=1` in `gazebo-x500.parm` (wait for EKF/GPS before arm)

---



## [2026-07-28] — Arm joints logging & PWM 500–2500



### Added

- Planned/actual joint topics → CSV (`planned_joint_path.csv` / `actual_joint_path.csv`)
- Joint figures in `Tools/plot_data/plot_trajectory.py`
- `arm_fb_bridge` stub (serial → `/arm/joint_states`)



### Changed

- Mission PWM range **500–2500** (0°…180°); Gazebo servo + SITL parm aligned
- Arm sequence defined in **degrees**, converted via `deg_to_pwm`



### Fixed

- Planned joint stamps synced to `/x500/joint_states` time base

---



## [2026-07] — X500 with 2-DOF arm (sim baseline)



### Added

- `x500_with_arm` Gazebo model + `x500_with_arm_runway` launch
- `arm_mission` / `arm_mission_pid_ann` with INIT_ARM before motors, `arm_only` mode
- Elbow CAD rest at mission 90° (Gazebo rad == mission rad)
- Path logger node + offline trajectory plotting



### Fixed

- X500 flip / IMU / PID issues (see `docs/X500_FLIP_BUG_FIX.md`)

---



## [2026-05 … 2026-06] — Flight scenario & tuner (earlier)



### Changed

- Takeoff altitude / hover timings for arm missions
- Flight scenario and PID tuner iterations (`feature/ttanh3-test` line)

