# AerialManipulatorControl

ROS 2 Humble + Gazebo Harmonic + ArduPilot SITL workspace for an **X500 quadrotor with a 2-DOF arm**.

## Quick start

1. **Workspace setup (clone / build):**  
   [docs/ROS2_Gazebo_ArduPilot_Setup.md](docs/ROS2_Gazebo_ArduPilot_Setup.md)

2. **Arm mission, PWM map, logging, spawn pose:**  
   [docs/ARM_CONTROL_USER_GUIDE.md](docs/ARM_CONTROL_USER_GUIDE.md)

```bash
source install/setup.bash

# Headless sim, yaw 0
ros2 launch ardupilot_gz_bringup x500_with_arm_runway.launch.py \
  use_gz_sim_gui:=false rviz:=false Y:=0

# Other terminals
ros2 run drone_control path_logger
ros2 run drone_control arm_mission
```

Trajectory CSVs land in `logs/trajectory_runs/`. Plot with `Tools/plot_data/plot_trajectory.py`.

## Layout

| Path | Role |
|------|------|
| `src/drone_control/` | App: missions, path logger, arm PWM, FB bridge |
| `src/ardupilot_gazebo/` | Fork: X500+arm SDF, `gazebo-x500.parm` |
| `src/ardupilot_gz/` | Fork: bringup launches / bridges |
| `src/ardupilot/` (+ other submodules) | SITL / Gazebo infra forks |
| `docs/` | Setup + arm guide + RCA notes |
| `Tools/plot_data/` | Offline trajectory / joint plots |
| `logs/` | Runtime mission CSVs (gitignored) |

Submodules are listed in `.gitmodules`. Fresh checkouts can also use `ros2_gz.repos` (kept in sync with those branches).

## More docs

- [docs/X500_FLIP_BUG_FIX.md](docs/X500_FLIP_BUG_FIX.md) — X500 flip RCA  
- [docs/ANN_PID_TUNER.md](docs/ANN_PID_TUNER.md) — PID tuner notes (separate owner)
