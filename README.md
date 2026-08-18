# AerialManipulatorControl

ROS 2 Humble + Gazebo Harmonic + ArduPilot SITL workspace for an **X500 quadrotor with a 2-DOF arm**.

## Quick start

1. **Workspace setup (clone / build):**  
   [docs/ROS2_Gazebo_ArduPilot_Setup.md](docs/ROS2_Gazebo_ArduPilot_Setup.md)

2. **Arm mission (sim and hardware), PWM map, logging:**  
   [docs/ARM_CONTROL_USER_GUIDE.md](docs/ARM_CONTROL_USER_GUIDE.md)

**Simulation** (Gazebo + SITL + MAVROS in one launch):

```bash
source install/setup.bash
ros2 launch ardupilot_gz_bringup x500_with_arm_runway.launch.py \
  use_gz_sim_gui:=false rviz:=false Y:=0

# other terminals
ros2 run drone_control path_logger
ros2 run drone_control arm_mission
```

**Hardware** (Pi + Pixhawk — do not launch Gazebo):

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:115200   # or ttyAMA0:921600
ros2 run drone_control arm_fb_bridge --ros-args -p port:=/dev/ttyUSB0
ros2 run drone_control path_logger
ros2 run drone_control arm_mission --ros-args \
  -p arm_only:=true \
  -p joint_states_topic:=/arm/joint_states
```

Trajectory CSVs are written under `logs/trajectory_runs/`. Plot with `Tools/plot_data/plot_trajectory.py`.

## Layout

| Path | Role |
|------|------|
| `src/drone_control/` | App: missions, path logger, arm PWM, FB bridge |
| `src/ardupilot_gazebo/` | Fork: X500+arm SDF, `gazebo-x500.parm` |
| `src/ardupilot_gz/` | Fork: bringup launches / bridges |
| `src/ardupilot/` (+ other submodules) | SITL / Gazebo infra forks |
| `docs/` | Setup, arm guide, RCA notes |
| `firmware/arm_joint_fb/` | ESP32 ADC joint feedback → USB serial |
| `Tools/plot_data/` | Offline trajectory / joint plots |
| `logs/` | Runtime mission CSVs (gitignored) |

Submodules are listed in `.gitmodules`. Fresh checkouts can also use `ros2_gz.repos` (kept in sync with those branches).

## Documentation

| Doc | Description |
|-----|-------------|
| [docs/ROS2_Gazebo_ArduPilot_Setup.md](docs/ROS2_Gazebo_ArduPilot_Setup.md) | Install workspace, build, launch sim |
| [docs/ARM_CONTROL_USER_GUIDE.md](docs/ARM_CONTROL_USER_GUIDE.md) | Arm PWM, **sim vs hardware** run, logger (**canonical**) |
| [docs/X500_ARM_MISSION_GUIDE.md](docs/X500_ARM_MISSION_GUIDE.md) | Redirect → ARM_CONTROL_USER_GUIDE |
| [docs/X500_FLIP_BUG_FIX.md](docs/X500_FLIP_BUG_FIX.md) | RCA / fix for X500 flip in Gazebo |
| [docs/ANN_PID_TUNER.md](docs/ANN_PID_TUNER.md) | PID / ANN tuner notes (separate owner) |
| [CHANGELOG.md](CHANGELOG.md) | Notable milestones (not every commit) |
| [Tools/plot_data/README.md](Tools/plot_data/README.md) | Plot planned/actual path and joint CSVs |
| [firmware/arm_joint_fb/README.md](firmware/arm_joint_fb/README.md) | ESP32 joint ADC firmware + serial protocol |
