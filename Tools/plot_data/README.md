# Trajectory Plotting Tool

Visualize drone trajectory and arm joint data from mission runs.

## Setup

```bash
cd ~/ros2_project/AerialManipulatorControl/Tools/plot_data
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Usage

```bash
python plot_trajectory.py
python plot_trajectory.py /full/path/to/run/directory
python plot_trajectory.py --list
```

## How storage mirrors drone path

| | Drone path | Arm joints |
|--|------------|------------|
| Actual stamp | `/mavros/.../pose` header | `/x500/joint_states` header |
| Planned stamp | mission clock (ideally same domain) | **same as last joint_states stamp** |
| CSV raw | `x,y,z,qx,qy,qz,qw` | `j1,j2` (rad) |
| Plot derived | roll/pitch/yaw from quat | rad (same as CSV) |
| Time axis | `t_rel = t - min(t)` | same |
| Compare | interp planned → actual time | same |

## CSV files

**`planned_path.csv` / `actual_path.csv`**
```
time_sec, time_nsec, x, y, z, qx, qy, qz, qw
```

**`planned_joint_path.csv` / `actual_joint_path.csv`**
```
time_sec, time_nsec, j1, j2
```
- `j1` = shoulder (rad)
- `j2` = elbow (rad)

## Figures

1. **Drone**: X/Y/Z, RPY, 3D path  
2. **Joints**: j1/j2 vs time, time-aligned overlay, joint-space plot  
