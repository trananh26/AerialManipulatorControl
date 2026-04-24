# Trajectory Plotting Tool

Visualize drone trajectory data from mission runs.

## Setup

### 1. Create and activate virtual environment

```bash
cd ~/ros2_project/AerialManipulatorControl/Tools/plot_data
python3 -m venv .venv
source .venv/bin/activate  # On Linux/Mac
# or on Windows:
# .venv\Scripts\activate
```

### 2. Install dependencies

```bash
pip install -r requirements.txt
```

## Usage

### Plot the latest mission run

```bash
python plot_trajectory.py
```

This automatically finds the latest run directory in `logs/trajectory_runs/`

### Plot a specific run

```bash
python plot_trajectory.py /full/path/to/run/directory
```

Example:
```bash
python plot_trajectory.py ~/ros2_project/AerialManipulatorControl/logs/trajectory_runs/arm_mission_20260423_104633
```

### List available runs

```bash
python plot_trajectory.py --list
```

## Output

The script generates:
- **Position vs Time**: X, Y, Z coordinates over mission duration
- **Euler Angles** (if available): Roll, Pitch, Yaw angles
- **3D Trajectory**: Side-by-side visualization of planned vs actual path

Legend:
- **Red dashed**: Planned path
- **Blue/Green/Magenta solid**: Actual path

## Data Format

Expected CSV files in each run directory:
- `planned_path.csv`: Pre-computed waypoints
- `actual_path.csv`: Recorded drone position

Both files must have columns:
- `time_sec`, `time_nsec`: Timestamp
- `x`, `y`, `z`: Position in meters
- (Optional) `roll_deg`, `pitch_deg`, `yaw_deg`: Euler angles
