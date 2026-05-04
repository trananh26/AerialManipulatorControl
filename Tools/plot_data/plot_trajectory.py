"""
Plot trajectory data from mission runs.
Automatically finds the latest run directory in logs/trajectory_runs/
or accepts a custom path as argument.
"""

import sys
import os
from pathlib import Path
import argparse
from datetime import datetime

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import ScalarFormatter
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation


def find_latest_run(workspace_root):
    """Find the latest run directory in logs/trajectory_runs/
    
    Checks in two locations:
    1. workspace_root/logs/trajectory_runs/ (from source)
    2. workspace_root/install/drone_control/lib/logs/trajectory_runs/ (from install)
    """
    # Try workspace root logs first
    trajectory_runs = Path(workspace_root) / 'logs' / 'trajectory_runs'
    if trajectory_runs.exists():
        runs = sorted([d for d in trajectory_runs.iterdir() if d.is_dir()])
        if runs:
            return runs[-1]  # Return latest
    
    # Try install folder as fallback
    install_logs = Path(workspace_root) / 'install' / 'drone_control' / 'lib' / 'logs' / 'trajectory_runs'
    if install_logs.exists():
        runs = sorted([d for d in install_logs.iterdir() if d.is_dir()])
        if runs:
            return runs[-1]
    
    # If neither found, raise error with info about both locations
    raise FileNotFoundError(
        f"No mission runs found in:\n"
        f"  - {trajectory_runs}\n"
        f"  - {install_logs}\n"
        f"Run a mission first (circle_mission or arm_mission) to generate logs."
    )


def load_run_data(run_dir):
    """Load planned_path.csv and actual_path.csv from a run directory"""
    run_path = Path(run_dir)
    
    planned_file = run_path / 'planned_path.csv'
    actual_file = run_path / 'actual_path.csv'
    
    if not planned_file.exists() or not actual_file.exists():
        raise FileNotFoundError(
            f"Missing CSV files in {run_path}\n"
            f"  planned_path.csv: {planned_file.exists()}\n"
            f"  actual_path.csv: {actual_file.exists()}"
        )
    
    return planned_file, actual_file


def preprocess_df(df):
    """Add relative time column to dataframe"""
    t_val = df['time_sec'].values + df['time_nsec'].values * 1e-9
    df['t'] = t_val
    return df


def quaternion_to_euler(qx, qy, qz, qw):
    """Convert quaternion to Euler angles (roll, pitch, yaw) in radians
    
    Args:
        qx, qy, qz, qw: Quaternion components
    
    Returns:
        roll, pitch, yaw: Euler angles in radians
    """
    r = Rotation.from_quat([qx, qy, qz, qw])
    euler_rad = r.as_euler('xyz')  # Roll, Pitch, Yaw in radians
    return euler_rad


def main():
    parser = argparse.ArgumentParser(
        description='Plot trajectory data from mission runs',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Plot latest run
  python plot_trajectory.py
  
  # Plot specific run
  python plot_trajectory.py /path/to/run/dir
  
  # List available runs
  python plot_trajectory.py --list
        """
    )
    
    parser.add_argument(
        'run_dir',
        nargs='?',
        help='Path to mission run directory (default: latest)'
    )
    parser.add_argument(
        '--list',
        action='store_true',
        help='List available mission runs'
    )
    
    args = parser.parse_args()
    
    # Find workspace root (script is in Tools/plot_data/)
    script_dir = Path(__file__).parent.parent.parent  # Go up 3 levels to workspace root
    
    if args.list:
        runs_found = []
        
        # Check workspace root logs
        trajectory_runs = script_dir / 'logs' / 'trajectory_runs'
        if trajectory_runs.exists():
            for d in sorted(trajectory_runs.iterdir()):
                if d.is_dir():
                    runs_found.append((d, "workspace"))
        
        # Check install folder
        install_logs = script_dir / 'install' / 'drone_control' / 'lib' / 'logs' / 'trajectory_runs'
        if install_logs.exists():
            for d in sorted(install_logs.iterdir()):
                if d.is_dir():
                    runs_found.append((d, "install"))
        
        if runs_found:
            print("Available mission runs:")
            for run_path, location in runs_found:
                print(f"  {run_path.name} (from {location}/)")
        else:
            print("No mission runs found")
            print(f"Checked in:")
            print(f"  - {trajectory_runs}")
            print(f"  - {install_logs}")
        return
    
    # Determine which run to plot
    if args.run_dir:
        run_dir = Path(args.run_dir)
    else:
        run_dir = find_latest_run(script_dir)
    
    print(f"Loading data from: {run_dir}")
    
    try:
        planned_file, actual_file = load_run_data(run_dir)
        
        # Load and preprocess data
        raw_act = preprocess_df(pd.read_csv(actual_file))
        raw_pla = preprocess_df(pd.read_csv(planned_file))
        
        # Convert to numpy arrays for interpolation
        df_act = {col: raw_act[col].values for col in raw_act.columns}
        df_pla = {col: raw_pla[col].values for col in raw_pla.columns}
        
        # Normalize time to start at 0 (use MIN of both paths)
        t_start = min(df_act['t'].min(), df_pla['t'].min())
        df_act['t_rel'] = df_act['t'] - t_start
        df_pla['t_rel'] = df_pla['t'] - t_start
        
        # Interpolate planned path to actual time points
        interp_plan = {
            'x': np.interp(df_act['t_rel'], df_pla['t_rel'], df_pla['x']),
            'y': np.interp(df_act['t_rel'], df_pla['t_rel'], df_pla['y']),
            'z': np.interp(df_act['t_rel'], df_pla['t_rel'], df_pla['z']),
        }
        
        # Check if quaternion components exist
        has_quaternion = all(col in df_act and col in df_pla for col in ['qx', 'qy', 'qz', 'qw'])
        
        interp_euler = {}
        if has_quaternion:
            # Convert quaternions to Euler angles for actual path
            euler_act = np.array([quaternion_to_euler(df_act['qx'][i], df_act['qy'][i], 
                                                        df_act['qz'][i], df_act['qw'][i]) 
                                 for i in range(len(df_act['qx']))])
            
            # Convert quaternions to Euler angles for planned path
            euler_pla = np.array([quaternion_to_euler(df_pla['qx'][i], df_pla['qy'][i], 
                                                        df_pla['qz'][i], df_pla['qw'][i]) 
                                 for i in range(len(df_pla['qx']))])
            
            # Interpolate planned Euler angles to actual time points
            interp_euler['roll'] = np.interp(df_act['t_rel'], df_pla['t_rel'], euler_pla[:, 0])
            interp_euler['pitch'] = np.interp(df_act['t_rel'], df_pla['t_rel'], euler_pla[:, 1])
            interp_euler['yaw'] = np.interp(df_act['t_rel'], df_pla['t_rel'], euler_pla[:, 2])
            
            # Store actual Euler angles
            df_act['roll'] = euler_act[:, 0]
            df_act['pitch'] = euler_act[:, 1]
            df_act['yaw'] = euler_act[:, 2]
        
        # Create figure with subplots
        fig = plt.figure(figsize=(20, 10))
        fig.suptitle(f"Mission Run: {run_dir.name}", fontsize=16, fontweight='bold')
        grid = plt.GridSpec(3, 3, wspace=0.35, hspace=0.4)
        
        def fix_axis(ax, title, ylabel):
            ax.set_title(title, fontweight='bold')
            ax.set_ylabel(ylabel)
            ax.yaxis.set_major_formatter(ScalarFormatter(useOffset=False))
            ax.grid(True, alpha=0.3, linestyle='--')
            ax.legend(loc='upper right')
        
        # Position time series
        ax_x = fig.add_subplot(grid[0, 0])
        ax_x.plot(df_pla['t_rel'], df_pla['x'], 'r--', label='Planned', linewidth=2)
        ax_x.plot(df_act['t_rel'], df_act['x'], 'b-', label='Actual', alpha=0.7)
        fix_axis(ax_x, "X Position", "X (m)")
        
        ax_y = fig.add_subplot(grid[1, 0])
        ax_y.plot(df_pla['t_rel'], df_pla['y'], 'r--', label='Planned', linewidth=2)
        ax_y.plot(df_act['t_rel'], df_act['y'], 'g-', label='Actual', alpha=0.7)
        fix_axis(ax_y, "Y Position", "Y (m)")
        
        ax_z = fig.add_subplot(grid[2, 0])
        ax_z.plot(df_pla['t_rel'], df_pla['z'], 'r--', label='Planned', linewidth=2)
        ax_z.plot(df_act['t_rel'], df_act['z'], 'm-', label='Actual', alpha=0.7)
        ax_z.set_xlabel("Time (s)")
        fix_axis(ax_z, "Z Position (Altitude)", "Z (m)")
        
        # Euler angles (if quaternion available)
        if has_quaternion:
            euler_specs = [
                ('roll', 'Roll Angle', 'Roll (rad)', 'tab:orange'),
                ('pitch', 'Pitch Angle', 'Pitch (rad)', 'tab:cyan'),
                ('yaw', 'Yaw Angle', 'Yaw (rad)', 'tab:brown'),
            ]
            for idx, (key, title, ylabel, color) in enumerate(euler_specs):
                ax = fig.add_subplot(grid[idx, 1])
                ax.plot(df_act['t_rel'], interp_euler[key], 'r--', label='Planned', linewidth=2)
                ax.plot(df_act['t_rel'], df_act[key], color=color, label='Actual', alpha=0.8)
                if idx == 2:
                    ax.set_xlabel("Time (s)")
                fix_axis(ax, title, ylabel)
        else:
            ax_note = fig.add_subplot(grid[:, 1])
            ax_note.axis('off')
            ax_note.text(
                0.5, 0.5,
                "Quaternion data not available\n(no qx, qy, qz, qw columns)",
                ha='center', va='center',
                fontsize=12, color='dimgray'
            )
        
        # 3D trajectory plot
        ax_3d = fig.add_subplot(grid[:, 2], projection='3d')
        ax_3d.plot(df_pla['x'], df_pla['y'], df_pla['z'], 'r--', label='Planned Path', linewidth=2)
        ax_3d.plot(df_act['x'], df_act['y'], df_act['z'], 'b-', label='Actual Path', alpha=0.6, linewidth=1.5)
        ax_3d.set_title("3D Trajectory", fontsize=14, fontweight='bold')
        ax_3d.set_xlabel("X (m)")
        ax_3d.set_ylabel("Y (m)")
        ax_3d.set_zlabel("Z (m)")
        ax_3d.legend()
        
        plt.show()
        
        print(f"\nData loaded successfully:")
        print(f"  Planned path points: {len(df_pla['x'])}")
        print(f"  Actual path points: {len(df_act['x'])}")
        print(f"  Duration: {df_act['t_rel'][-1]:.2f} seconds")
        
    except Exception as e:
        import traceback
        print(f"\nError loading or plotting data:")
        print(traceback.format_exc())
        sys.exit(1)


if __name__ == '__main__':
    main()
