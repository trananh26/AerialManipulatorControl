"""
Plot trajectory data from mission runs.
Plots drone path (planned/actual) and arm joint angles when available.
"""

import sys
from pathlib import Path
import argparse

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import ScalarFormatter
from scipy.spatial.transform import Rotation


def find_latest_run(workspace_root):
    """Find the latest run directory in logs/trajectory_runs/."""
    candidates = [
        Path(workspace_root) / 'logs' / 'trajectory_runs',
        Path(workspace_root) / 'install' / 'drone_control' / 'lib' / 'logs' / 'trajectory_runs',
    ]
    found = []
    for root in candidates:
        if root.exists():
            found.extend([d for d in root.iterdir() if d.is_dir()])
    if not found:
        raise FileNotFoundError(
            'No mission runs found in:\n  - '
            + '\n  - '.join(str(c) for c in candidates)
        )
    return sorted(found)[-1]


def preprocess_df(df):
    """Add absolute and relative-ready time column."""
    df = df.copy()
    df['t'] = df['time_sec'].values + df['time_nsec'].values * 1e-9
    return df


def quaternion_to_euler(qx, qy, qz, qw):
    r = Rotation.from_quat([qx, qy, qz, qw])
    return r.as_euler('xyz')


def load_optional_csv(run_dir, name):
    path = Path(run_dir) / name
    if not path.exists():
        return None
    return preprocess_df(pd.read_csv(path))


def fix_axis(ax, title, ylabel):
    ax.set_title(title, fontweight='bold')
    ax.set_ylabel(ylabel)
    ax.yaxis.set_major_formatter(ScalarFormatter(useOffset=False))
    ax.grid(True, alpha=0.3, linestyle='--')
    ax.legend(loc='upper right')


def plot_drone_path(run_dir, raw_pla, raw_act):
    df_act = {col: raw_act[col].values for col in raw_act.columns}
    df_pla = {col: raw_pla[col].values for col in raw_pla.columns}

    t_start = min(df_act['t'].min(), df_pla['t'].min())
    df_act['t_rel'] = df_act['t'] - t_start
    df_pla['t_rel'] = df_pla['t'] - t_start

    has_quaternion = all(
        col in df_act and col in df_pla for col in ['qx', 'qy', 'qz', 'qw'])

    interp_euler = {}
    if has_quaternion:
        euler_act = np.array([
            quaternion_to_euler(
                df_act['qx'][i], df_act['qy'][i],
                df_act['qz'][i], df_act['qw'][i])
            for i in range(len(df_act['qx']))
        ])
        euler_pla = np.array([
            quaternion_to_euler(
                df_pla['qx'][i], df_pla['qy'][i],
                df_pla['qz'][i], df_pla['qw'][i])
            for i in range(len(df_pla['qx']))
        ])
        interp_euler['roll'] = np.interp(
            df_act['t_rel'], df_pla['t_rel'], euler_pla[:, 0])
        interp_euler['pitch'] = np.interp(
            df_act['t_rel'], df_pla['t_rel'], euler_pla[:, 1])
        interp_euler['yaw'] = np.interp(
            df_act['t_rel'], df_pla['t_rel'], euler_pla[:, 2])
        df_act['roll'] = euler_act[:, 0]
        df_act['pitch'] = euler_act[:, 1]
        df_act['yaw'] = euler_act[:, 2]

    fig = plt.figure(figsize=(20, 10))
    fig.suptitle(
        f'Drone Path: {run_dir.name}', fontsize=16, fontweight='bold')
    grid = plt.GridSpec(3, 3, wspace=0.35, hspace=0.4)

    ax_x = fig.add_subplot(grid[0, 0])
    ax_x.plot(df_pla['t_rel'], df_pla['x'], 'r--', label='Planned', linewidth=2)
    ax_x.plot(df_act['t_rel'], df_act['x'], 'b-', label='Actual', alpha=0.7)
    fix_axis(ax_x, 'X Position', 'X (m)')

    ax_y = fig.add_subplot(grid[1, 0])
    ax_y.plot(df_pla['t_rel'], df_pla['y'], 'r--', label='Planned', linewidth=2)
    ax_y.plot(df_act['t_rel'], df_act['y'], 'g-', label='Actual', alpha=0.7)
    fix_axis(ax_y, 'Y Position', 'Y (m)')

    ax_z = fig.add_subplot(grid[2, 0])
    ax_z.plot(df_pla['t_rel'], df_pla['z'], 'r--', label='Planned', linewidth=2)
    ax_z.plot(df_act['t_rel'], df_act['z'], 'm-', label='Actual', alpha=0.7)
    ax_z.set_xlabel('Time (s)')
    fix_axis(ax_z, 'Z Position (Altitude)', 'Z (m)')

    if has_quaternion:
        euler_specs = [
            ('roll', 'Roll Angle', 'Roll (rad)', 'tab:orange'),
            ('pitch', 'Pitch Angle', 'Pitch (rad)', 'tab:cyan'),
            ('yaw', 'Yaw Angle', 'Yaw (rad)', 'tab:brown'),
        ]
        for idx, (key, title, ylabel, color) in enumerate(euler_specs):
            ax = fig.add_subplot(grid[idx, 1])
            ax.plot(df_act['t_rel'], interp_euler[key], 'r--',
                    label='Planned', linewidth=2)
            ax.plot(df_act['t_rel'], df_act[key], color=color,
                    label='Actual', alpha=0.8)
            if idx == 2:
                ax.set_xlabel('Time (s)')
            fix_axis(ax, title, ylabel)
    else:
        ax_note = fig.add_subplot(grid[:, 1])
        ax_note.axis('off')
        ax_note.text(
            0.5, 0.5,
            'Quaternion data not available',
            ha='center', va='center', fontsize=12, color='dimgray')

    ax_3d = fig.add_subplot(grid[:, 2], projection='3d')
    ax_3d.plot(df_pla['x'], df_pla['y'], df_pla['z'], 'r--',
               label='Planned Path', linewidth=2)
    ax_3d.plot(df_act['x'], df_act['y'], df_act['z'], 'b-',
               label='Actual Path', alpha=0.6, linewidth=1.5)
    ax_3d.set_title('3D Trajectory', fontsize=14, fontweight='bold')
    ax_3d.set_xlabel('X (m)')
    ax_3d.set_ylabel('Y (m)')
    ax_3d.set_zlabel('Z (m)')
    ax_3d.legend()

    print(f'Drone path: planned={len(df_pla["x"])}, actual={len(df_act["x"])}, '
          f'duration={df_act["t_rel"][-1]:.2f}s')
    return fig


def _joint_rad_cols(df):
    """Return (j1_rad, j2_rad). Supports new CSV (j1,j2) and legacy names."""
    if 'j1' in df.columns and 'j2' in df.columns:
        return df['j1'].values.astype(float), df['j2'].values.astype(float)
    if 'shoulder_rad' in df.columns and 'elbow_rad' in df.columns:
        return (df['shoulder_rad'].values.astype(float),
                df['elbow_rad'].values.astype(float))
    if 'shoulder_deg' in df.columns and 'elbow_deg' in df.columns:
        return (np.radians(df['shoulder_deg'].values.astype(float)),
                np.radians(df['elbow_deg'].values.astype(float)))
    raise ValueError(
        'Joint CSV needs j1,j2 (rad) or legacy shoulder_*/elbow_* columns')


def plot_joint_path(run_dir, raw_pla, raw_act):
    """Plot joints like drone path: t_rel, planned+actual, interp planned→actual time.

    Drone analogy:
      x,y,z  → j1, j2 (rad stored; deg for display)
      roll/pitch/yaw from quat at plot time → deg from rad at plot time
    """
    if raw_pla is None or raw_act is None or len(raw_pla) == 0 or len(raw_act) == 0:
        print('Need both planned and actual joint CSVs — skipping joint plot')
        return None

    pla_j1, pla_j2 = _joint_rad_cols(raw_pla)
    act_j1, act_j2 = _joint_rad_cols(raw_act)

    df_act_t = raw_act['t'].values
    df_pla_t = raw_pla['t'].values
    t_start = min(df_act_t.min(), df_pla_t.min())
    act_t = df_act_t - t_start
    pla_t = df_pla_t - t_start

    # Degrees for display (like converting quat→euler at plot time)
    pla_j1_deg, pla_j2_deg = np.degrees(pla_j1), np.degrees(pla_j2)
    act_j1_deg, act_j2_deg = np.degrees(act_j1), np.degrees(act_j2)

    # Interpolate planned onto actual time base (same as RPY plot)
    interp_j1 = np.interp(act_t, pla_t, pla_j1_deg)
    interp_j2 = np.interp(act_t, pla_t, pla_j2_deg)

    fig = plt.figure(figsize=(16, 8))
    fig.suptitle(
        f'Arm Joints: {run_dir.name}', fontsize=16, fontweight='bold')
    grid = plt.GridSpec(2, 2, wspace=0.3, hspace=0.35)

    # Column 0: joint vs time (like x/y/z vs time)
    ax_j1 = fig.add_subplot(grid[0, 0])
    ax_j1.plot(pla_t, pla_j1_deg, 'r--', label='Planned', linewidth=2)
    ax_j1.plot(act_t, act_j1_deg, 'b-', label='Actual', alpha=0.7)
    fix_axis(ax_j1, 'Joint 1 (shoulder)', 'Angle (deg)')

    ax_j2 = fig.add_subplot(grid[1, 0])
    ax_j2.plot(pla_t, pla_j2_deg, 'r--', label='Planned', linewidth=2)
    ax_j2.plot(act_t, act_j2_deg, 'g-', label='Actual', alpha=0.7)
    ax_j2.set_xlabel('Time (s)')
    fix_axis(ax_j2, 'Joint 2 (elbow)', 'Angle (deg)')

    # Column 1 top: overlay on common time (like RPY planned interp)
    ax_cmp = fig.add_subplot(grid[0, 1])
    ax_cmp.plot(act_t, interp_j1, 'r--', label='Planned j1', linewidth=2)
    ax_cmp.plot(act_t, act_j1_deg, 'b-', label='Actual j1', alpha=0.7)
    ax_cmp.plot(act_t, interp_j2, color='tab:orange', linestyle='--',
                label='Planned j2', linewidth=2)
    ax_cmp.plot(act_t, act_j2_deg, color='tab:green', label='Actual j2', alpha=0.7)
    fix_axis(ax_cmp, 'Both joints (time-aligned)', 'Angle (deg)')

    # Column 1 bottom: joint space (like 3D path)
    ax_phase = fig.add_subplot(grid[1, 1])
    ax_phase.plot(pla_j1_deg, pla_j2_deg, 'r--', label='Planned', linewidth=2)
    ax_phase.plot(act_j1_deg, act_j2_deg, 'b-', label='Actual', alpha=0.7)
    ax_phase.scatter([pla_j1_deg[0]], [pla_j2_deg[0]], c='red', s=40, zorder=3)
    ax_phase.scatter([act_j1_deg[0]], [act_j2_deg[0]], c='blue', s=40, zorder=3)
    ax_phase.set_xlabel('Joint 1 shoulder (deg)')
    ax_phase.set_ylabel('Joint 2 elbow (deg)')
    fix_axis(ax_phase, 'Joint space', 'Joint 2 (deg)')
    ax_phase.set_aspect('equal', adjustable='datalim')

    print(f'Joint path: planned={len(pla_j1)}, actual={len(act_j1)}, '
          f'duration={act_t[-1]:.2f}s')
    return fig


def main():
    parser = argparse.ArgumentParser(
        description='Plot drone path and arm joint trajectories from mission runs',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python plot_trajectory.py
  python plot_trajectory.py /path/to/run/dir
  python plot_trajectory.py --list
        """,
    )
    parser.add_argument('run_dir', nargs='?', help='Run directory (default: latest)')
    parser.add_argument('--list', action='store_true', help='List available runs')
    args = parser.parse_args()

    script_dir = Path(__file__).parent.parent.parent

    if args.list:
        runs_found = []
        for label, root in [
            ('workspace', script_dir / 'logs' / 'trajectory_runs'),
            ('install', script_dir / 'install' / 'drone_control' / 'lib' /
             'logs' / 'trajectory_runs'),
        ]:
            if root.exists():
                for d in sorted(root.iterdir()):
                    if d.is_dir():
                        runs_found.append((d, label))
        if runs_found:
            print('Available mission runs:')
            for run_path, location in runs_found:
                print(f'  {run_path.name} (from {location}/)')
        else:
            print('No mission runs found')
        return

    run_dir = Path(args.run_dir) if args.run_dir else find_latest_run(script_dir)
    print(f'Loading data from: {run_dir}')

    try:
        raw_pla = load_optional_csv(run_dir, 'planned_path.csv')
        raw_act = load_optional_csv(run_dir, 'actual_path.csv')
        raw_j_pla = load_optional_csv(run_dir, 'planned_joint_path.csv')
        raw_j_act = load_optional_csv(run_dir, 'actual_joint_path.csv')

        figures = []
        if raw_pla is not None and raw_act is not None:
            figures.append(plot_drone_path(run_dir, raw_pla, raw_act))
        else:
            print('Drone path CSV missing/incomplete — skipping drone plot')

        if raw_j_pla is not None or raw_j_act is not None:
            fig_j = plot_joint_path(run_dir, raw_j_pla, raw_j_act)
            if fig_j is not None:
                figures.append(fig_j)
        else:
            print('Joint CSV missing — skipping joint plot '
                  '(expected planned_joint_path.csv / actual_joint_path.csv)')

        if not figures:
            print('Nothing to plot in this run directory.')
            sys.exit(1)

        plt.show()

    except Exception:
        import traceback
        print('\nError loading or plotting data:')
        print(traceback.format_exc())
        sys.exit(1)


if __name__ == '__main__':
    main()
