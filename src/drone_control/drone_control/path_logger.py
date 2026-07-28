"""
PathLoggerNode: saves drone paths and arm joint trajectories to CSV.

Subscribes:
  /planned_path, /actual_path              (nav_msgs/Path)
  /planned_joint_states, /actual_joint_states  (sensor_msgs/JointState samples)
  /mission/save_log                        (std_msgs/String trigger)

Mission nodes publish save_log with the mission name (e.g. "arm_mission").
"""

import csv
import os
import time

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class PathLoggerNode(Node):

    def __init__(self):
        super().__init__('path_logger')

        self.declare_parameter('log_dir', '')
        self.log_dir = self.get_parameter('log_dir').get_parameter_value().string_value

        self.planned_path = None
        self.actual_path = None
        self.planned_joints = []  # list of JointState samples
        self.actual_joints = []

        self.create_subscription(Path, '/planned_path', self._planned_cb, 10)
        self.create_subscription(Path, '/actual_path', self._actual_cb, 10)
        self.create_subscription(
            JointState, '/planned_joint_states', self._planned_joint_cb, 10)
        self.create_subscription(
            JointState, '/actual_joint_states', self._actual_joint_cb, 10)
        self.create_subscription(String, '/mission/save_log', self._save_cb, 10)

        self.get_logger().info(
            'PathLoggerNode ready — drone paths + arm joints')

    # ── Callbacks ──

    def _planned_cb(self, msg: Path):
        self.planned_path = msg

    def _actual_cb(self, msg: Path):
        self.actual_path = msg

    def _planned_joint_cb(self, msg: JointState):
        self.planned_joints.append(msg)

    def _actual_joint_cb(self, msg: JointState):
        self.actual_joints.append(msg)

    def _save_cb(self, msg: String):
        mission_name = msg.data.strip() or 'mission'
        self.save_to_csv(mission_name)

    # ── CSV writers ──

    def save_to_csv(self, mission_name: str):
        has_drone = self.planned_path is not None and self.actual_path is not None
        has_joints = bool(self.planned_joints) or bool(self.actual_joints)

        if not has_drone and not has_joints:
            self.get_logger().warn('No path/joint data available — nothing saved.')
            return

        ts = time.strftime('%Y%m%d_%H%M%S')

        if self.log_dir:
            log_parent = os.path.abspath(self.log_dir)
        else:
            workspace_root = os.path.abspath(
                os.path.join(os.path.dirname(__file__), '..', '..', '..'))
            log_parent = os.path.join(workspace_root, 'logs', 'trajectory_runs')

        log_dir = os.path.join(log_parent, f'{mission_name}_{ts}')
        os.makedirs(log_dir, exist_ok=True)

        if has_drone:
            planned_file = os.path.join(log_dir, 'planned_path.csv')
            self._write_path_csv(planned_file, self.planned_path)
            self.get_logger().info(
                f'Planned path saved: {planned_file} '
                f'({len(self.planned_path.poses)} points)')

            actual_file = os.path.join(log_dir, 'actual_path.csv')
            self._write_path_csv(actual_file, self.actual_path)
            self.get_logger().info(
                f'Actual path saved: {actual_file} '
                f'({len(self.actual_path.poses)} points)')
        else:
            self.get_logger().warn(
                'Drone path incomplete — skipped planned_path/actual_path CSV')

        if self.planned_joints:
            planned_j = os.path.join(log_dir, 'planned_joint_path.csv')
            self._write_joint_csv(planned_j, self.planned_joints)
            self.get_logger().info(
                f'Planned joints saved: {planned_j} '
                f'({len(self.planned_joints)} points)')

        if self.actual_joints:
            actual_j = os.path.join(log_dir, 'actual_joint_path.csv')
            self._write_joint_csv(actual_j, self.actual_joints)
            self.get_logger().info(
                f'Actual joints saved: {actual_j} '
                f'({len(self.actual_joints)} points)')

        if not self.planned_joints and not self.actual_joints:
            self.get_logger().warn('No joint samples — skipped joint CSV')

    @staticmethod
    def _write_path_csv(filepath, path_msg: Path):
        with open(filepath, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(
                ['time_sec', 'time_nsec', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
            for pose in path_msg.poses:
                writer.writerow([
                    pose.header.stamp.sec,
                    pose.header.stamp.nanosec,
                    pose.pose.position.x,
                    pose.pose.position.y,
                    pose.pose.position.z,
                    pose.pose.orientation.x,
                    pose.pose.orientation.y,
                    pose.pose.orientation.z,
                    pose.pose.orientation.w,
                ])

    @staticmethod
    def _write_joint_csv(filepath, samples):
        """Mirror path CSV: time + raw values (rad). Degrees are derived at plot time
        (same idea as qx,qy,qz,qw → roll/pitch/yaw)."""
        with open(filepath, 'w', newline='') as f:
            writer = csv.writer(f)
            # j1 = shoulder_joint, j2 = elbow_joint
            writer.writerow(['time_sec', 'time_nsec', 'j1', 'j2'])
            for msg in samples:
                name_to_pos = dict(zip(msg.name, msg.position))
                j1 = name_to_pos.get('shoulder_joint')
                j2 = name_to_pos.get('elbow_joint')
                if j1 is None or j2 is None:
                    if len(msg.position) < 2:
                        continue
                    j1, j2 = msg.position[0], msg.position[1]
                writer.writerow([
                    msg.header.stamp.sec,
                    msg.header.stamp.nanosec,
                    j1,
                    j2,
                ])


def main():
    rclpy.init()
    node = PathLoggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
