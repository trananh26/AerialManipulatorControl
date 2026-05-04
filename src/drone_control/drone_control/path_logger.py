"""
PathLoggerNode: subscribes to /planned_path and /actual_path, then saves
both to CSV when triggered via the /mission/save_log topic.

Mission nodes (arm_mission, circle_mission, …) publish a std_msgs/String
to /mission/save_log with the mission name as the message data
(e.g. "arm_mission" or "circle_mission").  This node uses that name as
the directory prefix so every run produces a uniquely-named folder.
"""

import csv
import os
import time

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from std_msgs.msg import String


class PathLoggerNode(Node):

    def __init__(self):
        super().__init__('path_logger')

        self.declare_parameter('log_dir', '')
        self.log_dir = self.get_parameter('log_dir').get_parameter_value().string_value

        self.planned_path = None
        self.actual_path = None

        self.create_subscription(Path, '/planned_path', self._planned_cb, 10)
        self.create_subscription(Path, '/actual_path', self._actual_cb, 10)
        self.create_subscription(String, '/mission/save_log', self._save_cb, 10)

        self.get_logger().info('PathLoggerNode ready — waiting for mission data...')

    # ── Callbacks ──

    def _planned_cb(self, msg: Path):
        self.planned_path = msg

    def _actual_cb(self, msg: Path):
        self.actual_path = msg

    def _save_cb(self, msg: String):
        mission_name = msg.data.strip() or 'mission'
        self.save_paths_to_csv(mission_name)

    # ── CSV writer ──

    def save_paths_to_csv(self, mission_name: str):
        if self.planned_path is None or self.actual_path is None:
            self.get_logger().warn('No path data available — nothing saved.')
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

        def _write(filepath, path_msg: Path):
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

        planned_file = os.path.join(log_dir, 'planned_path.csv')
        _write(planned_file, self.planned_path)
        self.get_logger().info(
            f'Planned path saved: {planned_file} ({len(self.planned_path.poses)} points)')

        actual_file = os.path.join(log_dir, 'actual_path.csv')
        _write(actual_file, self.actual_path)
        self.get_logger().info(
            f'Actual path saved: {actual_file} ({len(self.actual_path.poses)} points)')


def main():
    rclpy.init()
    node = PathLoggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
