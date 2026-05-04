import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu
from mavros_msgs.msg import State
from mavros_msgs.srv import ParamSet
from std_msgs.msg import String
import numpy as np
import math
import time
from collections import deque


class PIDTuner(Node):
    """
    Node to tune PID parameters using simple optimization.
    Uses gradient descent to adjust P, I, D gains for position and attitude control based on error metrics.
    """

    def __init__(self):
        super().__init__('pid_tuner')

        # MAVROS QoS profile - matches MAVROS publishers
        mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers with MAVROS-compatible QoS
        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.pose_callback, mavros_qos)
        self.velocity_sub = self.create_subscription(
            TwistStamped, '/mavros/local_position/velocity_local', self.velocity_callback, mavros_qos)
        self.imu_sub = self.create_subscription(
            Imu, '/mavros/imu/data', self.imu_callback, mavros_qos)
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_callback, mavros_qos)
        self.target_pose_sub = self.create_subscription(
            PoseStamped, '/mission/target_pose', self.target_pose_callback, mavros_qos)
        self.phase_sub = self.create_subscription(
            String, '/mission/phase', self.phase_callback, mavros_qos)

        # Service client for setting parameters
        self.param_set_client = self.create_client(ParamSet, '/mavros/param/set')

        # Data storage
        self.errors = []
        self.error_history = deque(maxlen=10)
        self.max_samples = 100
        self.target_pose_active = False
        self.in_landing_phase = False

        # PID parameters to tune (position and attitude control)
        self.pid_params = {
            # Position
            'POS_X_P': 0.5, 'POS_X_I': 0.01, 'POS_X_D': 0.0,
            'POS_Y_P': 0.35, 'POS_Y_I': 0.005, 'POS_Y_D': 0.02,
            'POS_Z_P': 0.5, 'POS_Z_I': 0.01, 'POS_Z_D': 0.0,
            # Attitude
            'ROLL_P': 0.5, 'ROLL_I': 0.01, 'ROLL_D': 0.0,
            'PITCH_P': 0.5, 'PITCH_I': 0.01, 'PITCH_D': 0.0,
            'YAW_P': 0.5, 'YAW_I': 0.01, 'YAW_D': 0.0
        }

        # Tuning configuration
        self.learning_rate = 0.003  # Giảm learning rate để ổn định hơn
        self.min_error_to_tune = 0.15
        self.min_tune_interval = 3.0  # Tăng interval để tuning chậm hơn
        self.last_tune_time = time.time()
        self.min_velocity_for_tuning = 0.6

        # Current state
        self.current_pose = None
        self.current_velocity = None
        self.current_attitude = None
        self.target_pose = np.array([0.0, 0.0, 1.0])  # Example target position
        self.target_attitude = np.array([0.0, 0.0, 0.0])  # Target attitude (roll, pitch, yaw)

        # Timer for periodic tuning
        self.timer = self.create_timer(1.0, self.tune_pid)  # Tune every second

        self.get_logger().info('PID Tuner initialized (no TensorFlow required)')

    def pose_callback(self, msg):
        self.current_pose = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])

    def velocity_callback(self, msg):
        self.current_velocity = np.array([
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z
        ])

    def imu_callback(self, msg):
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        q = msg.orientation
        self.current_attitude = self.quaternion_to_euler(q.x, q.y, q.z, q.w)

    def target_pose_callback(self, msg):
        self.target_pose = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ])
        self.target_pose_active = True

    def phase_callback(self, msg):
        if msg.data == 'LAND' or msg.data == 'WAIT_LAND' or msg.data == 'DISARM':
            self.in_landing_phase = True
        else:
            self.in_landing_phase = False

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles (roll, pitch, yaw)."""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return np.array([roll, pitch, yaw])

    def state_callback(self, msg):
        pass  # Could use for armed state, etc.

    def calculate_error(self):
        """Calculate position and attitude errors."""
        pos_error = np.zeros(3)
        att_error = np.zeros(3)

        if self.current_pose is not None:
            if self.target_pose_active:
                pos_error = self.target_pose - self.current_pose
            else:
                pos_error = self.target_pose - self.current_pose

        if self.current_attitude is not None:
            att_error = self.wrap_angle(self.target_attitude - self.current_attitude)

        return pos_error, att_error

    def wrap_angle(self, angle):
        """Wrap angle differences to [-pi, pi]."""
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def has_oscillation(self):
        if len(self.error_history) < 4:
            return False

        # Count sign changes for attitude errors over recent samples
        signs = np.sign(np.array(self.error_history)[:, 3:])
        sign_changes = np.abs(np.diff(signs, axis=0)).sum()
        return sign_changes >= 4

    def axis_oscillation(self, axis_index):
        if len(self.error_history) < 4:
            return False

        signs = np.sign(np.array(self.error_history)[:, axis_index])
        sign_changes = np.abs(np.diff(signs)).sum()
        return sign_changes >= 3

    def axis_trend(self, axis_index):
        if len(self.errors) < 2:
            return 0.0
        return self.errors[-1][axis_index] - self.errors[-2][axis_index]

    def steady_error(self, axis_index, window=5):
        if len(self.errors) == 0:
            return 0.0
        values = [e[axis_index] for e in list(self.errors)[-window:]]
        return np.mean(values)

    def tune_pid(self):
        """Adjust PID parameters conservatively to reduce oscillations."""
        if self.current_pose is None or self.current_velocity is None or self.current_attitude is None:
            return

        if time.time() - self.last_tune_time < self.min_tune_interval:
            return

        if self.in_landing_phase:
            return

        pos_error, att_error = self.calculate_error()
        current_error = np.concatenate([pos_error, att_error])
        self.errors.append(current_error)
        self.error_history.append(current_error)

        if len(self.errors) < 3:
            return

        # Skip tuning while vehicle is actively moving rapidly; wait for disturbance to settle
        if np.linalg.norm(self.current_velocity) > self.min_velocity_for_tuning:
            return

        pos_error_magnitude = np.linalg.norm(pos_error)
        att_error_magnitude = np.linalg.norm(att_error)

        previous_params = self.pid_params.copy()
        y_axis_osc = self.axis_oscillation(1)
        y_trend = self.axis_trend(1)
        y_steady = self.steady_error(1)

        # If Y-axis oscillation is detected, damp Y control strongly.
        if y_axis_osc and abs(pos_error[1]) > self.min_error_to_tune:
            self.adjust_gain('POS_Y_P', -0.02)
            self.adjust_gain('POS_Y_D', 0.04)
            self.adjust_gain('POS_Y_I', -0.005)
            self.get_logger().info('Y-axis oscillation detected, strongly damping Y gains')

        # Fine tune POS Y based only on Y behavior.
        if abs(pos_error[1]) > self.min_error_to_tune:
            if y_trend * pos_error[1] > 0:
                # Error magnitude is increasing; reduce P and increase D.
                self.adjust_gain('POS_Y_P', -0.01)
                self.adjust_gain('POS_Y_D', 0.02)
                self.adjust_gain('POS_Y_I', -0.002)
            elif y_trend * pos_error[1] < 0:
                # Error magnitude is decreasing; increase P if not oscillating.
                if not y_axis_osc:
                    self.adjust_gain('POS_Y_P', 0.005)
                    self.adjust_gain('POS_Y_D', -0.003)

            if abs(y_steady) > 0.12 and not y_axis_osc:
                self.adjust_gain('POS_Y_I', 0.002)
            elif abs(y_steady) < 0.05:
                self.adjust_gain('POS_Y_I', -0.001)

        elif abs(pos_error[1]) < 0.05:
            # If Y is already stable, reduce gains to avoid overshoot.
            self.adjust_gain('POS_Y_P', -0.005)
            self.adjust_gain('POS_Y_D', -0.002)
            self.adjust_gain('POS_Y_I', -0.001)

        # Global oscillation handling for other axes and attitude.
        if self.has_oscillation() and (pos_error_magnitude > self.min_error_to_tune or att_error_magnitude > self.min_error_to_tune):
            self.adjust_gain('POS_X_P', -0.01)
            self.adjust_gain('POS_Z_P', -0.01)
            self.adjust_gain('ROLL_D', 0.01)
            self.adjust_gain('PITCH_D', 0.01)
            self.adjust_gain('YAW_D', 0.01)

        if not y_axis_osc and abs(pos_error[0]) > self.min_error_to_tune:
            x_trend = self.axis_trend(0)
            if x_trend * pos_error[0] > 0:
                self.adjust_gain('POS_X_P', -0.004)
            elif x_trend * pos_error[0] < 0:
                self.adjust_gain('POS_X_P', 0.002)

        if not y_axis_osc and abs(pos_error[2]) > self.min_error_to_tune:
            z_trend = self.axis_trend(2)
            if z_trend * pos_error[2] > 0:
                self.adjust_gain('POS_Z_P', -0.004)
            elif z_trend * pos_error[2] < 0:
                self.adjust_gain('POS_Z_P', 0.002)

        changed = {k: (previous_params[k], self.pid_params[k])
                   for k in self.pid_params
                   if not np.isclose(previous_params[k], self.pid_params[k])}

        # Keep only recent errors
        if len(self.errors) > self.max_samples:
            self.errors.pop(0)

        self.last_tune_time = time.time()
        self.apply_pid_params()

        if changed:
            change_strings = [f'{k}: {old:.3f}->{new:.3f}' for k, (old, new) in changed.items()]
            self.get_logger().info('PID changed: ' + ', '.join(change_strings))

        # Log system status every 20 cycles
        if len(self.errors) % 20 == 0:
            if self.current_pose is not None and self.current_velocity is not None:
                pos_error = np.linalg.norm(self.target_pose - self.current_pose)
                vel_magnitude = np.linalg.norm(self.current_velocity)
                self.get_logger().info(
                    f'Status: pos_error={pos_error:.2f}m, velocity={vel_magnitude:.2f}m/s, '
                    f'oscillation={self.has_oscillation()}, Y_osc={self.axis_oscillation(1)} | '
                    f'PID: POS_P={self.pid_params["POS_Y_P"]:.3f}, POS_I={self.pid_params["POS_Y_I"]:.3f}, POS_D={self.pid_params["POS_Y_D"]:.3f}')

    def adjust_gain(self, key, delta):
        self.pid_params[key] = np.clip(self.pid_params[key] + delta, 0.0, 2.0)

    def apply_pid_params(self):
        """Set PID parameters in ArduPilot via MAVROS."""
        if not self.param_set_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning('ParamSet service not available yet')
            return

        for param_name, value in self.pid_params.items():
            req = ParamSet.Request()
            req.param_id = param_name
            req.value.real = float(value)
            self.param_set_client.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    tuner = PIDTuner()
    rclpy.spin(tuner)
    tuner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()