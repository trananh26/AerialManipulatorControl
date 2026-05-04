import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu
from mavros_msgs.msg import State
from mavros_msgs.srv import ParamSet
import numpy as np
import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense
import math
import time


class PIDANNTuner(Node):
    """
    Node to tune PID parameters using Artificial Neural Network.
    Uses reinforcement learning approach to adjust P, I, D gains for both position and attitude control based on error metrics.
    """

    def __init__(self):
        super().__init__('pid_ann_tuner')

        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.pose_callback, 10)
        self.velocity_sub = self.create_subscription(
            TwistStamped, '/mavros/local_position/velocity_local', self.velocity_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/mavros/imu/data', self.imu_callback, 10)
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_callback, 10)

        # Service client for setting parameters
        self.param_set_client = self.create_client(ParamSet, '/mavros/param/set')

        # ANN Model
        self.model = self.build_model()

        # Data storage
        self.errors = []
        self.adjustments = []
        self.max_samples = 1000

        # PID parameters to tune (position and attitude control)
        self.pid_params = {
            # Position
            'POS_X_P': 0.5, 'POS_X_I': 0.0, 'POS_X_D': 0.0,
            'POS_Y_P': 0.5, 'POS_Y_I': 0.0, 'POS_Y_D': 0.0,
            'POS_Z_P': 0.5, 'POS_Z_I': 0.0, 'POS_Z_D': 0.0,
            # Attitude
            'ROLL_P': 0.5, 'ROLL_I': 0.0, 'ROLL_D': 0.0,
            'PITCH_P': 0.5, 'PITCH_I': 0.0, 'PITCH_D': 0.0,
            'YAW_P': 0.5, 'YAW_I': 0.0, 'YAW_D': 0.0
        }

        # Current state
        self.current_pose = None
        self.current_velocity = None
        self.current_attitude = None
        self.target_pose = np.array([0.0, 0.0, 1.0])  # Example target position
        self.target_attitude = np.array([0.0, 0.0, 0.0])  # Target attitude (roll, pitch, yaw)

        # Timer for periodic tuning
        self.timer = self.create_timer(1.0, self.tune_pid)  # Tune every second

    def build_model(self):
        """Build ANN for PID tuning (position and attitude)."""
        model = Sequential([
            Dense(128, activation='relu', input_shape=(9,)),  # Input: pos_error x,y,z, vel x,y,z, att_error r,p,y
            Dense(64, activation='relu'),
            Dense(6, activation='linear')  # Output: delta_P_pos, delta_I_pos, delta_D_pos, delta_P_att, delta_I_att, delta_D_att
        ])
        model.compile(optimizer='adam', loss='mse')
        return model

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
            pos_error = self.target_pose - self.current_pose
        
        if self.current_attitude is not None:
            att_error = self.target_attitude - self.current_attitude
        
        return pos_error, att_error

    def tune_pid(self):
        """Use ANN to predict PID adjustments for position and attitude."""
        if self.current_pose is None or self.current_velocity is None or self.current_attitude is None:
            return

        pos_error, att_error = self.calculate_error()

        # Prepare input for ANN: pos_error x,y,z, vel x,y,z, att_error r,p,y
        input_data = np.concatenate([
            pos_error,
            -self.current_velocity,  # Approximation for position error_dot
            att_error
        ])

        # Predict adjustments: [delta_P_pos, delta_I_pos, delta_D_pos, delta_P_att, delta_I_att, delta_D_att]
        adjustments = self.model.predict(input_data.reshape(1, -1))[0]

        # Update position PID parameters
        self.pid_params['POS_X_P'] += adjustments[0] * 0.01
        self.pid_params['POS_X_I'] += adjustments[1] * 0.001
        self.pid_params['POS_X_D'] += adjustments[2] * 0.01
        self.pid_params['POS_Y_P'] += adjustments[0] * 0.01
        self.pid_params['POS_Y_I'] += adjustments[1] * 0.001
        self.pid_params['POS_Y_D'] += adjustments[2] * 0.01
        self.pid_params['POS_Z_P'] += adjustments[0] * 0.01
        self.pid_params['POS_Z_I'] += adjustments[1] * 0.001
        self.pid_params['POS_Z_D'] += adjustments[2] * 0.01

        # Update attitude PID parameters
        self.pid_params['ROLL_P'] += adjustments[3] * 0.01
        self.pid_params['ROLL_I'] += adjustments[4] * 0.001
        self.pid_params['ROLL_D'] += adjustments[5] * 0.01
        self.pid_params['PITCH_P'] += adjustments[3] * 0.01
        self.pid_params['PITCH_I'] += adjustments[4] * 0.001
        self.pid_params['PITCH_D'] += adjustments[5] * 0.01
        self.pid_params['YAW_P'] += adjustments[3] * 0.01
        self.pid_params['YAW_I'] += adjustments[4] * 0.001
        self.pid_params['YAW_D'] += adjustments[5] * 0.01

        # Clamp values
        for key in self.pid_params:
            self.pid_params[key] = np.clip(self.pid_params[key], 0.0, 2.0)

        # Set parameters in ArduPilot
        self.set_pid_params()

        # Store data for training
        self.errors.append((pos_error, att_error))
        self.adjustments.append(adjustments)

        if len(self.errors) > self.max_samples:
            self.errors.pop(0)
            self.adjustments.pop(0)

        # Periodic retraining
        if len(self.errors) % 100 == 0 and len(self.errors) > 10:
            self.retrain_model()

    def set_pid_params(self):
        """Set PID parameters via MAVROS."""
        for param_name, value in self.pid_params.items():
            req = ParamSet.Request()
            req.param_id = param_name
            req.value.real = float(value)
            self.param_set_client.call_async(req)

    def retrain_model(self):
        """Retrain ANN with collected data."""
        X = []
        y = []

        for i in range(1, len(self.errors)):
            prev_pos_error, prev_att_error = self.errors[i-1]
            curr_pos_error, curr_att_error = self.errors[i]
            
            # Reward based on error reduction
            pos_reward = np.linalg.norm(prev_pos_error) - np.linalg.norm(curr_pos_error)
            att_reward = np.linalg.norm(prev_att_error) - np.linalg.norm(curr_att_error)
            reward = (pos_reward + att_reward) * 0.1

            # Input: pos_error, vel, att_error
            input_vec = np.concatenate([
                prev_pos_error,
                -self.current_velocity if self.current_velocity is not None else [0, 0, 0],
                prev_att_error
            ])
            X.append(input_vec)
            y.append(self.adjustments[i] + reward)

        if X:
            X = np.array(X)
            y = np.array(y)
            self.model.fit(X, y, epochs=5, verbose=0)


def main(args=None):
    rclpy.init(args=args)
    tuner = PIDANNTuner()
    rclpy.spin(tuner)
    tuner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()