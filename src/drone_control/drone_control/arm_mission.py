import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool, CommandLong

import math
import time
from .trajectory import SCurveTrajectory, MultiJointTrajectory


class ArmMission(Node):
    """
    Mission: takeoff → control arm (shoulder + elbow) → land.
    Arm servo control via MAV_CMD_DO_SET_SERVO (FUNCTION=0).
    """

    # Mission states
    WAIT_CONN = 0
    SEND_GUIDED = 1
    WAIT_GUIDED = 2
    SEND_ARM = 3
    WAIT_ARM = 4
    WAIT_STABLE = 5
    SEND_TAKEOFF = 6
    WAIT_TAKEOFF = 7
    CLIMBING = 8
    HOVER_BEFORE_ARM = 9
    ARM_SEQUENCE = 10
    HOVER_AFTER_ARM = 11
    SEND_LAND = 12
    WAIT_LAND = 13
    WAIT_DISARM = 14
    DONE = 15

    # Servo channels (ArduPilot numbering)
    SHOULDER_SERVO = 9   # SERVO9 = AUX1
    ELBOW_SERVO = 10     # SERVO10 = AUX2

    # PWM limits
    PWM_MIN = 1100
    PWM_MAX = 1900
    PWM_CENTER = 1500

    # S-curve trajectory parameters for smooth motion
    MAX_ACCELERATION = 50.0  # PWM units/sec² (adjust based on servo specs)
    JERK_LIMIT = 100.0        # PWM units/sec³ (limit jerk for smooth motion)

    def __init__(self):
        super().__init__('arm_mission')

        # ── Parameters ──
        self.declare_parameter('takeoff_alt', 5.0)
        self.declare_parameter('hover_time', 3.0)
        self.declare_parameter('move_time_sec', 5.0)  # Time for each arm movement
        self.declare_parameter('hold_time_sec', 1.0)   # Time to hold position after movement
        self.declare_parameter('max_acceleration', self.MAX_ACCELERATION)
        self.declare_parameter('jerk_limit', self.JERK_LIMIT)

        self.takeoff_alt = self.get_parameter('takeoff_alt').value
        self.hover_time = self.get_parameter('hover_time').value
        self.move_time_sec = self.get_parameter('move_time_sec').value
        self.hold_time_sec = self.get_parameter('hold_time_sec').value
        self.max_acceleration = self.get_parameter('max_acceleration').value
        self.jerk_limit = self.get_parameter('jerk_limit').value

        self.get_logger().info(
            f'Arm mission config: move_time={self.move_time_sec}s, '
            f'hold_time={self.hold_time_sec}s, '
            f'accel={self.max_acceleration}, jerk={self.jerk_limit}'
        )

        # ── Arm sequence definition with S-curve trajectories ──
        # Each step: (shoulder_pwm, elbow_pwm, move_duration_sec, hold_duration_sec)
        # Move duration: time for smooth S-curve movement
        # Hold duration: time to maintain position after reaching target
        # PWM 1100=0°, 1500=90°, 1900=180° for shoulder
        # PWM 1100=-90°, 1500=0°, 1900=+90° for elbow
        self.arm_sequence = [
            # Step 1: shoulder 90°, elbow center
            (1500, 1500, self.move_time_sec, self.hold_time_sec),
            # Step 2: shoulder 90°, elbow +45°
            (1500, 1700, self.move_time_sec, self.hold_time_sec),
            # Step 3: shoulder 135°, elbow -45°
            (1700, 1300, self.move_time_sec, self.hold_time_sec),
            # Step 4: shoulder 45°, elbow center
            (1300, 1500, self.move_time_sec, self.hold_time_sec),
            # Step 5: return to home (shoulder 0°, elbow center)
            (1100, 1500, self.move_time_sec, self.hold_time_sec),
        ]
        self.arm_step = 0
        self.step_start_time = None
        self.current_trajectories = None  # Will hold SCurveTrajectory for shoulder and elbow
        self.step_move_phase = True  # True during movement, False during hold
        self.shoulder_traj = None
        self.elbow_traj = None

        # ── QoS ──
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── Publishers / Subscribers ──
        self.vel_pub = self.create_publisher(
            TwistStamped, '/mavros/setpoint_velocity/cmd_vel', qos)

        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self.local_pos_cb, qos)

        self.create_subscription(
            State, '/mavros/state',
            self.state_cb, qos)

        # ── Drone state ──
        self.current_z = 0.0
        self.current_state = None

        # ── Mission state ──
        self.mission_state = self.WAIT_CONN
        self.pending_future = None
        self.takeoff_z = 0.0
        self.target_z = 0.0
        self.arm_time = None
        self.hover_start = None
        self.ARM_STABLE_SEC = 2.0

        # ── MAVROS services ──
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.cmd_client = self.create_client(CommandLong, '/mavros/cmd/command')

        for client, name in [
            (self.set_mode_client, 'set_mode'),
            (self.arming_client, 'arming'),
            (self.cmd_client, 'command'),
        ]:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for {name} service...')

        self.timer = self.create_timer(0.05, self.timer_cb)

    # ── Callbacks ──

    def local_pos_cb(self, msg):
        self.current_z = msg.pose.position.z

    def state_cb(self, msg):
        self.current_state = msg

    # ── Helpers ──

    def send_servo(self, servo_num, pwm):
        """Send MAV_CMD_DO_SET_SERVO (183) to set servo PWM."""
        req = CommandLong.Request()
        req.command = 183
        req.param1 = float(servo_num)
        req.param2 = float(pwm)
        future = self.cmd_client.call_async(req)
        future.add_done_callback(self._servo_done_cb)

    def _servo_done_cb(self, future):
        resp = future.result()
        if not resp or not resp.success:
            self.get_logger().warn('Servo command failed')

    def set_stream_rate(self, msg_id, rate):
        req = CommandLong.Request()
        req.command = 511  # MAV_CMD_SET_MESSAGE_INTERVAL
        req.param1 = float(msg_id)
        req.param2 = float(1000000 // rate)
        self.cmd_client.call_async(req)

    # ── Main state machine ──

    def timer_cb(self):
        s = self.mission_state

        # --- WAIT_CONN ---
        if s == self.WAIT_CONN:
            if self.current_state and self.current_state.connected:
                self.get_logger().info('MAVROS connected')
                self.set_stream_rate(32, 20)
                self.mission_state = self.SEND_GUIDED
            return

        # --- SEND_GUIDED ---
        if s == self.SEND_GUIDED:
            req = SetMode.Request()
            req.custom_mode = 'GUIDED'
            self.pending_future = self.set_mode_client.call_async(req)
            self.get_logger().info('Switching to GUIDED...')
            self.mission_state = self.WAIT_GUIDED
            return

        # --- WAIT_GUIDED ---
        if s == self.WAIT_GUIDED:
            if not self.pending_future.done():
                return
            resp = self.pending_future.result()
            if resp and resp.mode_sent:
                self.get_logger().info('GUIDED mode set')
                self.mission_state = self.SEND_ARM
            else:
                self.get_logger().warn('GUIDED rejected, retrying...')
                self.mission_state = self.SEND_GUIDED
            return

        # --- SEND_ARM ---
        if s == self.SEND_ARM:
            req = CommandBool.Request()
            req.value = True
            self.pending_future = self.arming_client.call_async(req)
            self.get_logger().info('Arming...')
            self.mission_state = self.WAIT_ARM
            return

        # --- WAIT_ARM ---
        if s == self.WAIT_ARM:
            if not self.pending_future.done():
                return
            resp = self.pending_future.result()
            if resp and resp.success:
                self.get_logger().info('Arm accepted, waiting for confirmation...')
                self.arm_time = None
                self.mission_state = self.WAIT_STABLE
            else:
                self.get_logger().warn('Arm rejected, retrying...')
                self.mission_state = self.SEND_ARM
            return

        # --- WAIT_STABLE ---
        if s == self.WAIT_STABLE:
            if self.current_state and self.current_state.armed:
                if self.arm_time is None:
                    self.arm_time = self.get_clock().now()
                elapsed = (self.get_clock().now() - self.arm_time).nanoseconds / 1e9
                if elapsed >= self.ARM_STABLE_SEC:
                    self.get_logger().info('Stable, sending takeoff')
                    self.mission_state = self.SEND_TAKEOFF
            else:
                self.arm_time = None
            return

        # --- SEND_TAKEOFF ---
        if s == self.SEND_TAKEOFF:
            req = CommandLong.Request()
            req.command = 22  # MAV_CMD_NAV_TAKEOFF
            req.param7 = self.takeoff_alt
            req.param4 = float('nan')
            self.pending_future = self.cmd_client.call_async(req)
            self.get_logger().info(f'Takeoff to {self.takeoff_alt}m...')
            self.mission_state = self.WAIT_TAKEOFF
            return

        # --- WAIT_TAKEOFF ---
        if s == self.WAIT_TAKEOFF:
            if not self.pending_future.done():
                return
            resp = self.pending_future.result()
            if resp and resp.success:
                self.takeoff_z = self.current_z
                self.target_z = self.takeoff_z + self.takeoff_alt
                self.get_logger().info(f'Climbing to {self.target_z:.1f}m...')
                self.mission_state = self.CLIMBING
            else:
                self.get_logger().warn('Takeoff rejected, retrying...')
                self.mission_state = self.SEND_TAKEOFF
            return

        # --- CLIMBING ---
        if s == self.CLIMBING:
            if self.current_z >= self.target_z - 0.3:
                self.get_logger().info('Altitude reached, hovering before arm control...')
                self.hover_start = time.time()
                self.mission_state = self.HOVER_BEFORE_ARM
            return

        # --- HOVER_BEFORE_ARM: stabilize before moving arm ---
        if s == self.HOVER_BEFORE_ARM:
            if time.time() - self.hover_start >= self.hover_time:
                self.get_logger().info('Starting arm sequence...')
                self.arm_step = 0
                self.step_start_time = None
                self.mission_state = self.ARM_SEQUENCE
            return

        # --- ARM_SEQUENCE: execute arm steps with smooth S-curve trajectories ---
        if s == self.ARM_SEQUENCE:
            if self.arm_step >= len(self.arm_sequence):
                self.get_logger().info('Arm sequence complete, hovering...')
                self.hover_start = time.time()
                self.mission_state = self.HOVER_AFTER_ARM
                return

            shoulder_target, elbow_target, move_duration, hold_duration = self.arm_sequence[self.arm_step]

            if self.step_start_time is None:
                # Initialize trajectories for this step
                self.get_logger().info(
                    f'Step {self.arm_step + 1}/{len(self.arm_sequence)}: '
                    f'Moving to shoulder={shoulder_target}PWM, elbow={elbow_target}PWM '
                    f'over {move_duration}s + hold {hold_duration}s'
                )

                # Get current positions (or use previous targets as start)
                if self.arm_step == 0:
                    shoulder_current = self.PWM_MIN   # Home: 1100 (0°)
                    elbow_current = self.PWM_CENTER   # Home: 1500 (center)
                else:
                    shoulder_current, elbow_current, _, _ = self.arm_sequence[self.arm_step - 1]

                # Create S-curve trajectories for both joints
                self.shoulder_traj = SCurveTrajectory(
                    start_pos=shoulder_current,
                    end_pos=shoulder_target,
                    duration=move_duration,
                    max_acceleration=self.max_acceleration,
                    jerk_limit=self.jerk_limit,
                )
                self.elbow_traj = SCurveTrajectory(
                    start_pos=elbow_current,
                    end_pos=elbow_target,
                    duration=move_duration,
                    max_acceleration=self.max_acceleration,
                    jerk_limit=self.jerk_limit,
                )

                self.step_start_time = time.time()
                self.step_move_phase = True

            elapsed = time.time() - self.step_start_time

            if self.step_move_phase:
                # Movement phase: interpolate along S-curves
                if elapsed < self.shoulder_traj.duration:
                    shoulder_pwm = int(self.shoulder_traj.position(elapsed))
                    elbow_pwm = int(self.elbow_traj.position(elapsed))
                    
                    # Send current servo positions
                    self.send_servo(self.SHOULDER_SERVO, shoulder_pwm)
                    self.send_servo(self.ELBOW_SERVO, elbow_pwm)
                else:
                    # Movement complete, switch to hold phase
                    self.shoulder_pwm = shoulder_target
                    self.elbow_pwm = elbow_target
                    self.send_servo(self.SHOULDER_SERVO, shoulder_target)
                    self.send_servo(self.ELBOW_SERVO, elbow_target)
                    self.step_move_phase = False
                    self.step_start_time = time.time()  # Reset timer for hold phase
                    self.get_logger().info(
                        f'Movement complete. Holding position for {hold_duration}s'
                    )
            else:
                # Hold phase: maintain position
                if elapsed >= hold_duration:
                    # Hold phase complete, move to next step
                    self.arm_step += 1
                    self.step_start_time = None
                    self.shoulder_traj = None
                    self.elbow_traj = None
            return

        # --- HOVER_AFTER_ARM: wait before landing ---
        if s == self.HOVER_AFTER_ARM:
            if time.time() - self.hover_start >= self.hover_time:
                self.get_logger().info('Landing...')
                self.mission_state = self.SEND_LAND
            return

        # --- SEND_LAND ---
        if s == self.SEND_LAND:
            req = SetMode.Request()
            req.custom_mode = 'LAND'
            self.pending_future = self.set_mode_client.call_async(req)
            self.mission_state = self.WAIT_LAND
            return

        # --- WAIT_LAND ---
        if s == self.WAIT_LAND:
            if not self.pending_future.done():
                return
            resp = self.pending_future.result()
            if resp and resp.mode_sent:
                self.get_logger().info('LAND mode set, waiting for disarm...')
                self.mission_state = self.WAIT_DISARM
            else:
                self.get_logger().warn('LAND rejected, retrying...')
                self.mission_state = self.SEND_LAND
            return

        # --- WAIT_DISARM ---
        if s == self.WAIT_DISARM:
            if self.current_state and not self.current_state.armed:
                self.get_logger().info('Disarmed. Mission complete!')
                self.mission_state = self.DONE
            return


def main():
    rclpy.init()
    node = ArmMission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
