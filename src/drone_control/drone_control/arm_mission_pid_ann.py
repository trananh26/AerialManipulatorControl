import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Path
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool, CommandLong
from std_msgs.msg import String

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
    INIT_ARM_POS = 3      # Send arm to init position before arming drone
    WAIT_INIT_ARM = 4     # Wait for arm to reach init position
    SEND_ARM = 5
    WAIT_ARM = 6
    WAIT_STABLE = 7
    SEND_TAKEOFF = 8
    WAIT_TAKEOFF = 9
    CLIMBING = 10
    HOVER_BEFORE_ARM = 11
    ARM_SEQUENCE = 12
    HOVER_AFTER_ARM = 13
    SEND_LAND = 14
    WAIT_LAND = 15
    WAIT_DISARM = 16
    DONE = 17

    # Servo channels (ArduPilot numbering)
    SHOULDER_SERVO = 9   # SERVO9 = AUX1
    ELBOW_SERVO = 10     # SERVO10 = AUX2

    # PWM limits
    PWM_MIN = 500
    PWM_MAX = 2500
    PWM_CENTER = 1500

    # S-curve trajectory parameters for smooth motion
    MAX_ACCELERATION = 50.0  # PWM units/sec² (adjust based on servo specs)
    JERK_LIMIT = 100.0        # PWM units/sec³ (limit jerk for smooth motion)

    def __init__(self):
        super().__init__('arm_mission')

        # ── Parameters ──
        self.declare_parameter('takeoff_alt', 3.0)
        self.declare_parameter('hover_time', 3.0)
        self.declare_parameter('move_time_sec', 6.0)  # Time for each arm movement
        self.declare_parameter('hold_time_sec', 2.0)   # Time to hold position after movement
        self.declare_parameter('max_acceleration', self.MAX_ACCELERATION)
        self.declare_parameter('jerk_limit', self.JERK_LIMIT)
        self.declare_parameter('save_log', True)
        self.declare_parameter('arm_only', False)  # If True: skip takeoff/land, only run arm sequence

        self.takeoff_alt = self.get_parameter('takeoff_alt').value
        self.hover_time = self.get_parameter('hover_time').value
        self.move_time_sec = self.get_parameter('move_time_sec').value
        self.hold_time_sec = self.get_parameter('hold_time_sec').value
        self.max_acceleration = self.get_parameter('max_acceleration').value
        self.jerk_limit = self.get_parameter('jerk_limit').value
        self.save_log = self.get_parameter('save_log').value
        self.arm_only = self.get_parameter('arm_only').value

        self.get_logger().info(
            f'Arm mission config: move_time={self.move_time_sec}s, '
            f'hold_time={self.hold_time_sec}s, '
            f'accel={self.max_acceleration}, jerk={self.jerk_limit}'
        )

        # ── Arm sequence definition with S-curve trajectories ──
        # Each step: (shoulder_pwm, elbow_pwm, move_duration_sec, hold_duration_sec)
        # Move duration: time for smooth S-curve movement
        # Hold duration: time to maintain position after reaching target
        # Sv9  (SHOULDER): PWM 500=0°(init), 1500=90°, 2500=180°
        # Sv10 (ELBOW):    PWM 500=0°,       1500=90°(init), 2500=180°
        self.arm_sequence = [
            (1500, 1500, self.move_time_sec, self.hold_time_sec),
            # Step 2: shoulder 90°, elbow 135°
            (1500, 2000, self.move_time_sec, self.hold_time_sec),
            (1500, 2400, self.move_time_sec, self.hold_time_sec),
            # Step 3: shoulder 135°, elbow 45°
            (2000, 1500, self.move_time_sec, self.hold_time_sec),
            (2400, 1500, self.move_time_sec, self.hold_time_sec),
            (2000, 1500, self.move_time_sec, self.hold_time_sec),
            (1500, 1000, self.move_time_sec, self.hold_time_sec),
            # Step 4: shoulder 45°, elbow 90°
            (1500, 1500, self.move_time_sec, self.hold_time_sec),
            # Step 5: shoulder 45°, elbow 90°
            (1000, 1500, self.move_time_sec, self.hold_time_sec),
            # Step 6: return to home (shoulder 0°, elbow 90°)
            (500, 1500, self.move_time_sec, self.hold_time_sec),
        ]
        self.arm_step = 0
        self.step_start_time = None
        self.current_trajectories = None  # Will hold SCurveTrajectory for shoulder and elbow
        self.step_move_phase = True  # True during movement, False during hold
        self.shoulder_traj = None
        self.elbow_traj = None

        # Init positions (current setup): Sv9=1500, Sv10=1500
        self.SHOULDER_INIT_PWM = self.PWM_CENTER
        self.ELBOW_INIT_PWM = self.PWM_CENTER
        self.INIT_ARM_DURATION = 3.0            # seconds to reach init position
        self.init_arm_start_time = None

        # ── QoS ──
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── Publishers / Subscribers ──
        self.vel_pub = self.create_publisher(
            TwistStamped, '/mavros/setpoint_velocity/cmd_vel', qos)
        self.planned_path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.actual_path_pub = self.create_publisher(Path, '/actual_path', 10)
        self.target_pose_pub = self.create_publisher(PoseStamped, '/mission/target_pose', 10)
        self.phase_pub = self.create_publisher(String, '/mission/phase', 10)
        self.save_log_pub = self.create_publisher(String, '/mission/save_log', 10)

        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self.local_pos_cb, qos)

        self.create_subscription(
            State, '/mavros/state',
            self.state_cb, qos)

        self.planned_path = Path()
        self.planned_path.header.frame_id = 'map'
        self.actual_path = Path()
        self.actual_path.header.frame_id = 'map'

        # ── Drone state ──
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_qx = 0.0
        self.current_qy = 0.0
        self.current_qz = 0.0
        self.current_qw = 1.0
        self.current_state = None
        self.takeoff_x = 0.0  # Fixed position for planned path waypoints
        self.takeoff_y = 0.0
        # Orientation captured at WAIT_STABLE (spawn orientation)
        self.init_qx = 0.0
        self.init_qy = 0.0
        self.init_qz = 0.0
        self.init_qw = 1.0

        # ── Mission state ──
        self.mission_state = self.WAIT_CONN
        self.pending_future = None
        self.takeoff_z = 0.0
        self.target_z = 0.0
        self.arm_time = None
        self.hover_start = None
        self.ARM_STABLE_SEC = 5.0
        self.arm_retry_count = 0
        self.max_arm_retries = 5

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
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.current_z = msg.pose.position.z
        self.current_qx = msg.pose.orientation.x
        self.current_qy = msg.pose.orientation.y
        self.current_qz = msg.pose.orientation.z
        self.current_qw = msg.pose.orientation.w
        if self.CLIMBING <= self.mission_state <= self.WAIT_DISARM:
            pose = PoseStamped()
            pose.header = msg.header
            pose.header.frame_id = 'map'
            pose.pose = msg.pose
            self.actual_path.poses.append(pose)
            self.actual_path.header.stamp = msg.header.stamp
            self.actual_path_pub.publish(self.actual_path)

    def state_cb(self, msg):
        self.current_state = msg

    # ── Helpers ──

    def add_planned(self, x, y, z, qx, qy, qz, qw):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.planned_path.poses.append(pose)
        self.planned_path.header.stamp = pose.header.stamp
        self.planned_path_pub.publish(self.planned_path)

    def send_servo(self, servo_num, pwm):
        """Send MAV_CMD_DO_SET_SERVO (183) to set servo PWM."""
        req = CommandLong.Request()
        req.command = 183
        req.param1 = float(servo_num)
        req.param2 = float(pwm)
        future = self.cmd_client.call_async(req)
        future.add_done_callback(self._servo_done_cb)

    def publish_mission_phase(self, phase):
        msg = String()
        msg.data = phase
        self.phase_pub.publish(msg)

    def publish_target_pose(self, x, y, z, qx, qy, qz, qw):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.target_pose_pub.publish(pose)

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
                self.mission_state = self.INIT_ARM_POS
            else:
                self.get_logger().warn('GUIDED rejected, retrying...')
                self.mission_state = self.SEND_GUIDED
            return

        # --- INIT_ARM_POS: move arm to init position before arming ---
        if s == self.INIT_ARM_POS:
            self.get_logger().info(
                f'Moving arm to init: shoulder={self.SHOULDER_INIT_PWM}PWM (0°), '
                f'elbow={self.ELBOW_INIT_PWM}PWM (90°)'
            )
            self.send_servo(self.SHOULDER_SERVO, self.SHOULDER_INIT_PWM)
            self.send_servo(self.ELBOW_SERVO, self.ELBOW_INIT_PWM)
            self.init_arm_start_time = time.time()
            self.mission_state = self.WAIT_INIT_ARM
            return

        # --- WAIT_INIT_ARM: wait for arm to reach init position ---
        if s == self.WAIT_INIT_ARM:
            elapsed = time.time() - self.init_arm_start_time
            # Keep sending servo commands during movement for reliability
            if elapsed < self.INIT_ARM_DURATION:
                self.send_servo(self.SHOULDER_SERVO, self.SHOULDER_INIT_PWM)
                self.send_servo(self.ELBOW_SERVO, self.ELBOW_INIT_PWM)
            else:
                if self.arm_only:
                    self.get_logger().info('Arm at init position. arm_only=True → skipping takeoff, starting arm sequence...')
                    self.arm_step = 0
                    self.step_start_time = None
                    self.mission_state = self.ARM_SEQUENCE
                else:
                    self.get_logger().info('Arm at init position. Arming drone...')
                    self.mission_state = self.SEND_ARM
            return

        # --- SEND_ARM ---
        if s == self.SEND_ARM:
            req = CommandBool.Request()
            req.value = True
            self.pending_future = self.arming_client.call_async(req)
            self.arm_retry_count += 1
            self.get_logger().info(f'Arming... (attempt {self.arm_retry_count}/{self.max_arm_retries})')
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
                self.arm_retry_count = 0
                self.mission_state = self.WAIT_STABLE
            else:
                if self.arm_retry_count >= self.max_arm_retries:
                    self.get_logger().error(
                        f'Arm rejected after {self.max_arm_retries} attempts. '
                        'Aborting mission. Ensure: drone is level, IMU calibrated, all pre-arm checks pass.'
                    )
                    if self.save_log:
                        self.save_log_pub.publish(String(data='arm_mission_failed'))
                    self.mission_state = self.DONE
                else:
                    self.get_logger().warn(
                        f'Arm rejected. Pre-arm check failed. Retry in 2s... (attempt {self.arm_retry_count}/{self.max_arm_retries})'
                    )
                    time.sleep(2.0)
                    self.mission_state = self.SEND_ARM
            return

        # --- WAIT_STABLE ---
        if s == self.WAIT_STABLE:
            if self.current_state and self.current_state.armed:
                if self.arm_time is None:
                    self.arm_time = self.get_clock().now()
                    self.init_qx = self.current_qx
                    self.init_qy = self.current_qy
                    self.init_qz = self.current_qz
                    self.init_qw = self.current_qw
                    self.add_planned(self.current_x, self.current_y, 0.2,
                                     self.init_qx, self.init_qy, self.init_qz, self.init_qw)
                    self.publish_target_pose(self.current_x, self.current_y, 0.2,
                                              self.init_qx, self.init_qy, self.init_qz, self.init_qw)
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
            self.add_planned(self.current_x, self.current_y, 0.2,
                             self.init_qx, self.init_qy, self.init_qz, self.init_qw)
            self.publish_target_pose(self.current_x, self.current_y, self.takeoff_alt,
                                      self.init_qx, self.init_qy, self.init_qz, self.init_qw)
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
                self.takeoff_x = self.current_x  # Record takeoff position for planned path
                self.takeoff_y = self.current_y
                self.get_logger().info(f'Climbing to {self.target_z:.1f}m...')
                self.add_planned(self.takeoff_x, self.takeoff_y, self.target_z,
                                 self.init_qx, self.init_qy, self.init_qz, self.init_qw)
                self.add_planned(self.takeoff_x, self.takeoff_y, self.target_z,
                                 self.init_qx, self.init_qy, self.init_qz, self.init_qw)
                self.publish_target_pose(self.takeoff_x, self.takeoff_y, self.target_z,
                                          self.init_qx, self.init_qy, self.init_qz, self.init_qw)
                self.mission_state = self.CLIMBING
            else:
                self.get_logger().warn('Takeoff rejected, retrying...')
                self.mission_state = self.SEND_TAKEOFF
            return

        # --- CLIMBING ---
        if s == self.CLIMBING:
            self.publish_target_pose(self.takeoff_x, self.takeoff_y, self.target_z,
                                      self.init_qx, self.init_qy, self.init_qz, self.init_qw)
            if self.current_z >= self.target_z - 0.3:
                self.get_logger().info('Altitude reached, hovering before arm control...')
                self.add_planned(self.takeoff_x, self.takeoff_y, self.target_z,
                                 self.init_qx, self.init_qy, self.init_qz, self.init_qw)
                self.hover_start = time.time()
                self.mission_state = self.HOVER_BEFORE_ARM
            return

        # --- HOVER_BEFORE_ARM: stabilize before moving arm ---
        if s == self.HOVER_BEFORE_ARM:
            self.publish_target_pose(self.takeoff_x, self.takeoff_y, self.target_z,
                                      self.init_qx, self.init_qy, self.init_qz, self.init_qw)
            if time.time() - self.hover_start >= self.hover_time:
                self.get_logger().info('Starting arm sequence...')
                self.arm_step = 0
                self.step_start_time = None
                self.mission_state = self.ARM_SEQUENCE
            return

        # --- ARM_SEQUENCE: execute arm steps with smooth S-curve trajectories ---
        if s == self.ARM_SEQUENCE:
            self.publish_target_pose(self.takeoff_x, self.takeoff_y, self.target_z,
                                      self.init_qx, self.init_qy, self.init_qz, self.init_qw)
            if self.arm_step >= len(self.arm_sequence):
                self.get_logger().info('Arm sequence complete.')
                if self.arm_only:
                    self.get_logger().info('arm_only=True → mission done.')
                    if self.save_log:
                        self.save_log_pub.publish(String(data='arm_mission'))
                    self.mission_state = self.DONE
                else:
                    self.get_logger().info('Hovering after arm...')
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
                    shoulder_current = self.SHOULDER_INIT_PWM
                    elbow_current = self.ELBOW_INIT_PWM
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
            self.publish_target_pose(self.takeoff_x, self.takeoff_y, self.target_z,
                                      self.init_qx, self.init_qy, self.init_qz, self.init_qw)
            if time.time() - self.hover_start >= self.hover_time:
                steps = int(self.target_z / 0.1)
                for z in [self.target_z - i * 0.1 for i in range(steps)]:
                    self.add_planned(self.takeoff_x, self.takeoff_y, z,
                                     self.init_qx, self.init_qy, self.init_qz, self.init_qw)
                self.get_logger().info('Landing...')
                self.mission_state = self.SEND_LAND
            return

        # --- SEND_LAND ---
        if s == self.SEND_LAND:
            self.publish_mission_phase('LAND')
            self.publish_target_pose(self.takeoff_x, self.takeoff_y, 0.0,
                                      self.init_qx, self.init_qy, self.init_qz, self.init_qw)
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
                if self.save_log:
                    self.save_log_pub.publish(String(data='arm_mission'))
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
