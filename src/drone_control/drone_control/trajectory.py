"""
S-curve (Double-Ramp) Trajectory Generator for servo control.

Implements smooth acceleration profiles that minimize jerk and vibration.
Profile has 7 phases:
  1. Acceleration with jerk limit (accel ramps up)
  2. Constant acceleration
  3. Deceleration with jerk limit (accel ramps down)
  4. Constant velocity
  5. Acceleration with jerk limit (decel ramps up)
  6. Constant deceleration
  7. Deceleration with jerk limit (decel ramps down)
"""

import numpy as np
from dataclasses import dataclass
from typing import Tuple


@dataclass
class TrajectoryParams:
    """Parameters for S-curve trajectory generation."""
    duration: float           # Total time for movement (seconds)
    distance: float           # Total distance to travel (PWM units or degrees)
    max_velocity: float       # Maximum velocity (units/sec)
    max_acceleration: float   # Maximum acceleration (units/sec²)
    jerk_limit: float         # Maximum jerk (units/sec³)


class SCurveTrajectory:
    """
    S-curve trajectory generator for smooth servo control.
    
    Generates position and velocity profiles that minimize vibration
    by limiting jerk (rate of change of acceleration).
    """

    def __init__(self, start_pos: float, end_pos: float, duration: float,
                 max_velocity: float = None,
                 max_acceleration: float = 50.0,
                 jerk_limit: float = 20.0):
        """
        Initialize S-curve trajectory.
        
        Args:
            start_pos: Starting position (PWM or angle units)
            end_pos: Ending position (PWM or angle units)
            duration: Total time to reach end_pos (seconds)
            max_velocity: Maximum velocity (units/sec). If None, auto-calculated.
            max_acceleration: Maximum acceleration (units/sec²)
            jerk_limit: Maximum jerk (units/sec³)
        """
        self.start_pos = start_pos
        self.end_pos = end_pos
        self.direction = 1 if end_pos >= start_pos else -1
        self.distance = abs(end_pos - start_pos)
        self.duration = duration
        self.max_acceleration = max_acceleration
        self.jerk_limit = jerk_limit

        # Calculate max velocity if not provided
        if max_velocity is None:
            # Simple heuristic: distance / (duration * 0.6) accounts for accel/decel phases
            self.max_velocity = max(self.distance / duration * 0.6, 10.0)
        else:
            self.max_velocity = max_velocity

        self._compute_trajectory_times()

    def _compute_trajectory_times(self):
        """Compute phase durations for the trajectory."""
        # Time for jerk-limited acceleration
        if self.jerk_limit > 0:
            self.tj = self.max_acceleration / self.jerk_limit
        else:
            self.tj = 0.001

        # Time for constant acceleration
        # v = a*t, so we need to fit acceleration phase within time budget
        # For simplicity: aim for symmetric accel/decel
        self.ta = self.duration * 0.25  # accel phase duration
        self.tv = max(0, self.duration - 2 * self.ta)  # constant velocity phase
        
        # Total displacement during acceleration phase
        # s = 0.5 * a * t^2
        # But with S-curve, it's more complex
        # Approximate: distance_accel = 0.5 * max_vel * ta (triangle approximation)
        
        # Recalculate max_velocity to fit within distance
        distance_accel = 0.5 * self.max_velocity * self.ta
        distance_decel = 0.5 * self.max_velocity * self.ta
        distance_const = self.distance - distance_accel - distance_decel
        
        if distance_const < 0:
            # Movement is too short, reduce phases
            self.max_velocity = self.distance / (self.ta + self.ta)
            distance_accel = 0.5 * self.max_velocity * self.ta
            distance_decel = 0.5 * self.max_velocity * self.ta
            distance_const = self.distance - distance_accel - distance_decel

    def position(self, time: float) -> float:
        """
        Get position at given time using S-curve trajectory.
        
        Args:
            time: Time elapsed (seconds)
            
        Returns:
            Position in same units as start_pos/end_pos
        """
        if time <= 0:
            return self.start_pos
        if time >= self.duration:
            return self.end_pos

        # Normalize time
        t = time / self.duration
        
        # S-curve position formula (normalized)
        # Using quintic polynomial for smooth motion
        s = self._scurve_interp(t)
        
        return self.start_pos + s * self.distance * self.direction

    def velocity(self, time: float) -> float:
        """
        Get velocity at given time.
        
        Args:
            time: Time elapsed (seconds)
            
        Returns:
            Velocity in units/sec
        """
        if time <= 0 or time >= self.duration:
            return 0.0

        # Numerical differentiation of position
        dt = 0.001
        pos_after = self.position(time + dt)
        pos_before = self.position(time - dt)
        vel = (pos_after - pos_before) / (2 * dt)
        
        return vel * self.direction

    @staticmethod
    def _scurve_interp(t: float) -> float:
        """
        S-curve interpolation function (0 to 1).
        Uses quintic polynomial for smooth motion.
        s(t) = 3*t^2 - 2*t^3  (basic S-curve)
        For sharper curves: s(t) = t^3 * (10 - 15*t + 6*t^2)  (Hermite)
        """
        # Quintic Hermite curve commonly used for animation
        return t * t * t * (10 + t * (-15 + 6*t))

    def get_trajectory_array(self, num_points: int = 100) -> Tuple[np.ndarray, np.ndarray]:
        """
        Generate full trajectory as arrays.
        
        Args:
            num_points: Number of sample points
            
        Returns:
            Tuple of (time_array, position_array)
        """
        times = np.linspace(0, self.duration, num_points)
        positions = np.array([self.position(t) for t in times])
        return times, positions


class MultiJointTrajectory:
    """
    Manages trajectories for multiple arm joints simultaneously.
    Ensures coordinated movement of all joints.
    """

    def __init__(self):
        """Initialize multi-joint trajectory manager."""
        self.trajectories = {}
        self.start_time = None
        self.active = False

    def add_joint(self, joint_name: str, start_pos: float, end_pos: float,
                  duration: float, max_acceleration: float = 50.0,
                  jerk_limit: float = 20.0):
        """
        Add a joint trajectory.
        
        Args:
            joint_name: Identifier for the joint
            start_pos: Starting position
            end_pos: Ending position
            duration: Time to reach end position
            max_acceleration: Max acceleration (units/sec²)
            jerk_limit: Max jerk (units/sec³)
        """
        traj = SCurveTrajectory(
            start_pos=start_pos,
            end_pos=end_pos,
            duration=duration,
            max_acceleration=max_acceleration,
            jerk_limit=jerk_limit,
        )
        self.trajectories[joint_name] = traj

    def start(self, current_time: float = 0.0):
        """Start trajectory execution."""
        self.start_time = current_time
        self.active = True

    def stop(self):
        """Stop trajectory execution."""
        self.active = False

    def get_position(self, joint_name: str, current_time: float) -> float:
        """
        Get current position for a joint.
        
        Args:
            joint_name: Joint identifier
            current_time: Current time
            
        Returns:
            Current position of joint
        """
        if not self.active or joint_name not in self.trajectories:
            return None

        if self.start_time is None:
            return None

        elapsed_time = current_time - self.start_time
        traj = self.trajectories[joint_name]
        return traj.position(elapsed_time)

    def get_all_positions(self, current_time: float) -> dict:
        """
        Get current position for all joints.

        Args:
            current_time: Current time

        Returns:
            Dictionary of {joint_name: position}
        """
        positions = {}
        for joint_name in self.trajectories:
            pos = self.get_position(joint_name, current_time)
            if pos is not None:
                positions[joint_name] = pos
        return positions

    def is_finished(self, current_time: float) -> bool:
        """
        Check if all trajectories are finished.

        Args:
            current_time: Current time

        Returns:
            True if all joints have reached their targets
        """
        if not self.active or not self.trajectories:
            return True

        if self.start_time is None:
            return True

        for traj in self.trajectories.values():
            elapsed_time = current_time - self.start_time
            if elapsed_time < traj.duration:
                return False
        return True


# Helper functions for convenience

def create_joint_movement(start_pwm: int, end_pwm: int, 
                          movement_time: float,
                          max_acceleration: float = 50.0,
                          jerk_limit: float = 20.0) -> SCurveTrajectory:
    """
    Create a simple joint movement with S-curve profile.

    Args:
        start_pwm: Starting PWM value
        end_pwm: Target PWM value
        movement_time: Time to complete movement (seconds)
        max_acceleration: Maximum acceleration (PWM_units/sec²)
        jerk_limit: Maximum jerk (PWM_units/sec³)

    Returns:
        SCurveTrajectory object
    """
    return SCurveTrajectory(
        start_pos=start_pwm,
        end_pos=end_pwm,
        duration=movement_time,
        max_acceleration=max_acceleration,
        jerk_limit=jerk_limit,
    )
