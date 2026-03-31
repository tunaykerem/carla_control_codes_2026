"""
pid_controller.py
-----------------
Reusable PID controller classes for longitudinal (speed) and lateral
(steering) control of a CARLA vehicle.

Classes
-------
PIDController      – generic single-axis PID with anti-windup
LongitudinalPID    – throttle / brake output from target speed
LateralPID         – steering output from cross-track / heading error
"""

from __future__ import annotations

import math
import time
from collections import deque
from typing import Optional


# ---------------------------------------------------------------------------
# Generic PID
# ---------------------------------------------------------------------------
class PIDController:
    """
    Discrete PID controller.

    Parameters
    ----------
    kp, ki, kd : float
        Proportional, integral, derivative gains.
    max_integral : float
        Anti-windup clamp on the integral term (±).
    output_limits : tuple[float, float] | None
        Optional (min, max) clamp on the final output.
    """

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        max_integral: float = 100.0,
        output_limits: Optional[tuple[float, float]] = None,
    ) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_integral = max_integral
        self.output_limits = output_limits

        self._integral: float = 0.0
        self._prev_error: float = 0.0
        self._prev_time: Optional[float] = None

    def reset(self) -> None:
        """Reset internal state (useful when switching targets)."""
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None

    def step(self, error: float, timestamp: Optional[float] = None) -> float:
        """
        Compute PID output for the given *error* at *timestamp*.

        Parameters
        ----------
        error : float
            Setpoint minus measured value.
        timestamp : float | None
            Current time in seconds.  If None, ``time.time()`` is used.

        Returns
        -------
        float
            Control output (optionally clamped to *output_limits*).
        """
        now = time.time() if timestamp is None else timestamp
        dt = (now - self._prev_time) if self._prev_time is not None else 1e-4
        dt = max(dt, 1e-6)  # guard against zero dt

        self._integral += error * dt
        self._integral = max(
            -self.max_integral, min(self.max_integral, self._integral)
        )

        derivative = (error - self._prev_error) / dt

        output = self.kp * error + self.ki * self._integral + self.kd * derivative

        if self.output_limits is not None:
            lo, hi = self.output_limits
            output = max(lo, min(hi, output))

        self._prev_error = error
        self._prev_time = now
        return output


# ---------------------------------------------------------------------------
# Longitudinal controller (speed → throttle / brake)
# ---------------------------------------------------------------------------
class LongitudinalPID:
    """
    Converts a speed error (target_speed − current_speed) into a
    throttle (0..1) or brake (0..1) command.

    Parameters
    ----------
    kp, ki, kd : float
        PID gains for the speed loop.
    """

    def __init__(
        self,
        kp: float = 0.5,
        ki: float = 0.05,
        kd: float = 0.1,
    ) -> None:
        self._pid = PIDController(kp, ki, kd, output_limits=(-1.0, 1.0))

    def reset(self) -> None:
        self._pid.reset()

    def run_step(
        self,
        target_speed: float,
        current_speed: float,
        timestamp: Optional[float] = None,
    ) -> tuple[float, float]:
        """
        Compute (throttle, brake) for the given speeds.

        Parameters
        ----------
        target_speed : float
            Desired speed in **km/h**.
        current_speed : float
            Current vehicle speed in **km/h**.
        timestamp : float | None

        Returns
        -------
        (throttle, brake) : tuple[float, float]
            Both values are in [0, 1].
        """
        error = target_speed - current_speed
        output = self._pid.step(error, timestamp)
        if output >= 0.0:
            return (output, 0.0)
        return (0.0, -output)


# ---------------------------------------------------------------------------
# Lateral controller (waypoint → steer)
# ---------------------------------------------------------------------------
class LateralPID:
    """
    Stanley-inspired lateral controller that computes a steering angle
    from the heading error and cross-track error to the next waypoint.

    The controller uses a simple PID on the total lateral error rather
    than a full Stanley formulation so it works without vehicle speed.

    Parameters
    ----------
    kp, ki, kd : float
        PID gains.
    """

    def __init__(
        self,
        kp: float = 0.9,
        ki: float = 0.0,
        kd: float = 0.1,
    ) -> None:
        self._pid = PIDController(kp, ki, kd, output_limits=(-1.0, 1.0))
        self._e_buffer: deque[float] = deque(maxlen=10)

    def reset(self) -> None:
        self._pid.reset()
        self._e_buffer.clear()

    def run_step(
        self,
        waypoint_location,
        vehicle_transform,
        timestamp: Optional[float] = None,
    ) -> float:
        """
        Compute steering output in [-1, 1].

        Parameters
        ----------
        waypoint_location : carla.Location
            Target waypoint location.
        vehicle_transform : carla.Transform
            Current vehicle transform (location + rotation).
        timestamp : float | None

        Returns
        -------
        float
            Steering value in [-1.0, 1.0].
        """
        v_loc = vehicle_transform.location
        v_yaw = math.radians(vehicle_transform.rotation.yaw)

        # Vector from vehicle to waypoint
        dx = waypoint_location.x - v_loc.x
        dy = waypoint_location.y - v_loc.y

        # Angle of that vector in world frame
        target_angle = math.atan2(dy, dx)

        # Heading error (wrapped to [-π, π])
        heading_error = target_angle - v_yaw
        heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi

        return self._pid.step(heading_error, timestamp)


# ---------------------------------------------------------------------------
# Convenience factory
# ---------------------------------------------------------------------------
def make_default_controllers() -> tuple[LongitudinalPID, LateralPID]:
    """Return a ready-to-use (longitudinal, lateral) controller pair."""
    return LongitudinalPID(), LateralPID()
