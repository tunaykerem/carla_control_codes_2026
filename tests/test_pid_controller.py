"""
tests/test_pid_controller.py
----------------------------
Unit tests for pid_controller.py (no CARLA server required).
"""

import math
import sys
import os

# Make sure the package root is on the path when running from tests/ directory
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest
from pid_controller import PIDController, LongitudinalPID, LateralPID


# ---------------------------------------------------------------------------
# PIDController
# ---------------------------------------------------------------------------
class TestPIDController:
    def test_proportional_only(self):
        pid = PIDController(kp=2.0, ki=0.0, kd=0.0)
        output = pid.step(error=5.0, timestamp=0.0)
        # P term only: 2.0 * 5.0 = 10.0 (no clamp)
        assert pytest.approx(output, rel=1e-6) == 10.0

    def test_output_limits_clamp(self):
        pid = PIDController(kp=10.0, ki=0.0, kd=0.0, output_limits=(-1.0, 1.0))
        output = pid.step(error=5.0, timestamp=0.0)
        assert output == pytest.approx(1.0)

    def test_output_limits_negative_clamp(self):
        pid = PIDController(kp=10.0, ki=0.0, kd=0.0, output_limits=(-1.0, 1.0))
        output = pid.step(error=-5.0, timestamp=0.0)
        assert output == pytest.approx(-1.0)

    def test_integral_accumulates(self):
        pid = PIDController(kp=0.0, ki=1.0, kd=0.0, max_integral=1000.0)
        pid.step(error=1.0, timestamp=0.0)
        output = pid.step(error=1.0, timestamp=1.0)
        # After two steps: integral ≈ 1.0 * 1.0 (second dt) + first integral
        # first step: integral += 1.0 * 1e-4 ≈ 0 (dt is tiny on first step)
        # second step: integral += 1.0 * 1.0 = 1.0  → output ≈ 1.0
        assert output > 0.9

    def test_anti_windup(self):
        pid = PIDController(kp=0.0, ki=100.0, kd=0.0, max_integral=1.0)
        for t in range(100):
            pid.step(error=1.0, timestamp=float(t))
        # integral must be clamped to max_integral=1.0
        assert abs(pid._integral) <= 1.0 + 1e-9

    def test_reset_clears_state(self):
        pid = PIDController(kp=1.0, ki=1.0, kd=1.0)
        pid.step(error=5.0, timestamp=0.0)
        pid.reset()
        assert pid._integral == 0.0
        assert pid._prev_error == 0.0
        assert pid._prev_time is None

    def test_derivative_term(self):
        pid = PIDController(kp=0.0, ki=0.0, kd=1.0)
        pid.step(error=0.0, timestamp=0.0)
        # error jumps from 0 → 4 over dt=1s  →  derivative = 4/1 = 4
        output = pid.step(error=4.0, timestamp=1.0)
        assert pytest.approx(output, rel=1e-3) == 4.0


# ---------------------------------------------------------------------------
# LongitudinalPID
# ---------------------------------------------------------------------------
class TestLongitudinalPID:
    def test_throttle_when_below_target(self):
        pid = LongitudinalPID(kp=1.0, ki=0.0, kd=0.0)
        throttle, brake = pid.run_step(target_speed=30.0, current_speed=10.0,
                                       timestamp=0.0)
        assert throttle > 0.0
        assert brake == 0.0

    def test_brake_when_above_target(self):
        pid = LongitudinalPID(kp=1.0, ki=0.0, kd=0.0)
        throttle, brake = pid.run_step(target_speed=10.0, current_speed=60.0,
                                       timestamp=0.0)
        assert throttle == 0.0
        assert brake > 0.0

    def test_outputs_within_bounds(self):
        pid = LongitudinalPID(kp=5.0, ki=0.5, kd=0.2)
        for speed in range(0, 120, 10):
            throttle, brake = pid.run_step(30.0, float(speed),
                                           timestamp=float(speed) * 0.1)
            assert 0.0 <= throttle <= 1.0
            assert 0.0 <= brake <= 1.0

    def test_reset(self):
        pid = LongitudinalPID()
        pid.run_step(50.0, 0.0, timestamp=0.0)
        pid.reset()
        # After reset, integral should be 0
        assert pid._pid._integral == 0.0


# ---------------------------------------------------------------------------
# LateralPID  (uses duck-typed location / transform objects)
# ---------------------------------------------------------------------------
class _Location:
    def __init__(self, x, y, z=0.0):
        self.x = x
        self.y = y
        self.z = z


class _Rotation:
    def __init__(self, yaw):
        self.yaw = yaw  # degrees


class _Transform:
    def __init__(self, x, y, yaw_deg):
        self.location = _Location(x, y)
        self.rotation = _Rotation(yaw_deg)


class TestLateralPID:
    def test_steer_right_for_waypoint_to_the_right(self):
        pid = LateralPID(kp=1.0, ki=0.0, kd=0.0)
        # Vehicle at origin facing East (yaw=0)
        transform = _Transform(0.0, 0.0, 0.0)
        # Waypoint directly to the right (South in CARLA left-hand coords, positive Y)
        waypoint = _Location(1.0, -10.0, 0.0)
        steer = pid.run_step(waypoint, transform, timestamp=0.0)
        # heading error should be negative → steer negative (left)
        assert steer < 0.0

    def test_steer_zero_when_on_heading(self):
        pid = LateralPID(kp=1.0, ki=0.0, kd=0.0)
        # Vehicle at origin facing East (yaw=0), waypoint straight ahead
        transform = _Transform(0.0, 0.0, 0.0)
        waypoint = _Location(10.0, 0.0, 0.0)
        steer = pid.run_step(waypoint, transform, timestamp=0.0)
        assert abs(steer) < 0.05  # small heading error

    def test_output_within_bounds(self):
        pid = LateralPID(kp=5.0, ki=0.0, kd=0.0)
        transform = _Transform(0.0, 0.0, 45.0)
        waypoint = _Location(0.0, 100.0)
        steer = pid.run_step(waypoint, transform, timestamp=0.0)
        assert -1.0 <= steer <= 1.0
