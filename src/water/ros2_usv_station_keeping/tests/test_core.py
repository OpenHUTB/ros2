"""几何工具与船体运动模型的基础测试。"""
from __future__ import annotations

import math

import pytest

from ros2_usv_station_keeping.geometry import body_to_world, clamp, world_to_body, wrap_angle
from ros2_usv_station_keeping.vehicle import (
    VehicleParams,
    VehicleState,
    integrate,
    relative_velocity,
    thrust_to_wrench,
    turning_radius,
)


class TestWrapAngle:
    def test_zero_stays_zero(self):
        assert wrap_angle(0.0) == pytest.approx(0.0)

    def test_pi_wraps_to_negative_pi(self):
        assert wrap_angle(math.pi) == pytest.approx(-math.pi, abs=1e-9)

    def test_three_pi_maps_into_range(self):
        assert abs(wrap_angle(3 * math.pi)) <= math.pi

    def test_negative_angle_normalised(self):
        assert wrap_angle(-math.pi / 2) == pytest.approx(-math.pi / 2)

    def test_large_rotation_equivalent(self):
        a = 0.3
        assert wrap_angle(a + 8 * math.pi) == pytest.approx(a, abs=1e-9)


class TestCoordinateTransform:
    def test_round_trip_identity(self):
        angle = 0.7
        vx, vy = world_to_body(angle, *body_to_world(angle, 1.3, -0.4))
        assert vx == pytest.approx(1.3)
        assert vy == pytest.approx(-0.4)

    def test_heading_ninety_rotates_x_into_y(self):
        x, y = body_to_world(math.pi / 2, 1.0, 0.0)
        assert x == pytest.approx(0.0, abs=1e-9)
        assert y == pytest.approx(1.0)

    def test_clamp_bounds(self):
        assert clamp(5.0, 0.0, 1.0) == 1.0
        assert clamp(-5.0, 0.0, 1.0) == 0.0

    def test_clamp_rejects_inverted_range(self):
        with pytest.raises(ValueError):
            clamp(0.5, 1.0, 0.0)


class TestThrustMapping:
    def test_symmetric_thrust_gives_no_yaw(self):
        surge, yaw = thrust_to_wrench(2.0, 2.0, VehicleParams())
        assert surge == pytest.approx(4.0)
        assert yaw == pytest.approx(0.0)

    def test_more_right_thrust_turns_left(self):
        _, yaw = thrust_to_wrench(1.0, 3.0, VehicleParams())
        assert yaw > 0.0

    def test_more_left_thrust_turns_right(self):
        _, yaw = thrust_to_wrench(3.0, 1.0, VehicleParams())
        assert yaw < 0.0

    def test_yaw_scales_with_spacing(self):
        narrow = VehicleParams(thruster_spacing=0.2)
        wide = VehicleParams(thruster_spacing=0.8)
        assert thrust_to_wrench(1.0, 3.0, wide)[1] == pytest.approx(
            4.0 * thrust_to_wrench(1.0, 3.0, narrow)[1]
        )


class TestRelativeVelocity:
    def test_no_current_means_equal(self):
        state = VehicleState(u=0.4, v=0.1)
        assert relative_velocity(state, (0.0, 0.0)) == pytest.approx((0.4, 0.1))

    def test_head_current_reduces_forward_speed(self):
        state = VehicleState(psi=0.0, u=0.4, v=0.0)
        u_rel, _ = relative_velocity(state, (0.2, 0.0))
        assert u_rel == pytest.approx(0.2)

    def test_cross_current_appears_in_sway(self):
        state = VehicleState(psi=0.0, u=0.0, v=0.0)
        _, v_rel = relative_velocity(state, (0.0, 0.3))
        assert v_rel == pytest.approx(-0.3)


class TestIntegration:
    def test_no_thrust_no_motion(self):
        params = VehicleParams()
        state = VehicleState()
        out = integrate(state, params, 0.0, 0.0, (0.0, 0.0), 0.1)
        assert out.x == pytest.approx(0.0)
        assert out.u == pytest.approx(0.0)

    def test_forward_thrust_increases_speed(self):
        params = VehicleParams()
        out = integrate(VehicleState(), params, 1.0, 1.0, (0.0, 0.0), 0.1)
        assert out.u > 0.0

    def test_speed_saturates_at_drag_balance(self):
        params = VehicleParams()
        state = VehicleState()
        for _ in range(3000):
            state = integrate(state, params, params.thrust_max, params.thrust_max, (0.0, 0.0), 0.05)
        assert state.u == pytest.approx(params.max_surge_speed, rel=0.05)

    def test_current_drags_idle_boat(self):
        params = VehicleParams()
        state = VehicleState()
        for _ in range(400):
            state = integrate(state, params, 0.0, 0.0, (0.0, 0.3), 0.05)
        assert state.y > 1.0

    def test_negative_dt_rejected(self):
        with pytest.raises(ValueError):
            integrate(VehicleState(), VehicleParams(), 0.0, 0.0, (0.0, 0.0), 0.0)

    def test_state_not_mutated(self):
        params = VehicleParams()
        state = VehicleState()
        integrate(state, params, 2.0, 2.0, (0.0, 0.0), 0.1)
        assert state.u == 0.0


class TestTurningRadius:
    def test_straight_line_has_infinite_radius(self):
        assert math.isinf(turning_radius(1.0, 0.0))

    def test_radius_is_speed_over_rate(self):
        assert turning_radius(0.5, 0.25) == pytest.approx(2.0)
