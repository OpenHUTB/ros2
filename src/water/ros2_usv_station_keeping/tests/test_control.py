"""控制、推力分配与状态监视的测试。"""
from __future__ import annotations

import math

import pytest

from ros2_usv_station_keeping.allocator import allocate, yaw_authority
from ros2_usv_station_keeping.control import ControlConfig, StationKeepingController
from ros2_usv_station_keeping.monitor import (
    FlowEstimator,
    FlowEstimatorConfig,
    HealthConfig,
    HealthLevel,
    ThrusterHealth,
    policy_for,
)
from ros2_usv_station_keeping.vehicle import VehicleParams, VehicleState


class TestSurgeControl:
    def test_positive_error_gives_positive_thrust(self):
        ctrl = StationKeepingController()
        out = ctrl.update(0.3, 0.0, 0.0, 0.0, 0.0)
        assert out.surge > 0.0

    def test_overshoot_gives_braking(self):
        ctrl = StationKeepingController()
        out = ctrl.update(0.1, 0.0, 0.0, 0.5, 0.0)
        assert out.surge < 0.0

    def test_steady_state_matches_drag_feedforward(self):
        cfg = ControlConfig()
        ctrl = StationKeepingController(cfg)
        out = ctrl.update(0.3, 0.0, 0.0, 0.3, 0.0)
        assert out.surge == pytest.approx(cfg.surge_feedforward_drag * 0.3, rel=0.05)

    def test_deadband_disables_thrust(self):
        ctrl = StationKeepingController()
        out = ctrl.update(0.0, 0.0, 0.0, 0.0, 0.0)
        assert out.surge == pytest.approx(0.0)
        assert out.active is False


class TestYawControl:
    def test_positive_heading_error_yaws_left(self):
        ctrl = StationKeepingController()
        out = ctrl.update(0.2, 0.3, 0.0, 0.2, 0.0)
        assert out.yaw > 0.0

    def test_negative_heading_error_yaws_right(self):
        ctrl = StationKeepingController()
        out = ctrl.update(0.2, -0.3, 0.0, 0.2, 0.0)
        assert out.yaw < 0.0

    def test_heading_deadband_at_low_speed(self):
        ctrl = StationKeepingController()
        out = ctrl.update(0.0, math.radians(0.5), 0.0, 0.0, 0.0)
        assert out.yaw == pytest.approx(0.0)

    def test_yaw_rate_damping(self):
        ctrl = StationKeepingController()
        still = ctrl.update(0.2, 0.3, 0.0, 0.2, 0.0)
        spinning = ctrl.update(0.2, 0.3, 0.0, 0.2, 0.5)
        assert abs(spinning.yaw) < abs(still.yaw)


class TestAllocation:
    def test_pure_surge_is_symmetric(self):
        params = VehicleParams()
        out = allocate(4.0, 0.0, params)
        assert out.left == pytest.approx(out.right)
        assert out.left == pytest.approx(2.0)

    def test_pure_yaw_is_antisymmetric(self):
        params = VehicleParams()
        out = allocate(0.0, 1.0, params)
        assert out.left == pytest.approx(-out.right)

    def test_saturation_detected(self):
        params = VehicleParams()
        out = allocate(50.0, 0.0, params)
        assert out.saturated is True
        assert out.left == pytest.approx(params.thrust_max)

    def test_shortfall_reported_on_saturation(self):
        params = VehicleParams()
        out = allocate(50.0, 0.0, params)
        assert out.shortfall > 0.0

    def test_unsatisfied_side_is_compensated(self):
        """一侧被限幅时，另一侧应尽量补上总推力。"""
        params = VehicleParams()
        out = allocate(0.0, -3.0, params)   # 大力矩，左侧被推到上限
        assert out.left == pytest.approx(params.thrust_max) or out.right == pytest.approx(params.thrust_min)

    def test_efficiency_compensation_scales_command(self):
        params = VehicleParams()
        healthy = allocate(2.0, 0.0, params)
        degraded = allocate(2.0, 0.0, params, efficiency=(0.5, 0.5))
        assert degraded.left == pytest.approx(healthy.left * 2, rel=0.05)

    def test_zero_spacing_rejected(self):
        with pytest.raises(ValueError):
            allocate(1.0, 1.0, VehicleParams(thruster_spacing=0.0))

    def test_yaw_authority_shrinks_at_full_thrust(self):
        params = VehicleParams()
        assert yaw_authority(0.0, params) > yaw_authority(2 * params.thrust_max, params)


class TestFlowEstimator:
    def _feed(self, est: FlowEstimator, u: float, v: float, r: float, thrust: float, steps: int, dt: float = 0.05):
        for _ in range(steps):
            est.update(0.0, 0.0, 0.0, u, v, r, thrust, dt)

    def test_zero_current_estimates_zero(self):
        est = FlowEstimator()
        self._feed(est, 0.0, 0.0, 0.0, 0.0, 400)
        assert abs(est.current_body[0]) < 0.05

    def test_head_current_recovered_from_dynamics(self):
        """船被顶住（地速 0）、推力等于阻力时，纵向应估出 -|c|。"""
        params = VehicleParams()
        est = FlowEstimator(params=params)
        current = 0.2
        thrust = params.damp_surge * current
        self._feed(est, 0.0, 0.0, 0.0, thrust, 1200)
        assert est.current_body[0] == pytest.approx(-current, abs=0.06)

    def test_estimate_is_clamped(self):
        est = FlowEstimator(FlowEstimatorConfig(max_current=0.3))
        self._feed(est, 0.0, 0.0, 0.0, 50.0, 800)
        assert est.magnitude <= 0.35

    def test_reset_clears_state(self):
        est = FlowEstimator()
        self._feed(est, 0.3, 0.0, 0.0, 3.0, 100)
        est.reset()
        assert est.current_body == (0.0, 0.0)
        assert est.valid is False


class TestThrusterHealth:
    def test_healthy_stays_nominal(self):
        health = ThrusterHealth()
        for _ in range(200):
            health.observe(3.0, 3.0, 0.05)
        assert health.level == HealthLevel.NOMINAL

    def test_low_efficiency_degrades(self):
        cfg = HealthConfig()
        health = ThrusterHealth(cfg)
        for _ in range(400):
            health.observe(3.0, 3.0 * 0.5, 0.05)
        assert health.level in (HealthLevel.DEGRADED, HealthLevel.FAILED)

    def test_severe_loss_fails(self):
        health = ThrusterHealth()
        for _ in range(600):
            health.observe(4.0, 0.2, 0.05)
        assert health.level == HealthLevel.FAILED

    def test_small_command_ignored(self):
        health = ThrusterHealth()
        for _ in range(200):
            health.observe(0.1, 0.0, 0.05)
        assert health.level == HealthLevel.NOMINAL
        assert health.efficiency == pytest.approx(1.0)

    def test_hysteresis_prevents_chatter(self):
        health = ThrusterHealth()
        for _ in range(400):
            health.observe(3.0, 1.6, 0.05)
        level = health.level
        for _ in range(20):
            health.observe(3.0, 1.7, 0.05)
        assert health.level == level

    def test_policy_relaxes_with_health(self):
        nominal = policy_for(HealthLevel.NOMINAL)
        failed = policy_for(HealthLevel.FAILED)
        assert failed.deadband_radius > nominal.deadband_radius
        assert failed.hold_position is False
