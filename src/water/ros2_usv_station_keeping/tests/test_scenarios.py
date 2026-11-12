"""闭环场景测试：这些断言直接对应文档里给出的量化结论。"""
from __future__ import annotations

import math
from dataclasses import replace

import pytest

from ros2_usv_station_keeping.pilot import PilotConfig, StationKeepingPilot
from ros2_usv_station_keeping.vehicle import VehicleState

from sim.experiments import compare_crab, run_episode
from sim.scenarios import SCENARIOS, get_scenario
from sim.water_world import EnvironmentConfig, WaterWorld


def short(key: str, seconds: float = 90.0):
    return replace(get_scenario(key), duration=seconds)


class TestCalmWater:
    def test_converges_tightly(self):
        metrics, _ = run_episode(short("calm"), noise_quality=0.0)
        assert metrics.rms_error < 0.25

    def test_energy_is_small(self):
        metrics, _ = run_episode(short("calm"), noise_quality=0.0)
        assert metrics.energy < 120.0

    def test_settles_within_episode(self):
        metrics, _ = run_episode(short("calm", 120.0), noise_quality=0.0)
        assert math.isfinite(metrics.settle_time)


class TestCurrentRejection:
    def test_cross_current_holds_position(self):
        metrics, _ = run_episode(short("cross_current"), noise_quality=0.0)
        assert metrics.rms_error < 0.45

    def test_crab_compensation_is_what_makes_it_work(self):
        """关掉横漂补偿后，横流场景的保持误差必须明显变大。"""
        on, off = compare_crab(short("cross_current"), noise_quality=0.0)
        assert off.rms_error > on.rms_error * 1.3

    def test_oblique_current_holds_position(self):
        metrics, _ = run_episode(short("oblique_current"), noise_quality=0.0)
        assert metrics.rms_error < 0.45

    def test_crab_angle_is_used(self):
        metrics, _ = run_episode(short("cross_current"), noise_quality=0.0)
        assert metrics.mean_crab_deg > 3.0


class TestGust:
    def test_survives_gust(self):
        metrics, _ = run_episode(short("gust"), noise_quality=0.0)
        assert metrics.rms_error < 0.75
        assert math.isfinite(metrics.final_error)


class TestDegradedThruster:
    def test_still_holds(self):
        metrics, _ = run_episode(short("degraded_thruster"), noise_quality=0.0)
        assert metrics.rms_error < 0.45

    def test_efficiency_estimate_drops(self):
        metrics, _ = run_episode(short("degraded_thruster"), noise_quality=0.0)
        assert metrics.min_health_efficiency < 1.0


class TestFailedThruster:
    def test_reports_failure_and_stops_fighting(self):
        metrics, _ = run_episode(short("failed_thruster", 60.0), noise_quality=0.0)
        assert metrics.final_health == "FAILED"

    def test_energy_collapses_after_giving_up(self):
        metrics, _ = run_episode(short("failed_thruster", 60.0), noise_quality=0.0)
        assert metrics.energy < 300.0


class TestRobustness:
    @pytest.mark.parametrize("key", [s.key for s in SCENARIOS])
    def test_no_nan_in_trace(self, key):
        _, trace = run_episode(short(key, 30.0), noise_quality=0.0)
        for series in (trace.x, trace.y, trace.psi, trace.u, trace.distance):
            assert all(math.isfinite(v) for v in series)

    def test_noisy_sensors_do_not_destabilise(self):
        metrics, _ = run_episode(short("calm"), noise_quality=1.0)
        assert metrics.rms_error < 0.4

    def test_unknown_scenario_raises(self):
        with pytest.raises(KeyError):
            get_scenario("does_not_exist")


class TestWorldPhysics:
    def test_idle_boat_drifts_with_current(self):
        world = WaterWorld(env=EnvironmentConfig(current_x=0.0, current_y=0.25, seed=1))
        for _ in range(600):
            world.step(0.0, 0.0, 0.05)
        assert world.state.y > 2.0

    def test_asymmetric_efficiency_induces_turn(self):
        world = WaterWorld(
            env=EnvironmentConfig(seed=1),
            efficiency=(0.0, 1.0),
        )
        for _ in range(200):
            world.step(2.0, 2.0, 0.05)
        assert abs(world.state.psi) > 0.2

    def test_observation_respects_noise_scale(self):
        world = WaterWorld(env=EnvironmentConfig(seed=3))
        quiet = [world.observe(0.0).x for _ in range(50)]
        assert max(quiet) - min(quiet) < 1e-9


class TestPilotIntegration:
    def test_stationary_in_calm_water_needs_no_thrust(self):
        pilot = StationKeepingPilot()
        pilot.reset(0.0)
        command = None
        for _ in range(60):
            command = pilot.step(VehicleState(x=0.0, y=0.0, psi=0.0), (0.0, 0.0), 0.05)
        assert command is not None
        assert abs(command.left) < 0.2 and abs(command.right) < 0.2

    def test_disabled_crab_ignores_estimate(self):
        pilot = StationKeepingPilot(config=PilotConfig(use_crab_compensation=False))
        pilot.reset(0.0)
        command = pilot.step(VehicleState(x=2.0, y=0.0, psi=0.0), (0.0, 0.0), 0.05)
        assert abs(command.snapshot.crab_deg) < 1.0
