"""多编组纵向动力学的单元测试（只用标准库，pytest 与 unittest 都能跑）。

    python3 -m pytest tests -v
    python3 -m unittest discover -s tests -v
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from train_consist_coupler.consist import Consist, Coupler  # noqa: E402
from train_consist_coupler.params import (CouplerParams, ControlParams,  # noqa: E402
                                          DEFAULT_CAR, Route, SINGLE_BODY)
from train_consist_coupler.profile import build_reference  # noqa: E402
from train_consist_coupler.scenarios import SCENARIOS  # noqa: E402
from train_consist_coupler.sim import (AtoController, coupling_delays,  # noqa: E402
                                       run_scenario)

SHORT = {"length": 700.0, "speed_limit": 90.0 / 3.6}


def short_spec(**overrides):
    spec = dict(SHORT)
    spec.update(overrides)
    return spec


class CouplerTest(unittest.TestCase):
    def setUp(self):
        self.coupler = Coupler(CouplerParams())

    def test_dead_zone_carries_no_force(self):
        self.assertEqual(self.coupler.force(0.0, 0.0), 0.0)
        self.assertEqual(self.coupler.force(0.001, 0.0), 0.0)

    def test_compression_pushes_forward(self):
        force = self.coupler.force(0.012, 0.0)
        self.assertAlmostEqual(force, 25e6 * 0.010, places=3)

    def test_tension_pulls_backward(self):
        force = self.coupler.force(-0.012, 0.0)
        self.assertAlmostEqual(force, -25e6 * 0.010, places=3)

    def test_damping_follows_relative_speed(self):
        base = self.coupler.force(0.012, 0.0)
        faster = self.coupler.force(0.012, 0.5)
        self.assertAlmostEqual(faster - base, 3e5 * 0.5, places=3)

    def test_force_is_continuous_at_the_gap_edge(self):
        just_inside = self.coupler.force(0.0022, 0.0)
        just_outside = self.coupler.force(-0.0022, 0.0)
        self.assertLess(abs(just_inside), 20e3)
        self.assertLess(abs(just_outside), 20e3)


class VehicleTest(unittest.TestCase):
    def test_traction_constant_torque_at_low_speed(self):
        self.assertAlmostEqual(DEFAULT_CAR.traction(0.0), DEFAULT_CAR.f_start)

    def test_traction_power_limited_at_high_speed(self):
        self.assertAlmostEqual(DEFAULT_CAR.traction(40.0),
                               DEFAULT_CAR.power / 40.0)

    def test_resistance_grows_with_speed(self):
        speeds = [0.0, 5.0, 20.0, 44.0]
        forces = [DEFAULT_CAR.resistance(v) for v in speeds]
        self.assertEqual(forces, sorted(forces))

    def test_single_body_matches_four_cars(self):
        self.assertAlmostEqual(SINGLE_BODY.mass, 4 * DEFAULT_CAR.mass)
        self.assertAlmostEqual(SINGLE_BODY.f_start, 4 * DEFAULT_CAR.f_start)
        self.assertAlmostEqual(SINGLE_BODY.power, 4 * DEFAULT_CAR.power)
        self.assertAlmostEqual(SINGLE_BODY.davis_a, 4 * DEFAULT_CAR.davis_a)


class ConsistTest(unittest.TestCase):
    def setUp(self):
        self.consist = Consist(4, DEFAULT_CAR, CouplerParams(), Route())

    def test_one_car_per_length_spacing(self):
        for i in range(3):
            self.assertAlmostEqual(self.consist.x[i] - self.consist.x[i + 1],
                                   DEFAULT_CAR.length)

    def test_head_is_half_a_car_ahead_of_first_centre(self):
        self.assertAlmostEqual(self.consist.head, 0.0)
        self.assertAlmostEqual(self.consist.tail, -self.consist.length)

    def test_single_car_has_no_couplers(self):
        solo = Consist(1, SINGLE_BODY, CouplerParams(), Route())
        self.assertEqual(solo.coupler_forces(), [])

    def test_couplers_are_internal_forces(self):
        """纯压缩时车钩力不改变整车的质量加权加速度和。"""
        consist = Consist(4, DEFAULT_CAR, CouplerParams(), Route())
        consist.x[0] -= 0.010
        accelerations = consist.accelerations()
        total = sum(m * a for m, a in
                    zip([DEFAULT_CAR.mass] * 4, accelerations))
        expected = -4 * DEFAULT_CAR.resistance(0.0) / DEFAULT_CAR.gamma
        self.assertAlmostEqual(total, expected, delta=1.0)

    def test_compression_accelerates_the_front_car(self):
        consist = Consist(4, DEFAULT_CAR, CouplerParams(), Route())
        consist.x[0] -= 0.010          # 车头往后退一点 = 车钩被压缩
        accelerations = consist.accelerations()
        self.assertGreater(accelerations[0], accelerations[3])

    def test_velocity_never_goes_negative(self):
        consist = Consist(2, DEFAULT_CAR, CouplerParams(), Route())
        consist.force = [-2e5, -2e5]
        for _ in range(200):
            consist.integrate(0.002)
        self.assertGreaterEqual(min(consist.v), 0.0)


class DelayTest(unittest.TestCase):
    def test_head_first_increases_with_car_index(self):
        self.assertEqual(coupling_delays(4, 0.06, "head"), [0.0, 0.06, 0.12, 0.18])

    def test_tail_first_is_mirrored(self):
        self.assertEqual(coupling_delays(4, 0.06, "tail"), [0.18, 0.12, 0.06, 0.0])

    def test_zero_delay_for_single_car(self):
        self.assertEqual(coupling_delays(1, 0.06, "head"), [0.0])


class ProfileTest(unittest.TestCase):
    def setUp(self):
        route = Route(**SHORT)
        consist = Consist(4, DEFAULT_CAR, CouplerParams(), route)
        self.reference, self.envelope = build_reference(route, consist,
                                                        ControlParams())

    def test_envelope_never_exceeds_speed_limit(self):
        self.assertLessEqual(max(self.envelope.values), SHORT["speed_limit"] + 1e-6)

    def test_reference_ends_at_zero(self):
        self.assertLess(self.reference.values[-1], 0.01)

    def test_braking_tail_is_monotonic(self):
        values = self.reference.values
        peak = values.index(max(values))
        braking = values[peak:]
        self.assertEqual(braking, sorted(braking, reverse=True))


class ControllerTest(unittest.TestCase):
    def setUp(self):
        route = Route(**SHORT)
        self.consist = Consist(4, DEFAULT_CAR, CouplerParams(), route)
        reference, _envelope = build_reference(route, self.consist, ControlParams())
        self.controller = AtoController(reference, route, ControlParams(),
                                        route.length)

    def test_requests_traction_at_standstill(self):
        self.assertGreater(self.controller.command(0.0, 0.0), 0.0)

    def test_requests_braking_above_target(self):
        self.assertLess(self.controller.command(300.0, 25.0), 0.0)

    def test_command_respects_service_brake_limit(self):
        self.assertGreaterEqual(self.controller.command(650.0, 24.0),
                                -ControlParams().a_service - 1e-9)


class ScenarioTest(unittest.TestCase):
    def test_synchronised_consist_has_no_coupler_force(self):
        """全列同时施加力时，各车运动完全相同，车钩力必须严格为 0。"""
        result = run_scenario(short_spec(n_cars=4, car=DEFAULT_CAR,
                                         delay=0.0, direction="sync"))
        self.assertLess(abs(result["peak_buff"]), 1e-6)
        self.assertLess(abs(result["peak_draft"]), 1e-6)

    def test_head_and_tail_directions_are_mirrored(self):
        head = run_scenario(short_spec(n_cars=4, car=DEFAULT_CAR,
                                       delay=0.06, direction="head"))
        tail = run_scenario(short_spec(n_cars=4, car=DEFAULT_CAR,
                                       delay=0.06, direction="tail"))
        self.assertLess(head["peak_draft"], -10e3)
        self.assertGreater(tail["peak_buff"], 10e3)
        self.assertGreater(abs(head["peak_draft"]), abs(head["peak_buff"]))
        self.assertGreater(abs(tail["peak_buff"]), abs(tail["peak_draft"]))

    def test_longer_delay_gives_larger_force(self):
        short = run_scenario(short_spec(n_cars=4, car=DEFAULT_CAR,
                                        delay=0.03, direction="tail"))
        long = run_scenario(short_spec(n_cars=4, car=DEFAULT_CAR,
                                       delay=0.20, direction="tail"))
        self.assertGreater(long["peak_buff"], short["peak_buff"])

    def test_stop_error_stays_inside_the_window(self):
        for name in ("consist_head_first", "consist_tail_first"):
            spec = dict(SCENARIOS[name])
            spec.update(SHORT)
            result = run_scenario(spec)
            self.assertLess(abs(result["stop_error"]), 0.3, name)

    def test_run_is_deterministic(self):
        first = run_scenario(short_spec(n_cars=4, car=DEFAULT_CAR,
                                        delay=0.06, direction="head"))
        second = run_scenario(short_spec(n_cars=4, car=DEFAULT_CAR,
                                         delay=0.06, direction="head"))
        self.assertEqual(first["stop_error"], second["stop_error"])
        self.assertEqual(first["peak_draft"], second["peak_draft"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
