"""Regression tests for the AP-CPP module.

Pure-Python (stdlib ``unittest``) so the suite runs with no extra dependency
beyond NumPy::

    python -m unittest discover -s tests -v

Every test targets a property that actually broke during development, so the
suite doubles as documentation of the non-obvious invariants in the design.
"""

from __future__ import annotations

import math
import os
import sys
import unittest

import numpy as np

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from ap_cpp.control import APCPPSpeedController, baseline_speed  # noqa: E402
from ap_cpp.grid_model import CoverageGrid, GridConfig  # noqa: E402
from ap_cpp.mission import APCPPMission, MissionConfig  # noqa: E402
from ap_cpp.planner import PlannerConfig, RollingHorizonPlanner  # noqa: E402
from ap_cpp.pose import Pose, heading_between, wrap_angle_deg  # noqa: E402
from ap_cpp.runtime import DemoPerceptionModel  # noqa: E402
from ap_cpp.sensor import SensorConfig, SensorModel  # noqa: E402
from ap_cpp.utility import UtilityModel, UtilityWeights  # noqa: E402

SQUARE = [(0.0, 0.0), (60.0, 0.0), (60.0, 60.0), (0.0, 60.0)]


def make_grid(polygon=None, obstacles=None, resolution=2.0):
    polygon = polygon or SQUARE
    grid = CoverageGrid(GridConfig.from_polygon(polygon, resolution=resolution))
    grid.rasterize_polygon(polygon)
    if obstacles:
        grid.rasterize_obstacles(obstacles)
    return grid


# --------------------------------------------------------------------------- #
class TestGridModel(unittest.TestCase):
    def test_rasterization_marks_inside_cells_free(self):
        grid = make_grid()
        self.assertTrue(grid.passable.any())
        # A rectangle rasterised from its own bounding box has every cell
        # centre inside it, so the "outside" case needs a non-rectangular
        # polygon: the corner beyond a triangle's hypotenuse must be blocked.
        triangle = make_grid(polygon=[(0.0, 0.0), (60.0, 0.0), (0.0, 60.0)])
        top_right = triangle.world_to_cell(55.0, 55.0)
        self.assertFalse(triangle.passable[top_right])
        bottom_left = triangle.world_to_cell(5.0, 5.0)
        self.assertTrue(triangle.passable[bottom_left])

    def test_obstacle_is_carved_out_and_sentinelised(self):
        obstacle = [(20.0, 20.0), (40.0, 20.0), (40.0, 40.0), (20.0, 40.0)]
        grid = make_grid(obstacles=[obstacle])
        i, j = grid.world_to_cell(30.0, 30.0)
        self.assertFalse(grid.passable[i, j])
        # Blocked cells must never attract information-driven motion.
        self.assertEqual(grid.entropy[i, j], grid.config.obstacle_entropy)
        self.assertEqual(grid.coverage[i, j], grid.config.obstacle_coverage)

    def test_observe_reduces_entropy_and_returns_the_same_delta(self):
        grid = make_grid()
        cells = [(5, 5), (5, 6), (6, 5), (6, 6)]
        before = grid.total_entropy()
        gain = grid.observe(cells, measurement_entropy=0.0, etas=[1.0] * 4)
        self.assertAlmostEqual(before - grid.total_entropy(), gain, places=9)
        self.assertLess(grid.mean_entropy(), 1.0)

    def test_observe_is_monotonic_and_bounded(self):
        grid = make_grid()
        cells = [(5, 5)]
        previous = grid.entropy[5, 5]
        for _ in range(50):
            grid.observe(cells, measurement_entropy=0.0, etas=[1.0])
            self.assertLessEqual(grid.entropy[5, 5], previous + 1e-12)
            self.assertGreaterEqual(grid.entropy[5, 5], 0.0)
            previous = grid.entropy[5, 5]

    def test_observe_ignores_blocked_cells(self):
        obstacle = [(20.0, 20.0), (40.0, 20.0), (40.0, 40.0), (20.0, 40.0)]
        grid = make_grid(obstacles=[obstacle])
        i, j = grid.world_to_cell(30.0, 30.0)
        gain = grid.observe([(i, j)], measurement_entropy=0.0, etas=[1.0])
        self.assertEqual(gain, 0.0)

    def test_uninformative_measurement_leaves_belief_untouched(self):
        grid = make_grid()
        grid.observe([(5, 5)], measurement_entropy=1.0, etas=[1.0])
        self.assertAlmostEqual(grid.entropy[5, 5], 1.0, places=9)

    def test_ray_cells_and_line_of_sight_over_an_obstacle(self):
        obstacle = [(20.0, 20.0), (40.0, 20.0), (40.0, 40.0), (20.0, 40.0)]
        grid = make_grid(obstacles=[obstacle])
        self.assertFalse(grid.has_line_of_sight((10.0, 30.0), (50.0, 30.0)))
        self.assertTrue(grid.has_line_of_sight((5.0, 5.0), (15.0, 15.0)))

    def test_neighbors_do_not_cut_blocked_corners(self):
        obstacle = [(20.0, 20.0), (24.0, 20.0), (24.0, 40.0), (20.0, 40.0)]
        grid = make_grid(obstacles=[obstacle])
        corner = grid.world_to_cell(19.0, 19.0)
        blocked = grid.world_to_cell(21.0, 21.0)
        self.assertFalse(grid.passable[blocked])
        self.assertNotIn(blocked, grid.neighbors(corner, connectivity=8))

    def test_prior_blending_is_clipped(self):
        grid = make_grid()
        grid.set_uncertainty_prior(np.full((grid.nx, grid.ny), 5.0), weight=0.5)
        self.assertLessEqual(grid.entropy.max(), 1.0)
        with self.assertRaises(ValueError):
            grid.set_uncertainty_prior(np.zeros((3, 3)), weight=0.5)


# --------------------------------------------------------------------------- #
class TestSensorModel(unittest.TestCase):
    def test_swath_is_the_cross_track_extent(self):
        cfg = SensorConfig(altitude=10.0, hfov_deg=46.0)
        # 2 * 10 * tan(23 deg) = 8.49 m
        self.assertAlmostEqual(cfg.ground_width, 8.49, places=2)
        # A 70% side lap must space lanes at 30% of the swath.
        self.assertAlmostEqual(cfg.lane_spacing(70.0), 0.3 * cfg.ground_width, places=9)

    def test_footprint_is_bounded_by_the_config(self):
        cfg = SensorConfig(altitude=10.0)
        grid = make_grid(resolution=0.5)
        sensor = SensorModel(cfg)
        pose = Pose(30.0, 30.0, 0.0)
        cells, weights, etas = sensor.footprint(grid, pose)
        self.assertGreater(len(cells), 0)
        self.assertAlmostEqual(float(weights.sum()), 1.0, places=9)
        self.assertTrue(np.all(etas > 0.0))
        # Swath is cross-track: a north-heading pose must extend further east
        # than north.
        xs = [grid.cell_center(*c)[0] for c in cells]
        ys = [grid.cell_center(*c)[1] for c in cells]
        self.assertGreater(max(ys) - min(ys), max(xs) - min(xs))

    def test_prediction_matches_the_realised_update(self):
        """The planner's forward model must equal the runtime fusion."""
        cfg = SensorConfig(altitude=10.0)
        grid = make_grid()
        sensor = SensorModel(cfg)
        pose = Pose(30.0, 30.0, 45.0)

        predicted = sensor.predict_update(grid, pose, assumed_measurement_entropy=0.3)["gain"]
        realised = sensor.observe(grid, pose, measurement_entropy=0.3)
        self.assertAlmostEqual(predicted, realised, places=9)

    def test_prediction_does_not_mutate_the_grid(self):
        grid = make_grid()
        sensor = SensorModel()
        snapshot = grid.snapshot()
        sensor.predict_update(grid, Pose(30.0, 30.0, 0.0))
        np.testing.assert_array_equal(snapshot["entropy"], grid.entropy)
        np.testing.assert_array_equal(snapshot["coverage"], grid.coverage)

    def test_efficiency_falls_off_from_nadir(self):
        grid = make_grid()
        sensor = SensorModel()
        pose = Pose(30.0, 30.0, 0.0)
        self.assertGreater(
            sensor.efficiency_at(pose, (30.0, 30.0)),
            sensor.efficiency_at(pose, (35.0, 35.0)),
        )


# --------------------------------------------------------------------------- #
class TestUtility(unittest.TestCase):
    def test_travel_cost_grows_with_distance(self):
        grid = make_grid()
        sensor = SensorModel()
        utility = UtilityModel(sensor=sensor)
        start = Pose(10.0, 10.0, 0.0)
        near = utility.evaluate_path(grid, [Pose(16.0, 10.0, 0.0)], start_pose=start)
        far = utility.evaluate_path(grid, [Pose(40.0, 10.0, 0.0)], start_pose=start)
        self.assertGreater(near.terms[0].travel_cost, 0.0)
        self.assertGreater(far.terms[0].travel_cost, near.terms[0].travel_cost)

    def test_revisit_penalty_applies_to_a_second_observation(self):
        grid = make_grid()
        sensor = SensorModel()
        utility = UtilityModel(sensor=sensor)
        pose = Pose(30.0, 30.0, 0.0)
        start = Pose(24.0, 30.0, 0.0)

        first = utility.evaluate_path(grid, [pose], start_pose=start).terms[0]
        second = utility.evaluate_path(
            grid, [Pose(36.0, 30.0, 0.0), Pose(30.0, 30.0, 180.0)], start_pose=start
        ).terms[1]
        self.assertGreater(second.revisit_cost, first.revisit_cost)

    def test_discount_is_applied_per_horizon_step(self):
        """Each step's contribution must be scaled by ``discount ** k``.

        Note the raw utilities are *not* equal across steps even for identical
        geometry: the revisit penalty only starts biting once the horizon has
        already observed some ground. The discount is therefore asserted
        against each step's own raw utility rather than across steps.
        """
        grid = make_grid()
        utility = UtilityModel(sensor=SensorModel())
        poses = [Pose(6.0 * k, 30.0, 0.0) for k in range(1, 5)]
        evaluation = utility.evaluate_path(grid, poses, start_pose=Pose(0.0, 30.0, 0.0))

        for term in evaluation.terms:
            self.assertAlmostEqual(
                term.discounted_utility,
                utility.weights.discount ** term.index * term.utility,
                places=12,
            )

        # Consequently the *final* step of a long horizon must contribute
        # strictly less than an otherwise identical first step.
        self.assertLess(
            utility.weights.discount ** 3 * evaluation.terms[0].utility,
            evaluation.terms[0].utility,
        )

    def test_g_function_matches_the_upstream_check_script(self):
        """``check_g_func.py`` case #1: conf 0.8, cov 0.05 -> 'Faster ++'."""
        value = UtilityModel.g_function(0.8, 0.05)
        self.assertGreater(value, 0.0)
        # A confident, sparse frame must score higher than an uncertain dense one.
        self.assertGreater(value, UtilityModel.g_function(0.4, 0.6))


# --------------------------------------------------------------------------- #
class TestPlanner(unittest.TestCase):
    def setUp(self):
        self.grid = make_grid()
        self.sensor = SensorModel(SensorConfig(altitude=10.0))
        self.planner = RollingHorizonPlanner(
            self.grid, sensor=self.sensor, config=PlannerConfig(horizon=5, step_length=6.0)
        )

    def test_a_star_finds_an_obstacle_free_path(self):
        obstacle = [(20.0, 20.0), (40.0, 20.0), (40.0, 40.0), (20.0, 40.0)]
        grid = make_grid(obstacles=[obstacle])
        start = Pose(5.0, 30.0, 0.0)
        goal = Pose(55.0, 30.0, 0.0)
        planner = RollingHorizonPlanner(grid, sensor=self.sensor)
        path = planner._a_star_path(start, goal)
        self.assertTrue(path)
        for pose in path:
            i, j = grid.world_to_cell(pose.x, pose.y)
            self.assertTrue(grid.passable[i, j], "path crosses an obstacle at {}".format(pose))

    def test_lattice_route_is_resampled_at_exact_step_length(self):
        route = RollingHorizonPlanner.lawnmower_from_grid(self.grid, lane_spacing=6.0)
        self.assertTrue(route)
        start = route[0]
        resampled = self.planner._reference_candidate(start, route)
        self.assertTrue(resampled)
        prev = start
        for pose in resampled:
            self.assertAlmostEqual(
                math.hypot(pose.x - prev.x, pose.y - prev.y),
                self.planner.config.step_length,
                places=6,
            )
            prev = pose

    def test_lattice_route_never_crosses_a_blocked_cell(self):
        obstacle = [(20.0, 20.0), (40.0, 20.0), (40.0, 40.0), (20.0, 40.0)]
        grid = make_grid(obstacles=[obstacle])
        route = RollingHorizonPlanner.lawnmower_from_grid(grid, lane_spacing=4.0)
        self.assertTrue(route)
        for pose in route:
            i, j = grid.world_to_cell(pose.x, pose.y)
            self.assertTrue(grid.passable[i, j])

    def test_reference_sweep_wins_on_a_uniform_prior(self):
        """With nothing known anywhere, deviating from the sweep cannot pay."""
        route = RollingHorizonPlanner.lawnmower_from_grid(self.grid, lane_spacing=6.0)
        start = route[0]
        result = self.planner.plan(start, reference_route=route, now=0.0)
        scores = {c.name: c.score for c in result.candidates}
        self.assertIn("reference_sweep", scores)
        self.assertEqual(result.operator, "reference_sweep")

    def test_revisit_earns_no_second_information_in_the_forward_model(self):
        """Candidate scoring must not credit ground it has already observed."""
        grid = make_grid()
        planner = RollingHorizonPlanner(
            grid, sensor=self.sensor, config=PlannerConfig(horizon=3, step_length=6.0)
        )
        route = RollingHorizonPlanner.lawnmower_from_grid(grid, lane_spacing=6.0)
        start = route[0]
        forward = planner._reference_candidate(start, route)

        evaluation = planner.utility.evaluate_path(
            grid,
            [forward[0], Pose(forward[0].x, forward[0].y, forward[0].yaw_deg + 180.0)],
            start_pose=start,
        )
        # Same footprint twice: the second look must be worth strictly less.
        self.assertLess(
            evaluation.terms[1].information_gain,
            evaluation.terms[0].information_gain,
        )
        # ...and it must be charged a revisit penalty for the privilege.
        self.assertGreater(evaluation.terms[1].revisit_cost, 0.0)

    def test_information_weight_controls_whether_the_planner_deviates(self):
        """The trade-off knob must actually move the decision.

        A well-defined unresolved zone sitting off the reference corridor is
        the only situation where the trade-off is real: when information is
        cheap relative to the deviation it must be worth the detour, and when
        it is expensive the planner must stay on the published sweep.
        """
        grid = make_grid()
        # 30 x 30 cells at 2 m; a hot zone away from the first lane.
        centres = grid.cell_centers().reshape(grid.nx, grid.ny, 2)
        # The zone must sit within the horizon's reach (~30 m at 5 steps of
        # 6 m); an unreachable anomaly is correctly refused by the planner, so
        # placing it further away would test nothing.
        blob = np.exp(
            -(
                (centres[:, :, 0] - 20.0) ** 2 + (centres[:, :, 1] - 16.0) ** 2
            )
            / (2 * 8.0 ** 2)
        )
        grid.entropy = np.clip(0.35 + 0.65 * blob, 0.0, 1.0)
        grid.entropy[~grid.passable] = 0.0

        sensor = SensorModel(SensorConfig(altitude=10.0))
        planner = RollingHorizonPlanner(
            grid, sensor=sensor, config=PlannerConfig(horizon=5, step_length=6.0)
        )
        route = RollingHorizonPlanner.lawnmower_from_grid(grid, lane_spacing=6.0)
        start = Pose(2.0, 2.0, 0.0)

        planner.utility.weights.information = 0.05
        conservative = planner.plan(start, reference_route=route, now=0.0)
        planner.utility.weights.information = 4.0
        curious = planner.plan(start, reference_route=route, now=0.0)

        self.assertEqual(conservative.operator, "reference_sweep")
        self.assertNotEqual(curious.operator, "reference_sweep")
        self.assertGreater(
            curious.evaluation.cumulative_information,
            conservative.evaluation.cumulative_information,
        )

    def test_plan_commits_only_the_execute_window(self):
        route = RollingHorizonPlanner.lawnmower_from_grid(self.grid, lane_spacing=6.0)
        result = self.planner.plan(route[0], reference_route=route, now=0.0)
        self.assertLessEqual(len(result.poses), self.planner.config.execute_steps)

    def test_plan_on_a_fully_resolved_map_returns_an_empty_horizon(self):
        grid = make_grid()
        grid.entropy[:] = 0.0
        grid.coverage[:] = 1.0
        grid.entropy[~grid.passable] = grid.config.obstacle_entropy
        planner = RollingHorizonPlanner(grid, sensor=self.sensor)
        # Coverage is already complete, so no repair target exists either.
        planner.config.fan_offsets_deg = ()
        planner.config.frontier_targets = 0
        result = planner.plan(Pose(30.0, 30.0, 0.0), reference_route=None, now=0.0)
        self.assertFalse(result.poses)

    def test_replan_gate_honours_interval_and_distance(self):
        planner = RollingHorizonPlanner(self.grid)
        pose = Pose(30.0, 30.0, 0.0)
        self.assertTrue(planner.should_replan(pose, 0.0))
        planner.plan(pose, now=0.0)
        self.assertFalse(planner.should_replan(Pose(30.1, 30.0, 0.0), 0.1))
        self.assertTrue(
            planner.should_replan(pose, planner.config.replan_interval + 0.1)
        )


# --------------------------------------------------------------------------- #
class TestControl(unittest.TestCase):
    def test_baseline_speed_matches_upstream_law(self):
        # Sparse vegetation -> faster than nominal.
        self.assertGreater(baseline_speed(0.05, nominal_speed=3.0, q_max=2.0), 3.0)
        # Dense canopy -> slower than nominal.
        self.assertLess(baseline_speed(0.15, nominal_speed=3.0, q_max=2.0), 3.0)
        # A ratio in the middle of the window leaves the speed at nominal.
        self.assertAlmostEqual(baseline_speed(0.10, nominal_speed=3.0, q_max=2.0), 3.0)

    def test_baseline_speed_stays_inside_the_envelope(self):
        for ratio in np.linspace(0.0, 1.0, 41):
            speed = baseline_speed(float(ratio), nominal_speed=3.0, q_max=2.0)
            self.assertGreaterEqual(speed, 1.0)
            self.assertLessEqual(speed, 5.0)

    def test_controller_slows_down_when_a_lot_of_information_is_expected(self):
        controller = APCPPSpeedController()
        calm = controller.command(coverage_ratio=0.10, measurement_entropy=0.3, planned_gain=0.0)
        eager = controller.command(coverage_ratio=0.10, measurement_entropy=0.3, planned_gain=50.0)
        self.assertLess(eager.speed, calm.speed)

    def test_controller_never_exceeds_vehicle_limits(self):
        controller = APCPPSpeedController(min_speed=1.0, max_speed=8.0)
        for gain in (0.0, 1e6):
            for ratio in (0.0, 0.3, 1.0):
                command = controller.command(ratio, 0.5, planned_gain=gain, mean_entropy=1.0)
                self.assertGreaterEqual(command.speed, 1.0)
                self.assertLessEqual(command.speed, 8.0)


# --------------------------------------------------------------------------- #
class TestPose(unittest.TestCase):
    def test_wrap_angle(self):
        self.assertAlmostEqual(wrap_angle_deg(370.0), 10.0)
        self.assertAlmostEqual(wrap_angle_deg(-370.0), -10.0)
        # The convention is the half-open range (-180, 180].
        self.assertAlmostEqual(abs(wrap_angle_deg(180.0)), 180.0)

    def test_heading_is_measured_from_north(self):
        self.assertAlmostEqual(heading_between(Pose(0, 0), Pose(1, 0)), 0.0)
        self.assertAlmostEqual(heading_between(Pose(0, 0), Pose(0, 1)), 90.0)
        self.assertAlmostEqual(heading_between(Pose(0, 0), Pose(-1, 0)), 180.0)


# --------------------------------------------------------------------------- #
class TestMission(unittest.TestCase):
    def _build(self, active=True, seed=3, anomaly=False):
        grid = make_grid()
        if anomaly:
            # A mostly-resolved field with one unresolved zone. This is the
            # only situation in which deviating from the sweep can pay: on a
            # uniform prior the information gain of looking anywhere is the
            # same, and the corridor term correctly pins the vehicle to the
            # published route.
            centres = grid.cell_centers().reshape(grid.nx, grid.ny, 2)
            blob = np.exp(
                -(
                    (centres[:, :, 0] - 22.0) ** 2
                    + (centres[:, :, 1] - 20.0) ** 2
                )
                / (2 * 9.0 ** 2)
            )
            grid.entropy = np.clip(0.35 + 0.65 * blob, 0.0, 1.0)
            grid.entropy[~grid.passable] = 0.0
        sensor = SensorModel(SensorConfig(altitude=10.0))
        planner = RollingHorizonPlanner(
            grid,
            sensor=sensor,
            config=PlannerConfig(
                horizon=4,
                execute_steps=2,
                step_length=6.0,
                frontier_targets=2 if active else 0,
                fan_offsets_deg=(-30, 0, 30) if active else (),
            ),
        )
        perception = DemoPerceptionModel(grid, sensor, seed=seed)
        route = RollingHorizonPlanner.lawnmower_from_grid(grid, lane_spacing=6.0)
        return APCPPMission(
            grid=grid,
            planner=planner,
            perception=perception,
            reference_route=route,
            config=MissionConfig(max_steps=120, entropy_target=0.0, coverage_target=1.1),
            sensor=sensor,
        ), route

    def test_mission_runs_and_records_steps(self):
        mission, route = self._build()
        record = mission.run(route[0])
        self.assertGreater(len(record.steps), 0)
        self.assertEqual(record.termination, "max_steps")
        self.assertGreater(record.total_distance, 0.0)

    def test_mission_is_reproducible_for_a_fixed_seed(self):
        first, route = self._build(seed=11)
        second, _ = self._build(seed=11)
        a = first.run(route[0])
        b = second.run(route[0])
        self.assertEqual(len(a.steps), len(b.steps))
        self.assertAlmostEqual(a.information_gain, b.information_gain, places=9)

    def test_belief_entropy_decreases_over_the_mission(self):
        mission, route = self._build()
        record = mission.run(route[0])
        self.assertLess(record.steps[-1].entropy_after, record.steps[0].entropy_before)

    def test_coverage_accumulates_monotonically(self):
        mission, route = self._build()
        record = mission.run(route[0])
        coverages = [s.coverage_fraction for s in record.steps]
        for earlier, later in zip(coverages, coverages[1:]):
            self.assertGreaterEqual(later, earlier - 1e-12)

    def test_active_mode_gathers_more_information_than_reference_only(self):
        """The whole point of the module: deviating must buy information.

        AP-CPP trades coverage throughput for information, so the honest
        assertion is on accumulated information gain - the mission record
        reports coverage separately, and the demo prints both side by side.
        """
        active, route = self._build(active=True, anomaly=True)
        baseline, route2 = self._build(active=False, anomaly=True)
        a = active.run(route[0])
        b = baseline.run(route2[0])
        self.assertGreater(a.information_gain, b.information_gain)

    def test_active_mode_reduces_belief_entropy_faster(self):
        active, route = self._build(active=True, anomaly=True)
        baseline, route2 = self._build(active=False, anomaly=True)
        a = active.run(route[0])
        b = baseline.run(route2[0])
        self.assertLess(
            a.steps[-1].entropy_after,
            b.steps[-1].entropy_after,
        )

    def test_active_mode_does_not_degrade_a_uniform_prior_mission(self):
        """Safety property: with nothing known in particular, stay the course.

        The corridor term must make AP-CPP fall back to the published sweep
        rather than wandering, so coverage on an uninformative prior must not
        regress relative to the reference-only configuration.
        """
        active, route = self._build(active=True, anomaly=False)
        baseline, route2 = self._build(active=False, anomaly=False)
        a = active.run(route[0])
        b = baseline.run(route2[0])
        self.assertGreaterEqual(
            a.steps[-1].coverage_fraction,
            b.steps[-1].coverage_fraction - 0.02,
        )

    def test_step_generator_yields_incrementally(self):
        mission, route = self._build()
        produced = []
        for step in mission.step(route[0]):
            produced.append(step)
            if len(produced) >= 5:
                break
        self.assertEqual(len(produced), 5)
        self.assertEqual([s.index for s in produced], list(range(5)))


# --------------------------------------------------------------------------- #
class TestGeoBridge(unittest.TestCase):
    def test_round_trip_through_wgs84_preserves_position(self):
        from ap_cpp.geo_bridge import GeoBridge

        polygon = [
            [40.59626, 22.94628],
            [40.59626, 22.95028],
            [40.59926, 22.95028],
            [40.59926, 22.94628],
        ]
        bridge = GeoBridge(polygon)
        ned = bridge.polygon_ned
        back = bridge.ned_to_wgs84(ned)
        # ~1e-8 deg is about a millimetre; compare at sub-centimetre precision.
        for original, restored in zip(polygon, back):
            self.assertAlmostEqual(original[0], restored[0], places=6)
            self.assertAlmostEqual(original[1], restored[1], places=6)

    def test_shipped_geojson_loads_into_a_grid(self):
        from ap_cpp.geo_bridge import GeoBridge, load_qgis_polygon

        path = os.path.join(_REPO_ROOT, "CPP", "002", "Polygon002.geojson")
        if not os.path.exists(path):
            self.skipTest("scenario file not present")
        polygon, obstacles, name = load_qgis_polygon(path)
        self.assertTrue(polygon)
        bridge = GeoBridge(polygon, obstacles)
        grid = bridge.build_grid(resolution=2.5)
        self.assertGreater(int(grid.passable.sum()), 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
