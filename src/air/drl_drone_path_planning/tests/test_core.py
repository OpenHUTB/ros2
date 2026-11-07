"""Unit tests for the shared math, environment and policy inference.

Run with::

    python3 -m unittest discover -s tests -v

These tests are NumPy/Gymnasium only -- they do not require ROS, PyTorch or a
simulator, so they can run on any development machine.

Developer: 汤湘粤 (学号: 2535030067)
"""

from __future__ import annotations

import math
import os
import sys
import unittest

import numpy as np

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
SCRIPTS_DIR = os.path.join(os.path.dirname(TESTS_DIR), "scripts")
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)

from baseline import baseline_action  # noqa: E402
from common import (  # noqa: E402
    action_to_velocity,
    apply_altitude_hold,
    build_observation,
    front_min_distance,
    histogram_from_ranges,
    load_config,
    observation_dim,
    quat_to_rpy,
    ray_circle_distance,
    raycast_histogram,
    rpy_to_quat,
    velocity_to_action,
    world_to_body,
)
from drone_env import DronePathEnv  # noqa: E402
from policy import NumpyActor, save_actor_npz  # noqa: E402

CONFIG_PATH = os.path.join(
    os.path.dirname(TESTS_DIR), "config", "drl_nav.yaml"
)


class MathTest(unittest.TestCase):
    """Quaternion, geometry and action-scaling helpers."""

    def test_quaternion_round_trip(self) -> None:
        rpy = np.array([0.2, -0.3, 1.1])
        recovered = quat_to_rpy(rpy_to_quat(rpy))
        np.testing.assert_allclose(recovered, rpy, atol=1e-9)

    def test_ray_circle_hit_and_miss(self) -> None:
        distance = ray_circle_distance((0, 0), (1, 0), (5, 0), 0.5)
        self.assertAlmostEqual(distance, 4.5, places=6)
        self.assertTrue(math.isinf(
            ray_circle_distance((0, 0), (0, 1), (5, 0), 0.5)
        ))

    def test_histogram_semantics(self) -> None:
        cfg = load_config(CONFIG_PATH)
        sectors = int(cfg["observation"]["sectors"])
        beams = 360
        ranges = np.full(beams, 50.0)
        ranges[180] = 3.0  # beam at 0 rad, i.e. straight ahead
        histogram = histogram_from_ranges(
            ranges, -math.pi, 2.0 * math.pi / beams, 0.1, 50.0,
            sectors, 12.0,
        )
        self.assertAlmostEqual(float(histogram[0]), 3.0 / 12.0, places=5)
        self.assertAlmostEqual(float(histogram[sectors // 2]), 1.0, places=6)
        self.assertAlmostEqual(
            front_min_distance(histogram, cfg), 3.0, places=4
        )

    def test_raycast_histogram(self) -> None:
        histogram = raycast_histogram(
            position=(0, 0, 2), yaw=0.0,
            obstacles=[(5.0, 0.0, 0.5, 4.0)],
            sectors=16, max_range=12.0,
        )
        self.assertAlmostEqual(float(histogram[0]), 4.5 / 12.0, places=5)
        self.assertAlmostEqual(float(histogram[8]), 1.0, places=6)

    def test_altitude_hold(self) -> None:
        cfg = load_config(CONFIG_PATH)
        action = np.array([0.0, 0.0, 1.0, 0.0])
        held = apply_altitude_hold(action, (0, 0, 2.0), (6, 0, 2.0), cfg)
        self.assertAlmostEqual(float(held[2]), 0.1, places=6)
        climbed = apply_altitude_hold(action, (0, 0, 2.0), (6, 0, 3.0), cfg)
        self.assertAlmostEqual(float(climbed[2]), 1.0, places=6)

    def test_action_scaling_round_trip(self) -> None:
        cfg = load_config(CONFIG_PATH)
        action = np.array([0.5, -0.25, 0.75, -1.0])
        velocity = action_to_velocity(action, cfg)
        recovered = velocity_to_action(velocity, cfg)
        np.testing.assert_allclose(recovered, action, atol=1e-9)


class EnvironmentTest(unittest.TestCase):
    """Environment contract: shapes, bounds and episode termination."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.cfg = load_config(CONFIG_PATH)
        cls.env = DronePathEnv(cls.cfg)

    def test_spaces(self) -> None:
        self.assertEqual(self.env.observation_space.shape[0],
                         observation_dim(self.cfg))
        self.assertEqual(self.env.action_space.shape, (4,))

    def test_reset_and_step_are_bounded(self) -> None:
        observation, info = self.env.reset(seed=3)
        self.assertTrue(self.env.observation_space.contains(observation))
        self.assertIn("distance", info)
        for _ in range(20):
            action = np.random.default_rng(7).uniform(-1.0, 1.0, 4)
            observation, reward, terminated, truncated, info = self.env.step(action)
            self.assertTrue(self.env.observation_space.contains(observation))
            self.assertTrue(np.isfinite(reward))
            self.assertIn("success", info)
            if terminated or truncated:
                break

    def test_baseline_observation_is_bounded(self) -> None:
        observation, _ = self.env.reset(seed=0)
        action = baseline_action(observation, self.cfg)
        self.assertEqual(action.shape, (4,))
        self.assertTrue(np.all(action >= -1.0) and np.all(action <= 1.0))

    def test_observation_matches_manual_assembly(self) -> None:
        observation, _ = self.env.reset(seed=0)
        body_velocity = world_to_body(self.env.velocity_world, self.env.yaw)
        manual = build_observation(
            self.env.position,
            self.env.rpy,
            body_velocity,
            self.env.angular_velocity_body,
            self.env._goal(),
            self.env.histogram,
            self.cfg,
        )
        np.testing.assert_allclose(observation, manual, atol=1e-6)
        self.assertEqual(observation.dtype, np.float32)


class PolicyTest(unittest.TestCase):
    """NumPy actor export/inference contract."""

    def test_numpy_actor_forward(self) -> None:
        rng = np.random.default_rng(0)
        weights = [
            rng.normal(0.0, 0.3, size=(8, 4)),
            rng.normal(0.0, 0.3, size=(4, 8)),
        ]
        biases = [np.zeros(8), np.zeros(4)]
        path = os.path.join(TESTS_DIR, "_tmp_actor.npz")
        try:
            save_actor_npz(path, weights, biases, obs_dim=4, action_dim=4)
            actor = NumpyActor(path)
            observation = rng.normal(size=4)
            action = actor.predict(observation, deterministic=True)
            hidden = np.tanh(weights[0] @ observation + biases[0])
            expected = np.tanh(weights[1] @ hidden + biases[1])
            np.testing.assert_allclose(action, expected, atol=1e-6)
            self.assertTrue(np.all(action >= -1.0) and np.all(action <= 1.0))
        finally:
            if os.path.isfile(path):
                os.remove(path)

    def test_ppo_clip_and_sac_tanh_modes(self) -> None:
        """PPO clips the Gaussian mean; SAC squashes it with tanh."""
        weights = [np.zeros((4, 4))]
        biases = [np.array([2.0, -2.0, 0.5, 0.0])]
        path = os.path.join(TESTS_DIR, "_tmp_actor_squash.npz")
        try:
            save_actor_npz(path, weights, biases, obs_dim=4, action_dim=4,
                           squash=False)
            clipped = NumpyActor(path).predict(np.zeros(4))
            np.testing.assert_allclose(clipped, [1.0, -1.0, 0.5, 0.0], atol=1e-6)

            save_actor_npz(path, weights, biases, obs_dim=4, action_dim=4,
                           squash=True)
            squashed = NumpyActor(path).predict(np.zeros(4))
            np.testing.assert_allclose(squashed, np.tanh(biases[0]), atol=1e-6)
        finally:
            if os.path.isfile(path):
                os.remove(path)

    def test_actor_rejects_wrong_observation_size(self) -> None:
        rng = np.random.default_rng(1)
        path = os.path.join(TESTS_DIR, "_tmp_actor_bad.npz")
        try:
            save_actor_npz(
                path,
                [rng.normal(size=(4, 4))],
                [np.zeros(4)],
                obs_dim=4,
                action_dim=4,
            )  # (out, in) = (4, 4)
            actor = NumpyActor(path)
            with self.assertRaises(ValueError):
                actor.predict(np.zeros(3))
        finally:
            if os.path.isfile(path):
                os.remove(path)


if __name__ == "__main__":
    unittest.main()
