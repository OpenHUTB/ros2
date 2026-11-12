"""监督学习、独立回合切分和真实接触传感器的回归测试。"""

import tempfile
import unittest
from pathlib import Path

import numpy as np

from mujoco_muscle_gripper.neural import (
    GripEnvironment, MLP, collect_force_data, force_rollout, split_episodes, teacher_delta,
)


class TestNetwork(unittest.TestCase):
    def test_learns_nonlinear_mapping_and_reloads_exactly(self):
        rng = np.random.default_rng(13)
        x = rng.uniform(-1, 1, (240, 2))
        y = (0.4 * np.sin(x[:, 0]) + 0.3 * x[:, 1] ** 2)
        model = MLP(2, hidden=16, seed=5)
        before = float(np.mean((model.predict(x[200:]).ravel() - y[200:]) ** 2))
        model.fit(x[:180], y[:180], (x[180:200], y[180:200]), epochs=350)
        after = float(np.mean((model.predict(x[200:]).ravel() - y[200:]) ** 2))
        self.assertLess(after, 0.003)
        self.assertLess(after, before * 0.1)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "weights.npz"
            model.save(path)
            np.testing.assert_array_equal(model.predict(x), MLP.load(path).predict(x))

    def test_split_has_no_episode_overlap(self):
        groups = np.repeat(np.arange(15), 5)
        parts = split_episodes(groups)
        seen = [set(groups[idx]) for idx in parts]
        self.assertFalse(seen[0] & seen[1] or seen[0] & seen[2] or seen[1] & seen[2])
        self.assertEqual(set(np.concatenate(parts)), set(range(len(groups))))

    def test_contact_measurement_changes_after_real_closure(self):
        env = GripEnvironment()
        try:
            self.assertAlmostEqual(env.observe()[0], 0.0)
            for _ in range(100):
                env.step(0.7)
            force, travel, _, contacts = env.observe()
            self.assertGreater(force, 1)
            self.assertGreater(travel, 0.005)
            self.assertGreater(contacts, 0)
        finally:
            env.close()

    def test_teacher_direction_and_clipping(self):
        self.assertGreater(teacher_delta(8, 2), 0)
        self.assertLess(teacher_delta(2, 8), 0)
        self.assertLessEqual(abs(teacher_delta(1000, 0)), 0.02)

    def test_trained_network_tracks_actual_contact_force(self):
        x, y, episodes = collect_force_data(episodes=8, seed=11)
        train, validation, test = split_episodes(episodes, seed=11)
        net = MLP(x.shape[1], hidden=24, seed=11)
        net.fit(x[train], y[train], (x[validation], y[validation]), epochs=160, seed=11)
        self.assertLess(float(np.mean((net.predict(x[test]).ravel() - y[test]) ** 2)), 0.00002)
        result, _ = force_rollout(4, network=net)
        self.assertLess(result["steady_mae_n"], 0.3)
        self.assertGreater(result["steady_force_n"], 3.7)


if __name__ == "__main__":
    unittest.main()
