"""无 ROS 的离线闭环评测测试（不依赖 rclpy / mujoco）。"""
import math
import unittest

import numpy as np

from rov_mujoco.nn_trajectory_controller import LOSTrajectoryController, NNTrajectoryController
from rov_mujoco.offline_loop import FirstOrderROV, run_episode
from rov_mujoco.trajectory_generator import TrajectoryGenerator

from rov_mujoco.offline_loop import check_gate


class TrajectoryWithoutRosTests(unittest.TestCase):
    def test_generator_imports_and_shapes(self):
        gen = TrajectoryGenerator(trajectory_type='3d_helix', radius=2.2, base_depth=-1.5)
        state = gen.get_state(0.0)
        self.assertEqual(state['pos'].shape, (3,))
        self.assertAlmostEqual(state['pos'][0], 2.2, places=6)
        self.assertAlmostEqual(state['pos'][1], 0.0, places=6)

    def test_lawnmower_is_bounded(self):
        gen = TrajectoryGenerator(trajectory_type='lawnmower', base_depth=-1.5)
        positions = np.array([gen.get_state(t)['pos'] for t in np.linspace(0, 40, 200)])
        self.assertTrue(np.all(np.abs(positions[:, :2]) <= 1.76))
        self.assertTrue(np.allclose(positions[:, 2], -1.5))


class OfflineLoopTests(unittest.TestCase):
    def test_gate_only_flags_combinations_over_threshold(self):
        summary = {'3d_helix/nn': {'rmse': 0.133},
                   '3d_helix/los': {'rmse': 0.112},
                   'config': {'duration': 60.0}}
        self.assertEqual([key for key, _ in check_gate(summary, 0.12)], ['3d_helix/nn'])
        self.assertEqual(check_gate(summary, 0.2), [])

    def test_gate_rejects_non_finite_rmse(self):
        """发散到 NaN/inf 的组合必须算失败：nan > x 恒为 False，会被静默放行"""
        for bad in (float('nan'), float('inf'), float('-inf')):
            summary = {'3d_helix/nn': {'rmse': bad}, 'config': {'duration': 60.0}}
            failures = check_gate(summary, 10.0)
            self.assertEqual(len(failures), 1, '非有限 RMSE %.1f 应被门禁拦下' % bad)

    def test_vehicle_is_stable_without_force(self):
        vehicle = FirstOrderROV()
        state = vehicle.initial_state((0.0, 0.0, 0.0))
        state['vel'] = np.array([1.0, 0.0, 0.0])
        for _ in range(800):
            state = vehicle.step(state, {'fx': 0.0, 'fy': 0.0, 'fz': 0.0, 'tau_z': 0.0}, 0.02)
        self.assertLess(abs(state['vel'][0]), 0.2, '阻尼应让速度衰减')

    def test_los_converges_on_helix(self):
        episode = run_episode(LOSTrajectoryController(), '3d_helix', duration=40,
                              start_offset=(1.0, -0.8, 0.5))
        self.assertLess(episode['final_error'], 0.10)
        for key in ('rmse', 'max_error', 'yaw_rmse', 'effort'):
            self.assertTrue(math.isfinite(episode[key]))

    def test_nn_controller_also_tracks(self):
        episode = run_episode(NNTrajectoryController(), '3d_helix', duration=40,
                              start_offset=(1.0, -0.8, 0.5))
        self.assertLess(episode['final_error'], 0.20)
        self.assertGreater(episode['effort'], 0.0)


if __name__ == '__main__':
    unittest.main()
