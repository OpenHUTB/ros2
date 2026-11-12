"""端到端训练恢复与相机输入的回归测试。"""

import os
import csv
import io
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from unittest.mock import patch
from types import SimpleNamespace

import numpy as np
import torch

SCRIPTS = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'scripts')
sys.path.insert(0, SCRIPTS)

try:
    import airsim  # noqa: F401
except ImportError:
    # 单测不连接模拟器；仓库提供的 AirSim PythonClient 可在实机测试时加入 PYTHONPATH。
    sys.modules['airsim'] = SimpleNamespace(
        ImageRequest=lambda *args: args,
        ImageType=SimpleNamespace(Scene=0, DepthPlanar=1))

from camera_node import bgr_frame  # noqa: E402
from dqn_agent import ImageDQNAgent  # noqa: E402
from train_dqn_e2e import End2EndTrainer  # noqa: E402
from train_dqn_nav import DQNTrainer  # noqa: E402


class ImageResponse(object):
    def __init__(self, data, width=2, height=2):
        self.image_data_uint8 = bytes(data)
        self.width = width
        self.height = height


class E2ERegressionTest(unittest.TestCase):
    def test_camera_accepts_three_and_four_channels(self):
        bgr = bytes(range(12))
        bgra = bytes([0, 1, 2, 255, 3, 4, 5, 255,
                      6, 7, 8, 255, 9, 10, 11, 255])
        expected = np.frombuffer(bgr, dtype=np.uint8).reshape(2, 2, 3)
        np.testing.assert_array_equal(bgr_frame(ImageResponse(bgr)), expected)
        np.testing.assert_array_equal(bgr_frame(ImageResponse(bgra)), expected)
        with self.assertRaises(ValueError):
            bgr_frame(ImageResponse(b'bad'))

    def test_resume_only_loads_image_agent(self):
        with tempfile.TemporaryDirectory() as directory:
            checkpoint = os.path.join(directory, 'e2e.pt')
            ImageDQNAgent().save(checkpoint)
            with patch.object(DQNTrainer, '__init__', return_value=None) as parent:
                trainer = End2EndTrainer('127.0.0.1', 'Drone_1', 1,
                                        resume=checkpoint)
            self.assertIsInstance(trainer.agent, ImageDQNAgent)
            self.assertIsNone(parent.call_args.args[3])

    def test_checkpoint_restores_optimizer_and_steps(self):
        with tempfile.TemporaryDirectory() as directory:
            checkpoint = os.path.join(directory, 'e2e.pt')
            agent = ImageDQNAgent()
            agent.steps = 37
            agent.optimizer.param_groups[0]['lr'] = 0.001
            agent.save(checkpoint)
            resumed = ImageDQNAgent()
            resumed.load(checkpoint)
            self.assertEqual(resumed.steps, 37)
            self.assertEqual(resumed.optimizer.param_groups[0]['lr'], 0.001)
            compact = os.path.join(directory, 'best.pt')
            agent.save(compact, include_optimizer=False)
            stored = torch.load(compact, map_location='cpu')
            self.assertNotIn('optimizer', stored)
            compact_agent = ImageDQNAgent()
            compact_agent.load(compact)
            self.assertEqual(compact_agent.steps, 37)
            self.assertLess(os.path.getsize(compact), os.path.getsize(checkpoint))

    def test_one_camera_request_provides_image_and_near_obstacle(self):
        rgb = ImageResponse(bytes(range(27)), width=3, height=3)
        depth = SimpleNamespace(image_data_float=[3.0, 3.0, 3.0,
                                                  3.0, 0.4, 3.0,
                                                  3.0, 3.0, 3.0],
                                width=3, height=3)
        trainer = End2EndTrainer.__new__(End2EndTrainer)
        trainer.vehicle = 'Drone_1'
        trainer.client = SimpleNamespace(simGetImages=lambda *args, **kwargs:
                                         [rgb, depth])
        with patch.object(trainer.client, 'simGetImages',
                          wraps=trainer.client.simGetImages) as camera_rpc:
            image = trainer.read_image()
        self.assertEqual(image.shape, (84, 84))
        self.assertEqual(camera_rpc.call_count, 1)
        self.assertAlmostEqual(trainer.obstacle_distance(None), 0.4, places=5)

    def test_episode_skips_lidar_but_keeps_near_obstacle_stop(self):
        trainer = End2EndTrainer.__new__(End2EndTrainer)
        st = {'pos': (0.0, 0.0, -4.0), 'yaw': 0.0,
              'vel': (0.0, 0.0, 0.0)}
        trainer.goal = (0.0, 10.0, -4.0)
        trainer.max_steps = 1
        trainer.reset_episode = lambda: st
        trainer.read_state = lambda: st
        trainer.read_lidar = lambda: self.fail('端到端回合不应请求雷达')
        trainer.send_action = lambda action: None
        trainer.has_collided = lambda: False
        trainer.build = lambda *args: setattr(
            trainer, 'latest_obstacle', 0.4) or (np.zeros((84, 84)),
                                                 np.zeros(7))
        trainer.agent = SimpleNamespace(act=lambda *args: 0,
                                        remember=lambda *args: None,
                                        can_learn=lambda: False)
        result = trainer.run_episode(0.0, use_lidar=False)
        self.assertTrue(result['near_miss'])

    def test_training_delivers_log_best_and_last_after_twenty_episodes(self):
        trainer = End2EndTrainer.__new__(End2EndTrainer)
        trainer.episodes = 20
        trainer.run_episode = lambda *args, **kwargs: {
            'steps': 1, 'reward': 0.0, 'distance': 10.0, 'success': False,
            'collision': False, 'near_miss': False, 'loss': 0.0}
        save_count = [0]

        def save(path, include_optimizer=True):
            save_count[0] += 1
            with open(path, 'wb') as f:
                f.write(str(save_count[0]).encode('ascii'))

        trainer.agent = SimpleNamespace(save=save)
        with tempfile.TemporaryDirectory() as directory, patch(
                'train_dqn_e2e.MODEL_DIR', directory):
            with redirect_stdout(io.StringIO()):
                trainer.train()
            with open(os.path.join(directory, 'train_e2e_log.csv')) as f:
                rows = list(csv.reader(f))
            self.assertEqual(len(rows), 21)
            self.assertEqual(len(rows[0]), 9)
            with open(os.path.join(directory, 'dqn_e2e_best.pt'), 'rb') as f:
                best = f.read()
            with open(os.path.join(directory, 'dqn_e2e.pt'), 'rb') as f:
                self.assertEqual(f.read(), best)
            self.assertTrue(os.path.exists(
                os.path.join(directory, 'dqn_e2e_last.pt')))


if __name__ == '__main__':
    unittest.main()
