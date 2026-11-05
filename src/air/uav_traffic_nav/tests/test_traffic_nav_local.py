#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""uav_traffic_nav 纯 Python 单元测试（不需要 ROS / 仿真器）.

覆盖：观测与动作接口、交通特征、TTC、碰撞/成功/越界判定、课程难度、
基线控制器、车辆流转换、评估指标。
"""
from __future__ import annotations

import json
import math
import os
import sys
import tempfile
import time
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.join(os.path.dirname(HERE), "scripts")
sys.path.insert(0, SCRIPTS)

import baselines  # noqa: E402
import feed_utils  # noqa: E402
import policy  # noqa: E402
import traffic_env  # noqa: E402
import eval as eval_mod  # noqa: E402


class PolicyTests(unittest.TestCase):
    def test_observation_dims(self):
        self.assertEqual(policy.observation_dim(True), 34)
        self.assertEqual(policy.observation_dim(False), 22)
        self.assertEqual(policy.OBS_DIM, policy.N_SECTORS + 3 + 3 + policy.N_TRAFFIC * 4)

    def test_build_observation_scales_goal_and_velocity(self):
        hist = np.ones(policy.N_SECTORS, dtype=np.float32)
        obs = policy.build_observation(hist, [8.0, -4.0, 0.0], [3.0, 0.0, 1.5],
                                       traffic=np.zeros(12, dtype=np.float32))
        self.assertEqual(obs.shape, (34,))
        self.assertAlmostEqual(float(obs[16]), 1.0)     # 8 m / GOAL_SCALE
        self.assertAlmostEqual(float(obs[17]), -0.5)
        self.assertAlmostEqual(float(obs[19]), 1.0)     # 3 m/s / MAX_HORIZ_SPEED
        self.assertAlmostEqual(float(obs[21]), 1.0)     # 1.5 m/s / MAX_VERT_SPEED
        self.assertTrue(np.all(np.isfinite(obs)))

    def test_build_observation_without_traffic_is_22(self):
        obs = policy.build_observation(np.ones(policy.N_SECTORS), [1, 2, 3], [0, 0, 0],
                                       include_traffic=False)
        self.assertEqual(obs.shape, (22,))

    def test_traffic_features_sorted_by_distance_and_padded(self):
        cars = [(10.0, 0.0, 0.0, 0.0), (3.0, 0.0, 1.0, 0.0)]
        feat = policy.traffic_features(cars, drone_xy=(0, 0), drone_vel=(0, 0))
        self.assertEqual(feat.shape, (12,))
        # 最近的车（3 m）排在第一位
        self.assertAlmostEqual(float(feat[0] * policy.TRAFFIC_POS_SCALE), 3.0)
        # 第二辆车是 10 m
        self.assertAlmostEqual(float(feat[4] * policy.TRAFFIC_POS_SCALE), 10.0, places=4)
        # 第三位补零
        self.assertEqual(float(np.abs(feat[8:]).sum()), 0.0)

    def test_traffic_features_clip_large_values(self):
        feat = policy.traffic_features([(500.0, -500.0, 99.0, -99.0)])
        self.assertTrue(np.all(np.abs(feat) <= 1.0))

    def test_time_to_collision_head_on(self):
        ttc = policy.time_to_collision((0, 0), (0, 0), (5, 0), (-2, 0), max_time=5.0)
        self.assertAlmostEqual(ttc, 2.5, places=6)

    def test_time_to_collision_receding_is_none(self):
        self.assertIsNone(policy.time_to_collision((0, 0), (0, 0), (5, 0), (2, 0)))

    def test_time_to_collision_beyond_horizon_is_none(self):
        self.assertIsNone(policy.time_to_collision((0, 0), (0, 0), (100, 0), (-1, 0),
                                                   max_time=2.0))

    def test_action_velocity_roundtrip(self):
        action = policy.velocity_to_action(3.0, -3.0, 1.5, 1.0)
        self.assertTrue(np.allclose(action, [1.0, -1.0, 1.0, 1.0]))
        vx, vy, vz, wz = policy.action_to_velocity(action)
        self.assertAlmostEqual(vx, policy.MAX_HORIZ_SPEED)
        self.assertAlmostEqual(vy, -policy.MAX_HORIZ_SPEED)

    def test_mlp_policy_forward_is_bounded(self):
        weights = {
            "n_layers": 1,
            "h0_w": np.zeros((4, 34), dtype=np.float32),
            "h0_b": np.zeros(4, dtype=np.float32),
            "out_w": np.ones((4, 4), dtype=np.float32) * 10.0,
            "out_b": np.zeros(4, dtype=np.float32),
        }
        net = policy.MlpPolicy(weights)
        out = net.forward(np.zeros(34, dtype=np.float32))
        self.assertEqual(out.shape, (4,))
        self.assertTrue(np.all(np.abs(out) <= 1.0))


class EnvironmentTests(unittest.TestCase):
    def make(self, **kwargs):
        env = traffic_env.TrafficNavEnv(seed=5, **kwargs)
        env.set_difficulty(1.0)
        return env

    def test_reset_obs_shape(self):
        env = self.make()
        obs, info = env.reset(seed=1)
        self.assertEqual(obs.shape, (34,))
        self.assertEqual(info["cars"], len(env.cars))
        self.assertEqual(len(env.cars), len(env.lane_ys) * env._cars_per_lane())

    def test_reset_without_traffic_features(self):
        env = self.make(include_traffic=False)
        obs, _ = env.reset(seed=1)
        self.assertEqual(obs.shape, (22,))

    def test_car_collision_terminates(self):
        env = self.make()
        env.reset(seed=2)
        env.cars = [{"x": env.pos[0], "y": env.pos[1], "vx": 0.0, "vy": 0.0}]
        _obs, reward, terminated, _truncated, info = env.step(np.zeros(4))
        self.assertTrue(terminated)
        self.assertEqual(info["crash_reason"], "撞车辆")
        self.assertLess(reward, 0.0)

    def test_static_obstacle_collision_terminates(self):
        env = self.make(static_obstacles=1)
        env.reset(seed=3)
        env.cars = []
        env.obstacles_xy = [(env.pos[0], env.pos[1])]
        _obs, _reward, terminated, _truncated, info = env.step(np.zeros(4))
        self.assertTrue(terminated)
        self.assertEqual(info["crash_reason"], "撞静态障碍")

    def test_goal_reached_is_success(self):
        env = self.make()
        env.reset(seed=4)
        env.cars = []
        env.obstacles_xy = []
        env.goal = env.pos + np.array([0.1, 0.0, 0.0])
        _obs, reward, terminated, _truncated, info = env.step(np.zeros(4))
        self.assertTrue(terminated)
        self.assertTrue(info["success"])
        self.assertGreater(reward, 0.0)

    def test_out_of_bounds_terminates(self):
        env = self.make()
        env.reset(seed=6)
        env.cars = []
        env.obstacles_xy = []
        env.pos = np.array([env.bounds + 1.0, 0.0, env.init_z])
        env.goal = env.pos + np.array([0.0, 5.0, 0.0])
        _obs, _reward, terminated, _truncated, info = env.step(np.zeros(4))
        self.assertTrue(terminated)
        self.assertEqual(info["crash_reason"], "飞出场地")

    def test_cars_move_by_their_velocity(self):
        env = self.make()
        env.reset(seed=7)
        env.cars = [{"x": 0.0, "y": 0.0, "vx": 5.0, "vy": 0.0}]
        env.step(np.zeros(4))
        self.assertAlmostEqual(env.cars[0]["x"], 0.5, places=6)

    def test_difficulty_scales_vehicle_count(self):
        env = traffic_env.TrafficNavEnv(seed=5, cars_per_lane=1)
        env.max_cars_per_lane = 3
        env.set_difficulty(0.0)
        slow = env._cars_per_lane()
        env.set_difficulty(1.0)
        fast = env._cars_per_lane()
        self.assertLess(slow, fast)

    def test_ttc_penalty_reduces_reward(self):
        env = self.make()
        env.reset(seed=8)
        env.obstacles_xy = []
        env.cars = []
        base_reward = env.step(np.zeros(4))[1]
        env.reset(seed=8)
        env.obstacles_xy = []
        env.cars = [{"x": env.pos[0] + 2.0, "y": env.pos[1], "vx": -5.0, "vy": 0.0}]
        risky_reward = env.step(np.zeros(4))[1]
        self.assertLess(risky_reward, base_reward - 0.1)

    def test_max_steps_truncates(self):
        env = self.make(max_steps=3)
        env.reset(seed=9)
        env.cars = []
        env.obstacles_xy = []
        env.goal = env.pos + np.array([50.0, 50.0, 0.0])
        truncated = False
        for _ in range(3):
            _obs, _reward, _terminated, truncated, _info = env.step(np.zeros(4))
        self.assertTrue(truncated)

    def test_same_seed_gives_same_reset(self):
        a = self.make()
        b = self.make()
        obs_a, _ = a.reset(seed=11)
        obs_b, _ = b.reset(seed=11)
        self.assertTrue(np.allclose(obs_a, obs_b))
        self.assertTrue(np.allclose(a.pos, b.pos))
        self.assertEqual(len(a.cars), len(b.cars))


class BaselineTests(unittest.TestCase):
    def setUp(self):
        self.env = traffic_env.TrafficNavEnv(seed=12, static_obstacles=0)
        self.env.reset(seed=12)
        self.env.obstacles_xy = []
        self.env.cars = []
        self.env.goal = self.env.pos + np.array([5.0, 0.0, 0.0])

    def test_greedy_points_towards_goal(self):
        action = baselines.greedy_action(self.env)
        vx = action[0] * policy.MAX_HORIZ_SPEED
        self.assertGreater(vx, 2.5)

    def test_vo_avoids_car_directly_ahead(self):
        self.env.cars = [{"x": self.env.pos[0] + 2.0, "y": self.env.pos[1],
                          "vx": -3.0, "vy": 0.0}]
        action = baselines.vo_action(self.env)
        self.assertTrue(np.all(np.abs(action) <= 1.0))
        # 不应该仍然全速直冲（沿 +x）——要么减速，要么侧移
        speed = math.hypot(action[0], action[1]) * policy.MAX_HORIZ_SPEED
        lateral = abs(action[1]) * policy.MAX_HORIZ_SPEED
        self.assertTrue(speed < policy.MAX_HORIZ_SPEED * 0.99 or lateral > 0.3)

    def test_vo_returns_bounded_action_without_traffic(self):
        action = baselines.vo_action(self.env)
        self.assertTrue(np.all(np.abs(action) <= 1.0))


class FeedTests(unittest.TestCase):
    def test_to_vehicles_applies_offset_and_age(self):
        live = {"time": 1000.0, "vehicles": [{"x": 1.0, "y": 2.0, "vx": 3.0, "vy": 4.0}]}
        out = feed_utils.to_vehicles(live, (10.0, -5.0), max_age_s=1.0, now=1000.5)
        self.assertEqual(len(out), 1)
        self.assertAlmostEqual(out[0]["x"], 11.0)
        self.assertAlmostEqual(out[0]["y"], -3.0)
        self.assertAlmostEqual(out[0]["vx"], 3.0)

    def test_to_vehicles_drops_stale_and_broken(self):
        live = {"time": 1000.0, "vehicles": [{"x": 1.0, "y": 2.0}]}
        self.assertEqual(feed_utils.to_vehicles(live, (0, 0), 1.0, now=1005.0), [])
        self.assertEqual(feed_utils.to_vehicles({"vehicles": []}, (0, 0), 1.0), [])
        self.assertEqual(feed_utils.to_vehicles("bad", (0, 0), 1.0), [])

    def test_read_live_retries_permission_error(self):
        calls = {"n": 0}

        def fake_sleep(_seconds):
            return None

        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "live.json")
            with open(path, "w", encoding="utf-8") as fh:
                json.dump({"time": time.time(), "vehicles": []}, fh)
            original = feed_utils.read_live
            # 第一次 open 抛 PermissionError，第二次正常
            real_open = open
            state = {"first": True}

            def flaky_open(*args, **kwargs):
                if state["first"]:
                    state["first"] = False
                    raise PermissionError()
                return real_open(*args, **kwargs)

            import builtins
            builtins.open = flaky_open
            try:
                data = feed_utils.read_live(path, retries=3, sleep=fake_sleep)
            finally:
                builtins.open = real_open
            self.assertIsNotNone(data)
            self.assertIs(feed_utils.read_live, original)


class EvalTests(unittest.TestCase):
    def test_evaluate_returns_metrics(self):
        metrics = eval_mod.evaluate("greedy", lambda env, obs: baselines.greedy_action(env),
                                    episodes=2, difficulty=0.0, seed=3)
        self.assertEqual(metrics["episodes"], 2)
        self.assertGreaterEqual(metrics["success_rate"], 0.0)
        self.assertLessEqual(metrics["success_rate"], 1.0)
        self.assertEqual(metrics["success_rate"] + metrics["crash_rate"]
                         + metrics["timeout_rate"], 1.0)


if __name__ == "__main__":
    unittest.main()
