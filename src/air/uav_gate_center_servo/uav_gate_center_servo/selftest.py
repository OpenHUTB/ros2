#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""自检：不需要 ROS；无 GPU 时深度检查跳过。"""
from __future__ import annotations

import os
import tempfile
import time
from typing import List

import numpy as np

from .core.depth import depth_to_u8, downsample_depth, valid_fraction
from .core.env import EnvConfig, GateServoEnv, GateSpec
from .core.geometry import (CAM_OFFSET_BODY, camera_label, project_label)
from .core.mlp import TinyMLP
from .core.servo import GateServo, ServoConfig

__all__ = ["run_selftest"]


class _Report:
    def __init__(self):
        self.n = self.ok = 0
        self.fail: List[str] = []
        self.skip: List[str] = []

    def check(self, name: str, ok: bool, note: str = "") -> None:
        self.n += 1
        if ok:
            self.ok += 1
        else:
            self.fail.append(name)
        print("  [%s] %-30s %s" % ("PASS" if ok else "FAIL", name, note))

    def skip_check(self, name: str, note: str = "") -> None:
        self.n += 1
        self.skip.append(name)
        print("  [SKIP] %-30s %s" % (name, note))


def _run_episode(env, action=None, max_steps=300, seed=0, reset_kwargs=None):
    obs = env.reset(seed=seed, **(reset_kwargs or {}))
    done, info, k = False, {}, 0
    while not done and k < max_steps:
        a = env.oracle_command() if action is None else action
        obs, done, info = env.step(a)
        k += 1
    return obs, info


def run_selftest() -> int:
    rep = _Report()
    t0 = time.time()
    rng = np.random.default_rng(0)
    print("=" * 72)
    print("  uav_gate_center_servo 自检")
    print("=" * 72)

    # 1-4 depth
    d = np.full((48, 64), 2.0, np.float32)
    q = downsample_depth(d)
    rep.check("深度降采样 48x64 -> 12x16", q.shape == (12, 16) and q.dtype == np.float32)
    rep.check("块平均保留常值", abs(float(q.mean()) - 2.0) < 1e-6, "mean=%.3f" % q.mean())
    d2 = np.zeros((2, 2), np.float32)
    d2[0, 0] = 4.0
    rep.check("无效像素不拉低均值", abs(float(downsample_depth(d2, (1, 1))[0, 0]) - 4.0) < 1e-6)
    rep.check("有效像素比例", abs(valid_fraction(d2) - 0.25) < 1e-9,
              "%.2f" % valid_fraction(d2))

    # 5-8 labels
    q0 = np.array([1.0, 0.0, 0.0, 0.0])
    l_fwd = camera_label(np.array([5.0, 0.0, 2.0]), np.array([0.0, 0.0, 2.0]), q0)
    rep.check("标签前向符号", 4.8 < l_fwd[0] < 5.0, "forward=%.3f" % l_fwd[0])
    l_left = camera_label(np.array([5.0, 1.0, 2.0]), np.array([0.0, 0.0, 2.0]), q0)
    rep.check("标签左向符号", l_left[1] > 0.9, "left=%.3f" % l_left[1])
    l_up = camera_label(np.array([5.0, 0.0, 4.0]), np.array([0.0, 0.0, 2.0]), q0)
    rep.check("标签上向符号", l_up[2] > 1.9, "up=%.3f" % l_up[2])
    l_off = camera_label(np.array([1.075, 0.0, 0.012]), np.zeros(3), q0)
    rep.check("相机安装偏移", abs(l_off[0] - 1.0) < 1e-6,
              "forward=%.4f" % l_off[0])

    # 9-10 servo
    servo = GateServo(ServoConfig())
    cmd = servo.command(np.array([4.0, 1.0, -1.0]))
    rep.check("伺服符号与限幅",
              cmd[0] > 0 and cmd[1] > 0 and cmd[2] < 0
              and abs(cmd[1]) <= 1.8 and abs(cmd[2]) <= 0.9,
              "cmd=%s" % np.round(cmd, 3).tolist())
    zero = servo.command(np.array([4.0, 1.0, 0.0]), valid_frac=0.0)
    rep.check("失效/低有效像素减速", np.allclose(zero, 0.0), "cmd=%s" % zero.tolist())

    # 11-15 env
    env = GateServoEnv(EnvConfig())
    rep.check("单门几何", abs(env.gate.x - 5.0) < 1e-6 and env.gate.half == 1.1,
              "center=(%.3f, %.3f, %.3f)" % tuple(env.gate.center))
    ranges_ok = True
    for seed in range(12):
        o = env.reset(seed=seed)
        p = o["pos"]
        dist = env.gate.x - p[0]
        ranges_ok &= 1.99 <= dist <= 6.01
        ranges_ok &= abs(p[1] - env.gate.y) <= 1.51
        ranges_ok &= abs(p[2] - env.gate.z) <= 0.81
        ranges_ok &= abs(float(o["yaw"])) < 0.05
    rep.check("reset 范围与航向", bool(ranges_ok))
    o = env.reset(seed=0)
    if float(o["depth"].max()) <= 0.0:
        rep.skip_check("深度图非空", "MuJoCo 深度渲染不可用（无 GPU/显示）")
    else:
        rep.check("深度图非空", float(o["depth"].max()) > 0.5
                  and float(o["valid_frac"]) > 0.2,
                  "max=%.2fm valid=%.0f%%"
                  % (float(o["depth"].max()), 100 * float(o["valid_frac"])))
    _, info = _run_episode(env)
    rep.check("oracle 干净通过", info.get("reason") == "pass",
              "reason=%s time=%.1fs" % (info.get("reason"), info.get("time", 0)))
    env_miss = GateServoEnv(EnvConfig())
    _, info_miss = _run_episode(env_miss, action=np.array([1.0, 0.0, 0.0]),
                                reset_kwargs={"vertical": 1.5})
    rep.check("越过开口外判 miss", info_miss.get("reason") == "miss",
              "reason=%s" % info_miss.get("reason"))
    env_miss.close()
    env_to = GateServoEnv(EnvConfig(max_time=0.1))
    _, info_to = _run_episode(env_to, action=np.zeros(3), max_steps=5)
    rep.check("超时判 timeout", info_to.get("reason") == "timeout",
              "reason=%s" % info_to.get("reason"))
    env_to.close()

    # 16-19 MLP
    net = TinyMLP(n_in=6, n_hid=4, n_out=3, seed=0)
    rep.check("MLP 参数数/前向形状", net.n_params() == 6 * 4 + 4 + 4 * 3 + 3
              and net.forward(np.zeros((2, 6), np.float32)).shape == (2, 3),
              net.summary())
    x = rng.normal(size=(5, 6)).astype(np.float32)
    y = rng.normal(size=(5, 3)).astype(np.float32)
    yh, cache = net.forward(x, cache=True)
    dy = 2.0 * (yh - y) / yh.size
    gW1, _, _, _ = net.backward(dy, cache)
    eps = 1e-6
    old = net.W1[0, 0]
    net.W1[0, 0] = old + eps
    lp = float(np.mean((net.forward(x) - y) ** 2))
    net.W1[0, 0] = old - eps
    lm = float(np.mean((net.forward(x) - y) ** 2))
    net.W1[0, 0] = old
    num = (lp - lm) / (2 * eps)
    rep.check("MLP 数值梯度", abs(num - gW1[0, 0]) / max(abs(num), 1e-9) < 2e-4,
              "num=%.6f ana=%.6f" % (num, gW1[0, 0]))
    net2 = TinyMLP(n_in=6, n_hid=8, n_out=3, seed=1)
    net2.set_stats(rng.normal(size=(64, 6)).astype(np.float32),
                   rng.normal(size=(64, 3)).astype(np.float32))
    xb = rng.normal(size=(64, 6)).astype(np.float32)
    yb = np.tile(np.array([3.0, 0.5, -0.5], np.float32), (64, 1))
    first = net2.train_batch(xb, yb, lr=1e-3)
    for _ in range(80):
        last = net2.train_batch(xb, yb, lr=1e-3)
    rep.check("MLP 训练损失下降", last < first, "%.5f -> %.5f" % (first, last))
    tmpdir = tempfile.mkdtemp(prefix="_selftest_", dir=".")
    path = os.path.join(tmpdir, "m.npz")
    net2.save(path, {"test": True})
    net3 = TinyMLP.load(path)
    same = np.allclose(net2.forward(net2._norm_x(xb)),
                       net3.forward(net3._norm_x(xb)), atol=1e-6)
    size = os.path.getsize(path)
    for f in os.listdir(tmpdir):
        os.remove(os.path.join(tmpdir, f))
    os.rmdir(tmpdir)
    rep.check("MLP 存取一致且 <100KB", same and size < 100 * 1024,
              "%d bytes" % size)

    # 20-22 dataset
    from .core.dataset import mirror_batch, split_by_episode
    dd = np.zeros((10, 12, 16), np.float32)
    ll = np.zeros((10, 3), np.float32)
    ee = np.array([0, 0, 0, 0, 1, 1, 1, 2, 2, 2], np.int32)
    ds = split_by_episode(dd, ll, ee, val_frac=0.34, seed=0)
    rep.check("按 episode 切分无泄漏",
              not (set(ds.train_episode.tolist()) & set(ds.val_episode.tolist())),
              "train=%s val=%s" % (sorted(set(ds.train_episode.tolist())),
                                   sorted(set(ds.val_episode.tolist()))))
    orig = np.arange(2 * 12 * 16, dtype=np.float32).reshape(2, 12, 16)
    dm, lm = mirror_batch(orig,
                          np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], np.float32),
                          np.array([True, False]))
    rep.check("镜像翻转且只改 left",
              np.allclose(dm[0], orig[0][:, ::-1]) and lm[0, 1] == -2.0
              and lm[1, 1] == 5.0)
    from .core.dataset import Dataset
    dtr = np.full((4, 12, 16), 2.0, np.float32)
    dva = np.full((2, 12, 16), 5.0, np.float32)
    ds2 = Dataset(dtr, np.zeros((4, 3), np.float32), dva,
                  np.ones((2, 3), np.float32), np.zeros(4, np.int32),
                  np.ones(2, np.int32))
    net4 = TinyMLP(n_in=192, n_hid=4, n_out=3, seed=0)
    net4.set_stats(ds2.train_depth, ds2.train_label)
    rep.check("标准化统计只用训练集", abs(net4.x_mean - 2.0) < 1e-6,
              "x_mean=%.3f" % net4.x_mean)

    # 23-25 closed loop/units/ROS
    ok = True
    for seed in (0, 1, 2):
        _, inf = _run_episode(env, max_steps=300, seed=seed)
        ok &= inf.get("reason") == "pass"
    rep.check("oracle 3 seeds 全过", bool(ok))
    env.reset(seed=0)
    start = env._ref.copy()
    for _ in range(40):
        env._command(np.array([1.0, 0.0, 0.0]))
    speed = (env._ref[0] - start[0]) / (40 * env.cfg.frame_dt)
    rep.check("动作单位：1 m/s 前进约 1 m/s", 0.8 < speed < 1.1,
              "measured=%.2f m/s" % speed)
    try:
        import uav_gate_center_servo.ros.gate_servo_node  # noqa: F401
        rep.check("ROS 节点可导入", True)
    except ImportError:
        rep.skip_check("ROS 节点可导入", "rclpy 未安装")

    env.close()
    print("\n" + "=" * 72)
    print("  通过 %d/%d   跳过 %d   总耗时 %.1f s"
          % (rep.ok, rep.n, len(rep.skip), time.time() - t0))
    if rep.skip:
        print("  跳过：%s" % "、".join(rep.skip))
    if rep.fail:
        print("  未通过：%s" % "、".join(rep.fail))
    print("=" * 72)
    return 0 if rep.ok + len(rep.skip) == rep.n else 1


if __name__ == "__main__":
    raise SystemExit(run_selftest())
