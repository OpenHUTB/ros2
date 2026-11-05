#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""冒烟测试：不依赖 ROS/显示设备（无显示时自动跳过渲染项）。

    cd <包目录> && python -m pytest test/ -v     # 或 python main.py --selftest
"""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from uav_mujoco_sim.core import (  # noqa: E402
    Quadrotor, GeometricController, mixer, make_scene,
)

HAS_DISPLAY = bool(os.name == "nt" or os.environ.get("DISPLAY")
                   or os.environ.get("WAYLAND_DISPLAY"))


@pytest.fixture(scope="module")
def quad():
    return Quadrotor(scene=make_scene("open_ground"), dt=0.01, substeps=4)


# --------------------------------------------------------------------------- #
# 模型与混控
# --------------------------------------------------------------------------- #
def test_mass_includes_rotor_subtree(quad):
    """质量按子树计：body_mass 只取到 0.85 kg，漏掉 4×0.025 kg 旋翼（实测下坠 1.29 m）。"""
    assert 0.90 < quad.params.mass < 1.00, quad.params.mass


def test_mixer_hover_is_balanced(quad):
    p = quad.params
    th = mixer(p.mass * 9.81, np.zeros(3), p)
    assert np.allclose(th, p.mass * 9.81 / 4.0, atol=1e-6)


def test_mixer_saturation_stays_feasible(quad):
    """饱和整体等比缩小而非逐个 clip：逐个 clip 时圆轨迹误差 17 mm → 474 m。"""
    th = mixer(9.3, np.array([9.9, 9.9, 9.9]), quad.params)
    assert th.min() >= -1e-9
    assert th.max() <= quad.params.max_thrust + 1e-9


def test_mixer_yaw_channel_independent(quad):
    """X 型布局若对角桨不同向，混控矩阵奇异、偏航无法独立控制。"""
    p = quad.params
    th = mixer(p.mass * 9.81, np.array([0.0, 0.0, 0.05]), p)
    assert np.ptp(th) > 1e-3


# --------------------------------------------------------------------------- #
# 控制
# --------------------------------------------------------------------------- #
def test_hover_converges(quad):
    quad.reset(pos=(0.0, 0.0, 1.5))
    ctrl = GeometricController(quad.params)
    zero, tgt = np.zeros(3), np.array([0.0, 0.0, 1.5])
    for _ in range(1000):
        quad.step_with_control(ctrl.act(quad, tgt, zero, zero, 0.0))
        if np.linalg.norm(quad.state().pos - tgt) < 1e-4:
            break
    assert np.linalg.norm(quad.state().pos - tgt) < 1e-3


def test_circle_tracking(quad):
    quad.reset(pos=(2.0, 0.0, 1.5))
    ctrl = GeometricController(quad.params)
    R, w = 2.0, 0.5
    errs = []
    for k in range(1600):
        t = quad.time
        pd = np.array([R * np.cos(w * t), R * np.sin(w * t), 1.5])
        vd = np.array([-R * w * np.sin(w * t), R * w * np.cos(w * t), 0.0])
        ad = np.array([-R * w * w * np.cos(w * t), -R * w * w * np.sin(w * t), 0.0])
        quad.step_with_control(ctrl.act(quad, pd, vd, ad, np.arctan2(vd[1], vd[0])))
        if k > 900:
            errs.append(np.linalg.norm(quad.state().pos - pd))
    rmse = float(np.sqrt(np.mean(np.square(errs))))
    assert rmse < 0.05, "圆周轨迹 RMSE %.1f mm" % (rmse * 1000)


# --------------------------------------------------------------------------- #
# 场景
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("name", ["open_ground", "obstacle_field", "corridor",
                                  "gate_track", "urban_block"])
def test_scene_builds(name):
    q = Quadrotor(scene=make_scene(name, seed=0))
    assert q.model.ngeom > 5


def test_scene_seed_reproducible():
    a = make_scene("obstacle_field", seed=7)
    b = make_scene("obstacle_field", seed=7)
    c = make_scene("obstacle_field", seed=8)
    assert [o.name for o in a.objects] == [o.name for o in b.objects]
    assert [o.pos for o in a.objects] != [o.pos for o in c.objects]


# --------------------------------------------------------------------------- #
# 传感器（无显示环境自动跳过）
# --------------------------------------------------------------------------- #
@pytest.mark.skipif(not HAS_DISPLAY, reason="无显示环境，跳过渲染")
def test_vision_sensors():
    from uav_mujoco_sim.core import VisionSensors, CameraConfig
    q = Quadrotor(scene=make_scene("urban_block", seed=3))
    q.reset(pos=(-24.0, -4.0, 5.0), yaw=0.6)
    vs = VisionSensors(q.model, CameraConfig(width=160, height=120, quality="fast"))
    try:
        rgb = vs.rgb(q.data)
        assert rgb.shape == (120, 160, 3) and rgb.dtype == np.uint8

        dep = vs.depth(q.data)
        assert dep.shape == (120, 160) and dep.dtype == np.float32
        assert dep.max() <= vs.cfg.depth_max_range + 1e-6
        assert (dep > 0).any(), "视野里应当有东西，否则深度全无效"

        seg = vs.segmentation(q.data)
        # 语义类别只可能是 0~8；出现天文数字说明分割缓冲被当成了深度缓冲
        ids = [int(i) for i in np.unique(seg)]
        assert all(0 <= i <= 8 for i in ids), ids
        assert len([i for i in ids if i > 0]) >= 1
    finally:
        vs.close()


def test_lidar_no_self_hit():
    """雷达排除自身机体；MjData 构造后须先 mj_forward，site_xpos 才有效。"""
    from uav_mujoco_sim.core import LidarSensor, LidarConfig
    q = Quadrotor(scene=make_scene("obstacle_field", seed=7))
    ld = LidarSensor(q.model, LidarConfig(n_azimuth=72, max_range=15.0))
    pts = ld.scan(q.data)
    assert len(pts) > 0, "障碍场里应当能扫到东西（mj_forward 没做会扫到 0 点）"
    d = np.linalg.norm(pts - q.data.site_xpos[ld.site_id], axis=1)
    assert d.min() > 0.25, "最近命中 %.3f m，疑似自遮挡" % d.min()


def test_imu_specific_force():
    """悬停时加速度计 z 应约等于 +g（比力语义）。"""
    from uav_mujoco_sim.core import ImuSensor
    q = Quadrotor(scene=make_scene("open_ground"))
    ctrl = GeometricController(q.params)
    zero = np.zeros(3)
    tgt = np.array([0.0, 0.0, 2.0])
    for _ in range(800):
        q.step_with_control(ctrl.act(q, tgt, zero, zero, 0.0))
    imu = ImuSensor(q.model, seed=1)
    az = float(np.mean([imu.read(q.data, q.time).acc[2] for _ in range(20)]))
    assert abs(az - 9.81) < 0.5, az
