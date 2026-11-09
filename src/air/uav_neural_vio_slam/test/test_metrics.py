# -*- coding: utf-8 -*-
"""评测指标在已知误差下的取值。不需要 ROS、不需要 mujoco。"""
import math

import numpy as np
import pytest

from uav_neural_vio_slam.core.metrics import (align_se2, apply_se2, ate,
                                              landmark_errors,
                                              landmark_shape_rmse,
                                              local_transform, map_iou,
                                              poses_local, rpe)


def _pose_seq(n=60, step=0.2, yaw=0.0):
    x = np.arange(n) * step
    return np.column_stack([x, np.zeros(n), np.full(n, yaw)])


def test_align_se2_identity():
    p = np.random.default_rng(0).normal(size=(30, 2)) * 3.0
    z = np.zeros(30)
    a = align_se2(np.column_stack([p, z]), np.column_stack([p, z]))
    assert np.abs(a - p).max() < 1e-9


def test_align_se2_recovers_known_transform():
    rng = np.random.default_rng(1)
    p = rng.normal(size=(40, 2)) * 3.0
    th = 0.7
    R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
    q = p @ R.T + np.array([2.0, -1.0])
    z = np.zeros(40)
    a = align_se2(np.column_stack([q, z]), np.column_stack([p, z]))[:, :2]
    assert np.abs(a - p).max() < 1e-6


def test_align_se2_rejects_reflection():
    """真值点集抖动时，SVD 解可能给出镜像解，必须被排除。"""
    rng = np.random.default_rng(2)
    p = rng.normal(size=(50, 2)) * 5.0
    q = p + rng.normal(scale=1e-3, size=p.shape)
    z = np.zeros(len(p))
    a = align_se2(np.column_stack([q, z]), np.column_stack([p, z]))[:, :2]
    assert np.abs(a - p).max() < 0.02


def test_ate_translation_only():
    gt = _pose_seq()
    est = gt.copy()
    est[:, 0] += 0.3
    assert abs(ate(est, gt)["ate_raw"] - 0.3) < 1e-9
    # 纯平移在 SE(2) 对齐后应被完全吃掉
    assert ate(est, gt)["ate_aligned"] < 1e-9


def test_rpe_ignores_global_scale_error():
    gt = _pose_seq()
    est = gt.copy()
    est[:, 0] = gt[:, 0] * 1.001
    assert rpe(est, gt, delta=10)["rpe_t"] < 0.02


def test_rpe_detects_local_jump():
    gt = _pose_seq()
    est = gt.copy()
    est[30:, 1] += 0.5          # 第 30 帧起横向跳 0.5 m
    assert rpe(est, gt, delta=10)["rpe_t"] > 0.1


def test_poses_local_fixes_first_frame():
    gt = _pose_seq(yaw=0.9)
    gt[:, :2] += np.array([5.0, -3.0])
    p = poses_local(gt)
    assert np.abs(p[0]).max() < 1e-9


def test_local_transform_and_apply_se2_consistent():
    """apply_se2(local_transform(poses), pts) 应与 poses_local 同一套规范。"""
    gt = _pose_seq(yaw=0.9)
    gt[:, :2] += np.array([5.0, -3.0])
    pts = np.array([[6.0, -2.0], [4.0, -4.0]])
    T = local_transform(gt)
    q = apply_se2(T, pts)
    # 世界系下 pts 相对首帧位姿的关系，应与变换后的关系一致
    d0 = pts - gt[0, :2]
    c, s = math.cos(-gt[0, 2]), math.sin(-gt[0, 2])
    ref = np.column_stack([c * d0[:, 0] - s * d0[:, 1],
                           s * d0[:, 0] + c * d0[:, 1]])
    assert np.abs(q - ref).max() < 1e-9


def test_landmark_errors_nearest_neighbour():
    est = np.array([[0.0, 0.0], [1.0, 0.0], [2.5, 0.0]])
    gt = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [9.0, 9.0]])
    e = landmark_errors(est, gt)
    assert e["n_matched"] == 3
    assert abs(e["lm_max"] - 0.5) < 1e-9
    # 配对半径是开区间：正好落在边界上的不算匹配
    far = np.array([[10.0, 0.0]])
    assert landmark_errors(far, gt)["n_matched"] == 0


def test_landmark_shape_rmse_removes_rigid_offset():
    """整体平移/旋转应被 SE(2) 对齐吃掉，只剩下形状误差。"""
    rng = np.random.default_rng(3)
    gt = rng.normal(size=(25, 2)) * 4.0
    th = 0.9
    R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
    est = gt @ R.T + np.array([3.0, 2.0])
    s = landmark_shape_rmse(est, gt)
    assert s["lm_shape"] < 1e-6
    # 未对齐的最近邻配对会被刚体变换打乱：形状误差几乎为零，直接比却明显非零
    assert landmark_errors(est, gt)["lm_rmse"] > 0.05


def test_map_iou_bounds():
    a = np.zeros((10, 10))
    b = np.zeros((10, 10))
    # 两张空图没有并集，IoU 未定义
    assert np.isnan(map_iou(a, b))
    a[:, :5] = 1.0
    assert abs(map_iou(a, a) - 1.0) < 1e-9
    b[:, 5:] = 1.0
    assert map_iou(a, b) == 0.0


def test_map_iou_partial_overlap():
    a = np.zeros((10, 10))
    b = np.zeros((10, 10))
    a[:, :6] = 1.0
    b[:, :4] = 1.0
    # 交 4 列、并 6 列
    assert abs(map_iou(a, b) - 4.0 / 6.0) < 1e-9
