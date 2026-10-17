#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""SLAM 评测指标：ATE / RPE / 地图一致性 / 路标误差。

**两个 ATE 都给。** ``ate_raw`` 不做事后对齐，直接反映"从零开始积分"的真实误差；
``ate_aligned`` 先做 SE(2) 最优对齐再算，排除整体漂移自由度，是 KITTI 的通行做法。
只看其中一个都会被误导：对齐后的数字好看得多，但对齐本身用到了真值，
不能当作在线精度上报。
"""
from __future__ import annotations

import math
from typing import Dict, Tuple

import numpy as np

__all__ = ["se2_matrix", "align_se2", "ate", "rpe", "poses_local",
           "local_transform", "apply_se2", "landmark_errors",
           "landmark_shape_rmse", "map_iou", "summarize_runs", "format_table"]


# --------------------------------------------------------------------------- #
def se2_matrix(est: np.ndarray, gt: np.ndarray) -> np.ndarray:
    """求把 ``est`` 对齐到 ``gt`` 的最优 SE(2) 变换，返回 3×3 齐次矩阵。

    要求两个点集**等长且已配对**。需要把同一个变换用到另一组点上时用它，
    只要对齐后的点用 ``align_se2``。
    """
    est = np.asarray(est, float)[:, :2]
    gt = np.asarray(gt, float)[:, :2]
    ce, cg = est.mean(axis=0), gt.mean(axis=0)
    H = (est - ce).T @ (gt - cg)
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:                 # 排除反射解（"镜像对齐"）
        Vt = Vt.copy()
        Vt[-1] *= -1
        R = Vt.T @ U.T
    T = np.eye(3)
    T[:2, :2] = R
    T[:2, 2] = cg - R @ ce
    return T


def align_se2(est: np.ndarray, gt: np.ndarray) -> np.ndarray:
    """求把 ``est`` 对齐到 ``gt`` 的最优 SE(2) 变换（Umeyama 的二维特例）。"""
    return apply_se2(se2_matrix(est, gt), np.asarray(est, float)[:, :2])


def ate(est_xy: np.ndarray, gt_xy: np.ndarray) -> Dict[str, float]:
    est = np.asarray(est_xy, float)[:, :2]
    gt = np.asarray(gt_xy, float)[:, :2]
    n = min(len(est), len(gt))
    est, gt = est[:n], gt[:n]
    e_raw = np.linalg.norm(est - gt, axis=1)
    e_al = np.linalg.norm(align_se2(est, gt) - gt, axis=1)
    return {
        "ate_raw": float(np.sqrt(np.mean(e_raw ** 2))),
        "ate_aligned": float(np.sqrt(np.mean(e_al ** 2))),
        "ate_max": float(e_raw.max()),
        "ate_final": float(e_raw[-1]),
    }


# --------------------------------------------------------------------------- #
def _T(p) -> np.ndarray:
    """(x, y, θ) → 3×3 齐次矩阵。"""
    c, s = math.cos(float(p[2])), math.sin(float(p[2]))
    return np.array([[c, -s, float(p[0])], [s, c, float(p[1])], [0.0, 0.0, 1.0]])


def local_transform(poses: np.ndarray) -> np.ndarray:
    """返回把世界系点变换到 ``poses`` 第一帧坐标系的 3×3 齐次矩阵。"""
    return np.linalg.inv(_T(np.asarray(poses, float)[0]))


def apply_se2(T: np.ndarray, pts: np.ndarray) -> np.ndarray:
    """用 3×3 齐次矩阵变换二维点集：(N,≥2) → (N,2)。"""
    T = np.asarray(T, float)
    p = np.asarray(pts, float).reshape(-1, 2)
    return p @ T[:2, :2].T + T[:2, 2]


def poses_local(poses: np.ndarray) -> np.ndarray:
    """把一条位姿序列表示在**它自己第一帧**的坐标系里。

    估计器从原点出发，真值轨迹在世界系里另有起点，直接相减没有意义。
    统一换到各自第一帧后，两者都从单位位姿出发（固定整体自由度，
    与 SLAM 的规范自由度一致）。

    真值路标需用同一个 ``local_transform`` 变换。只转轨迹时，估计地图与
    真值地图相差一个首帧偏航旋转：本数据集首帧 yaw=0.965 rad，10 m 外
    错开 9 m，最近邻配对退化为随机（rmse 0.71 m ≈ 均匀随机期望 1/√2；
    修正后 0.22 m）。
    """
    p = np.asarray(poses, float)
    T0 = local_transform(p)
    out = np.empty_like(p)
    for i in range(len(p)):
        M = T0 @ _T(p[i])
        out[i, 0] = M[0, 2]
        out[i, 1] = M[1, 2]
        out[i, 2] = math.atan2(M[1, 0], M[0, 0])
    return out


def rpe(poses_est: np.ndarray, poses_gt: np.ndarray, delta: int = 10
        ) -> Dict[str, float]:
    """相对位姿误差：隔 ``delta`` 帧比较**相对运动**。

        E_i = (Q_i⁻¹ Q_{i+Δ})⁻¹ (P_i⁻¹ P_{i+Δ})

    比 ATE 更能反映局部一致性：整条轨迹平移一下 ATE 会很大而 RPE 不变，
    而漂移是逐段累积的，RPE 才看得出是哪一段坏了。
    """
    pe = np.asarray(poses_est, float)
    pg = np.asarray(poses_gt, float)
    n = min(len(pe), len(pg))
    if n <= delta + 1:
        return {"rpe_t": float("nan"), "rpe_r_deg": float("nan")}
    te, tr = [], []
    for i in range(0, n - delta):
        rel_e = np.linalg.inv(_T(pe[i])) @ _T(pe[i + delta])
        rel_g = np.linalg.inv(_T(pg[i])) @ _T(pg[i + delta])
        err = np.linalg.inv(rel_g) @ rel_e
        te.append(float(np.linalg.norm(err[:2, 2])))
        tr.append(abs(math.atan2(err[1, 0], err[0, 0])))
    return {"rpe_t": float(np.sqrt(np.mean(np.square(te)))),
            "rpe_r_deg": float(np.rad2deg(np.sqrt(np.mean(np.square(tr)))))}


# --------------------------------------------------------------------------- #
def landmark_errors(est: np.ndarray, gt: np.ndarray, match_radius: float = 1.0
                    ) -> Dict[str, float]:
    """估计地图与真值地图的最近邻配对误差。"""
    est = np.asarray(est, float).reshape(-1, 2)
    gt = np.asarray(gt, float).reshape(-1, 2)
    if len(est) == 0 or len(gt) == 0:
        return {"n_est": len(est), "n_gt": len(gt), "n_matched": 0,
                "lm_rmse": float("nan"), "lm_max": float("nan")}
    d = np.linalg.norm(est[:, None, :] - gt[None, :, :], axis=2)
    dd = d.min(axis=1)
    ok = dd < match_radius
    e = dd[ok]
    return {"n_est": len(est), "n_gt": len(gt), "n_matched": int(ok.sum()),
            "lm_rmse": float(np.sqrt(np.mean(e ** 2))) if len(e) else float("nan"),
            "lm_max": float(e.max()) if len(e) else float("nan")}


def landmark_shape_rmse(est: np.ndarray, gt: np.ndarray,
                        match_radius: float = 1.0) -> Dict[str, float]:
    """配对点对做 SE(2) 最优对齐后的残差 —— 只反映**地图形状**。

    ``landmark_errors`` 里估地图与真值地图共享第一帧坐标系，位置误差里混了
    "整体漂移"和"画歪了"两项；这里用配对点对估一个刚体变换再扣掉，剩下的
    就是形状误差。指标更严，但需要配对正确才有意义。
    """
    est = np.asarray(est, float).reshape(-1, 2)
    gt = np.asarray(gt, float).reshape(-1, 2)
    if len(est) < 3 or len(gt) < 3:
        return {"lm_shape": float("nan"), "n_shape": 0}
    # 迭代最近点：先粗对齐再配对，避免整体平移/旋转把配对全打乱
    # （半径 1 m 时旋转 0.9 rad、点距原点 4 m，位移能到 3.5 m）。
    cur = _rough_align(est, gt)
    n_ok = 0
    for _ in range(3):
        d2 = np.linalg.norm(cur[:, None, :] - gt[None, :, :], axis=2)
        j = d2.argmin(axis=1)
        ok = d2[np.arange(len(cur)), j] < match_radius
        n_ok = int(ok.sum())
        if n_ok < 3:
            return {"lm_shape": float("nan"), "n_shape": n_ok}
        # 变换只从配对点估计，但作用到全部点上（配对点与全集的长度不同）
        cur = apply_se2(se2_matrix(cur[ok], gt[j[ok]]), cur)
    d2 = np.linalg.norm(cur[:, None, :] - gt[None, :, :], axis=2)
    j = d2.argmin(axis=1)
    ok = d2[np.arange(len(cur)), j] < match_radius
    e = d2[np.arange(len(cur)), j][ok]
    if len(e) == 0:
        return {"lm_shape": float("nan"), "n_shape": 0}
    return {"lm_shape": float(np.sqrt(np.mean(e ** 2))), "n_shape": int(ok.sum())}


def _rough_align(est: np.ndarray, gt: np.ndarray) -> np.ndarray:
    """粗对齐：点数量相同就做一次整体 SE(2)，否则只对质心。"""
    if len(est) == len(gt):
        return apply_se2(se2_matrix(est, gt), est)
    return est - est.mean(axis=0) + gt.mean(axis=0)


def map_iou(est_grid: np.ndarray, gt_grid: np.ndarray,
            occupied_thresh: float = 0.5) -> float:
    a = np.asarray(est_grid) > occupied_thresh
    b = np.asarray(gt_grid) > occupied_thresh
    union = int(np.logical_or(a, b).sum())
    return float(np.logical_and(a, b).sum()) / union if union else float("nan")


# --------------------------------------------------------------------------- #
def summarize_runs(records, keys=("ate_raw", "ate_aligned", "rpe_t", "lm_rmse")) -> Dict:
    acc: Dict[str, Dict[str, list]] = {}
    for r in records:
        d = acc.setdefault(r["backend"], {k: [] for k in keys})
        for k in keys:
            v = r["metrics"].get(k)
            if v is not None and np.isfinite(v):
                d[k].append(v)
    return {b: {k: float(np.mean(v)) for k, v in d.items() if v} for b, d in acc.items()}


def format_table(rows, header) -> str:
    widths = [max(len(str(r[i])) for r in [header] + list(rows))
              for i in range(len(header))]
    line = "  ".join(str(h).ljust(w) for h, w in zip(header, widths))
    out = [line, "-" * len(line)]
    for r in rows:
        out.append("  ".join(str(c).ljust(w) for c, w in zip(r, widths)))
    return "\n".join(out)
