#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""轨迹精度指标：ATE、RPE、漂移率。

指标定义与主流 SLAM/里程计评估一致（参见 Sturm et al., 2012）：

* **ATE**（Absolute Trajectory Error，绝对轨迹误差）：
  先对整条轨迹做一次刚体对齐，再统计每个采样点的位置误差。
  反映"整条轨迹整体偏了多少"，对齐消除了起点坐标系差异。
* **RPE**（Relative Pose Error，相对位姿误差）：
  对固定时间间隔 ``delta`` 的两帧，比较估计与真值的**相对运动**，
  不需要对齐，反映"局部运动估计准不准"（尺度误差、漂移都会体现）。
* **漂移率**：终点位置误差除以真值轨迹长度，用百分比表示，直观回答
  "跑 100 m 会偏多少"。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Sequence

import numpy as np

from .se2 import Pose, align_umeyama, compose, inverse, path_length, positions, yaws, wrap_angle


@dataclass
class ATEResult:
    """绝对轨迹误差统计（单位：米 / 度）。"""

    count: int
    rmse: float
    mean: float
    max: float
    std: float
    rmse_rot_deg: float
    scale: float          # 对齐时估计的尺度（未启用尺度对齐时恒为 1）
    yaw_offset_deg: float  # 对齐引入的整体旋转
    aligned: bool

    def as_dict(self) -> Dict[str, float | int | bool]:
        return {
            "ate_count": self.count,
            "ate_rmse_m": round(self.rmse, 4),
            "ate_mean_m": round(self.mean, 4),
            "ate_max_m": round(self.max, 4),
            "ate_std_m": round(self.std, 4),
            "ate_rmse_rot_deg": round(self.rmse_rot_deg, 3),
            "ate_scale": round(self.scale, 5),
            "ate_yaw_offset_deg": round(self.yaw_offset_deg, 3),
            "ate_aligned": self.aligned,
        }


@dataclass
class RPEResult:
    """相对位姿误差统计（单位：米 / 度）。"""

    delta: int
    count: int
    trans_rmse: float
    trans_mean: float
    rot_rmse_deg: float
    rot_mean_deg: float

    def as_dict(self) -> Dict[str, float | int]:
        return {
            "rpe_delta": self.delta,
            "rpe_count": self.count,
            "rpe_trans_rmse_m": round(self.trans_rmse, 4),
            "rpe_trans_mean_m": round(self.trans_mean, 4),
            "rpe_rot_rmse_deg": round(self.rot_rmse_deg, 3),
            "rpe_rot_mean_deg": round(self.rot_mean_deg, 3),
        }


def _check_same_length(est: Sequence[Pose], gt: Sequence[Pose]) -> None:
    if len(est) != len(gt):
        raise ValueError(f"估计与真值轨迹长度不一致: {len(est)} vs {len(gt)}")
    if len(est) < 2:
        raise ValueError("至少需要 2 个位姿")


def ate(est: Sequence[Pose], gt: Sequence[Pose], align: bool = True,
        with_scale: bool = False) -> ATEResult:
    """计算绝对轨迹误差。

    参数
    ----
    est : 估计轨迹（里程计/SLAM 输出）
    gt  : 真值轨迹，与 est 逐点对应
    align : 是否先做刚体对齐（默认 True）。关闭时直接比较原始坐标
    with_scale : 对齐时是否估计尺度。默认 False，因为尺度误差本身就是要评估的对象
    """
    _check_same_length(est, gt)
    est_pts, gt_pts = positions(est), positions(gt)
    est_yaw = yaws(est)

    if align:
        alignment = align_umeyama(est_pts, gt_pts, with_scale=with_scale)
        est_pts = alignment.apply_points(est_pts)
        est_yaw = alignment.apply_yaws(est_yaw)
        scale, yaw_off = alignment.scale, alignment.yaw_offset
    else:
        scale, yaw_off = 1.0, 0.0

    err = np.hypot(*(est_pts - gt_pts).T)
    rot_err = np.abs([wrap_angle(a - b) for a, b in zip(est_yaw, yaws(gt))])
    return ATEResult(
        count=len(err),
        rmse=float(np.sqrt((err ** 2).mean())),
        mean=float(err.mean()),
        max=float(err.max()),
        std=float(err.std()),
        rmse_rot_deg=float(np.degrees(np.sqrt((rot_err ** 2).mean()))),
        scale=float(scale),
        yaw_offset_deg=float(np.degrees(yaw_off)),
        aligned=bool(align),
    )


def rpe(est: Sequence[Pose], gt: Sequence[Pose], delta: int = 1) -> RPEResult:
    """计算相对位姿误差。

    对每个 ``i``，取相对运动 ``gt[i]^-1 ∘ gt[i+delta]`` 与 ``est[i]^-1 ∘ est[i+delta]``，
    其差 ``(gt_rel)^-1 ∘ est_rel`` 的平移长度与旋转角即为该处的误差。
    """
    _check_same_length(est, gt)
    if delta < 1:
        raise ValueError("delta 必须 >= 1")
    if len(est) <= delta:
        raise ValueError(f"轨迹长度 {len(est)} 不足以计算 delta={delta} 的相对误差")

    trans, rot = [], []
    for i in range(len(est) - delta):
        gt_rel = compose(inverse(gt[i]), gt[i + delta])
        est_rel = compose(inverse(est[i]), est[i + delta])
        err = compose(inverse(gt_rel), est_rel)
        trans.append(math.hypot(err.x, err.y))
        rot.append(abs(wrap_angle(err.yaw)))

    trans = np.asarray(trans)
    rot = np.asarray(rot)
    return RPEResult(
        delta=int(delta),
        count=int(trans.shape[0]),
        trans_rmse=float(np.sqrt((trans ** 2).mean())),
        trans_mean=float(trans.mean()),
        rot_rmse_deg=float(np.degrees(np.sqrt((rot ** 2).mean()))),
        rot_mean_deg=float(np.degrees(rot.mean())),
    )


def drift_percent(est: Sequence[Pose], gt: Sequence[Pose], align: bool = True) -> float:
    """终点漂移率（%）：终点位置误差 / 真值轨迹长度 * 100。

    该指标最直观：跑完整条轨迹后累计偏了多少，按路径长度归一化，
    便于比较不同长度的轨迹。
    """
    _check_same_length(est, gt)
    est_pts, gt_pts = positions(est), positions(gt)
    if align:
        alignment = align_umeyama(est_pts, gt_pts, with_scale=False)
        est_pts = alignment.apply_points(est_pts)
    end_err = float(np.hypot(*(est_pts[-1] - gt_pts[-1])))
    length = path_length(gt)
    if length < 1e-9:
        return float("inf")
    return 100.0 * end_err / length


def evaluate_all(est: Sequence[Pose], gt: Sequence[Pose],
                 deltas: Sequence[int] = (1, 10, 50),
                 align: bool = True) -> Dict[str, float | int | bool]:
    """一次性给出 ATE + 多个 delta 的 RPE + 漂移率，返回可直接存 CSV 的字典。

    注意漂移率给两个版本：

    * ``drift_percent``：**未对齐**的原始终点误差 / 路径长度——这是
      "跑 100 m 偏多少" 的直接含义，评估里程计标定效果时应看这个；
    * ``drift_percent_aligned``：对齐后的版本，与 ATE 口径一致，便于和文献对照。
    """
    out: Dict[str, float | int | bool] = {}
    out.update(ate(est, gt, align=align).as_dict())
    for d in deltas:
        if len(est) > d:
            r = rpe(est, gt, delta=d)
            out[f"rpe_{d}_trans_rmse_m"] = round(r.trans_rmse, 4)
            out[f"rpe_{d}_trans_mean_m"] = round(r.trans_mean, 4)
            out[f"rpe_{d}_rot_rmse_deg"] = round(r.rot_rmse_deg, 3)
            out[f"rpe_{d}_rot_mean_deg"] = round(r.rot_mean_deg, 3)
            out[f"rpe_{d}_count"] = r.count
    out["drift_percent"] = round(drift_percent(est, gt, align=False), 4)
    out["drift_percent_aligned"] = round(drift_percent(est, gt, align=align), 4)
    out["gt_path_length_m"] = round(path_length(gt), 3)
    return out
