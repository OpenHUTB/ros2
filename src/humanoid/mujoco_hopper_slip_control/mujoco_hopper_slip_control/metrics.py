# -*- coding: utf-8 -*-
"""逐跳指标提取与 CSV 输出。纯 numpy，不依赖 mujoco。"""

import csv
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

import mujoco_hopper_slip_control.config as config


@dataclass
class HopRecord:
    """一次完整弹跳（离地 → 顶点 → 再次触地）的指标。"""

    index: int
    t_liftoff: float
    t_touchdown: float
    z_liftoff: float
    vz_liftoff: float
    z_apex: float
    stance_time: float
    flight_time: float
    peak_leg_force: float
    injected_force: float

    @property
    def energy_per_hop(self) -> float:
        """单跳注入能量估计：注能力 × 伸展行程。"""
        return self.injected_force * max(self.q_stroke, 0.0)

    q_stroke: float = 0.0


def _segments(mask: np.ndarray) -> List[tuple]:
    """把布尔序列切成 [start, end) 区间列表。"""
    out, start = [], None
    for i, v in enumerate(mask):
        if v and start is None:
            start = i
        elif not v and start is not None:
            out.append((start, i))
            start = None
    if start is not None:
        out.append((start, len(mask)))
    return out


def extract_hops(log: Dict[str, np.ndarray]) -> List[HopRecord]:
    """从遥测里提取每一跳。

    顶点取飞行段内躯干高度的最大值（不靠 vz 过零判据，避免数值抖动漏检）。
    """
    contact = log["contact"].astype(bool)
    t = log["t"]
    z = log["z"]
    vz = log["vz"]
    q = log["q"]
    force = log["leg_cmd"]

    hops: List[HopRecord] = []
    for start, end in _segments(~contact):
        if end - start < 3:
            continue
        if start == 0 or end >= len(t):
            continue
        seg = slice(start, end)
        apex_i = start + int(np.argmax(z[seg]))
        # 支撑相 = 这一段飞行之前的接触段
        prev = [s for s in _segments(contact) if s[1] == start]
        stance_time = float(t[start] - t[prev[0][0]]) if prev else float("nan")
        # 该飞行段之前那次支撑里记录的注能力与压缩行程
        st_seg = slice(prev[0][0], prev[0][1]) if prev else slice(0, 0)
        injected = float(np.abs(force[st_seg]).max()) if prev else 0.0
        stroke = float(q[st_seg].max() - q[st_seg].min()) if prev else 0.0
        hops.append(HopRecord(
            index=len(hops),
            t_liftoff=float(t[start]),
            t_touchdown=float(t[end]),
            z_liftoff=float(z[start]),
            vz_liftoff=float(vz[start]),
            z_apex=float(z[apex_i]),
            stance_time=stance_time,
            flight_time=float(t[end] - t[start]),
            peak_leg_force=float(np.abs(force[st_seg]).max()) if prev else 0.0,
            injected_force=injected,
            q_stroke=stroke,
        ))
    return hops


def steady_stats(hops: List[HopRecord], n: int = config.SETTLE_HOPS) -> Dict[str, float]:
    """最后 n 跳的稳态统计。

    ``count`` 是参与稳态统计的跳数（最多 n 跳），``hops_total`` 才是总跳数。
    两者混用会把「8 跳窗口」误读成「全程只跳了 8 次」。
    """
    if not hops:
        return {"count": 0, "hops_total": 0}
    tail = hops[-n:]
    apex = np.array([h.z_apex for h in tail])
    return {
        "count": len(tail),
        "hops_total": len(hops),
        "apex_mean": float(apex.mean()),
        "apex_std": float(apex.std()),
        "apex_min": float(apex.min()),
        "apex_max": float(apex.max()),
        "stance_mean": float(np.nanmean([h.stance_time for h in tail])),
        "flight_mean": float(np.nanmean([h.flight_time for h in tail])),
        "vz_liftoff_mean": float(np.mean([h.vz_liftoff for h in tail])),
        "peak_force_mean": float(np.mean([h.peak_leg_force for h in tail])),
        "injected_force_mean": float(np.mean([h.injected_force for h in tail])),
        "duty": float(np.nanmean([h.stance_time for h in tail]) /
                      (np.nanmean([h.stance_time for h in tail]) +
                       np.nanmean([h.flight_time for h in tail]))),
    }


def recovery_time(log: Dict[str, np.ndarray], t0: float,
                  thresh: float = config.RECOVER_VX) -> float:
    """扰动后 |vx| 最后一次超过阈值的时刻距 t0 的时长。

    仅用于报告横向漂移，**不作为"恢复成功"的判据**：点足单足机器人横向
    控制权限很弱，实测扰动后横向速度衰减很慢（见 README 能力边界）。
    """
    t, vx = log["t"], log["vx"]
    after = t >= t0
    if not after.any():
        return float("nan")
    idx = np.where(after & (np.abs(vx) > thresh))[0]
    return 0.0 if len(idx) == 0 else float(t[idx[-1]] - t0)


def apex_recovery_time(hops: List[HopRecord], t0: float,
                       target: float = config.TARGET_HEIGHT,
                       tol: float = config.TOL_APEX,
                       consecutive: int = 3) -> float:
    """扰动后**顶点高度**回到目标带的用时（竖直方向鲁棒性判据）。

    找第一个下标 i，使 ``hops[i:]`` 中连续 ``consecutive`` 跳的顶点都落在
    ``target ± tol`` 内，返回该跳距 t0 的时间；找不到返回 ``nan``。
    """
    after = [h for h in hops if h.t_liftoff > t0]
    for i in range(len(after)):
        win = after[i:i + consecutive]
        if len(win) < consecutive:
            break
        if all(abs(h.z_apex - target) <= tol for h in win):
            return float(after[i].t_liftoff - t0)
    return float("nan")


def write_csv(path: Path, rows: List[dict]) -> Path:
    """把字典列表写成 CSV。"""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return path
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    return path


def hops_to_rows(hops: List[HopRecord]) -> List[dict]:
    """HopRecord -> CSV 行。"""
    rows = []
    for h in hops:
        d = asdict(h)
        d["energy_per_hop"] = round(h.energy_per_hop, 3)
        rows.append(d)
    return rows
