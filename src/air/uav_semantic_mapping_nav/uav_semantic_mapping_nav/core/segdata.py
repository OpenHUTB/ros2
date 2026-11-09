#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""语义数据集：RGB 图 + 真值语义标签 + 深度 + 位姿。

采集用**扫描式航线**（弓字形往复）覆盖 urban_block 场景：这既是分割网络
训练集的分布，也是后面语义建图的观测序列。深度图按 0.2~20 m 量化成 uint8，
与底座 ``depth_to_u8`` 的量程不同（建图需要看更远），所以这里单独编码。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from uav_mujoco_sim.core import GeometricController, Quadrotor, quat_to_euler
from uav_mujoco_sim.core.scenes import make_scene
from uav_mujoco_sim.core.sensors import CameraConfig, VisionSensors

from .labels import N_CLASS, SEMANTIC_NAMES

__all__ = ["SegEpisode", "SegDataset", "ScanTrajectory", "EpisodeSpec",
           "collect_episode", "collect_dataset", "MAP_DEPTH_MIN",
           "MAP_DEPTH_MAX", "depth_to_u8", "u8_to_depth", "N_CLASS"]

MAP_DEPTH_MIN, MAP_DEPTH_MAX = 0.2, 20.0     # 建图用深度量程（米）
SIM_DT = 0.01
IMU_UNUSED = 0

DEFAULT_HW = (128, 160)                      # 采集分辨率（高、宽）
NET_HW = (96, 96)                            # 网络输入分辨率，采集后重采样


def depth_to_u8(d: np.ndarray) -> np.ndarray:
    a = np.asarray(d, np.float32)
    q = np.rint((a - MAP_DEPTH_MIN) / (MAP_DEPTH_MAX - MAP_DEPTH_MIN) * 254.0) + 1.0
    q[(a <= 1e-6) | (a < MAP_DEPTH_MIN) | (a > MAP_DEPTH_MAX)] = 0.0
    return np.clip(q, 0, 255).astype(np.uint8)


def u8_to_depth(q: np.ndarray) -> np.ndarray:
    a = np.asarray(q, np.float32)
    d = (a - 1.0) / 254.0 * (MAP_DEPTH_MAX - MAP_DEPTH_MIN) + MAP_DEPTH_MIN
    d[a == 0] = 0.0
    return d


# --------------------------------------------------------------------------- #
@dataclass
class SegEpisode:
    rgb: np.ndarray            # (N,H,W,3) uint8
    seg: np.ndarray            # (N,H,W) int32 真值类别
    depth: np.ndarray          # (N,H,W) uint8 量化深度
    pos: np.ndarray            # (N,3) float32 世界系
    yaw: np.ndarray            # (N,) float32
    meta: Dict[str, object] = field(default_factory=dict)

    def __len__(self) -> int:
        return int(len(self.rgb))

    def subset(self, i0: int, i1: int) -> "SegEpisode":
        return SegEpisode(self.rgb[i0:i1], self.seg[i0:i1], self.depth[i0:i1],
                          self.pos[i0:i1], self.yaw[i0:i1], dict(self.meta))


@dataclass
class SegDataset:
    episodes: List[SegEpisode] = field(default_factory=list)

    def __len__(self) -> int:
        return int(sum(len(e) for e in self.episodes))

    def stack(self, idx: Optional[Sequence[int]] = None):
        eps = self.episodes if idx is None else [self.episodes[i] for i in idx]
        return (np.concatenate([e.rgb for e in eps]),
                np.concatenate([e.seg for e in eps]))

    def save(self, path: str) -> None:
        d: Dict[str, np.ndarray] = {"n_ep": np.asarray(len(self.episodes))}
        for i, e in enumerate(self.episodes):
            for k in ("rgb", "seg", "depth", "pos", "yaw"):
                d["%s_%d" % (k, i)] = getattr(e, k)
        np.savez_compressed(path, **d)

    @classmethod
    def load(cls, path: str) -> "SegDataset":
        with np.load(path, allow_pickle=False) as d:
            n = int(d["n_ep"])
            eps = [SegEpisode(*[d["%s_%d" % (k, i)] for k in
                                ("rgb", "seg", "depth", "pos", "yaw")])
                   for i in range(n)]
        return cls(eps)


# --------------------------------------------------------------------------- #
@dataclass
class EpisodeSpec:
    """一条扫描航线。``lanes`` 条横向往返，覆盖 ``span`` × ``depth`` 的区域。"""
    seed: int = 0
    scene: str = "urban_block"
    span: float = 26.0          # 沿 y 方向扫过的宽度
    depth_m: float = 18.0       # 沿 x 方向推进的距离
    lanes: int = 7
    height: float = 3.2
    speed: float = 2.2          # m/s
    start_x: float = -9.0
    start_y: float = 0.0
    spin: float = 0.0           # 起始偏航（弧度）

    def label(self) -> str:
        return "%s/span%.0f/lanes%d/z%.1f" % (self.scene, self.span, self.lanes,
                                              self.height)


class ScanTrajectory:
    """弓字形扫描：奇数道沿 +x、偶数道沿 -x，两条横道之间用**半圆弧**换道。

    换道不能做成"位置平移 + 偏航阶跃"：那样速度和偏航在段边界上同时跳变，
    几何控制器要在一瞬间同时改平移方向和机头朝向，姿态环饱和后会把机身甩出
    航线（实测漂到 50 m 外）。半圆弧让位置、速度、偏航全部连续，圆弧的切向
    自动就是机头方向。
    """

    def __init__(self, spec: EpisodeSpec):
        self.s = spec
        # 道距取正值：半圆掉头的绕行方向由道次奇偶决定，符号再反过来会让
        # 入口切向与行进方向相反（速度在边界上跳 180°，实测 30 s 处发散）。
        self.lane_dy = abs(spec.span) / max(spec.lanes - 1, 1)
        self.t_lane = spec.depth_m / max(spec.speed, 1e-6)
        # 相邻两条横道的**端点重合**（偶数道在 x_end 掉头，奇数道在 x_start 掉头），
        # 所以换道是在同一个点上做 180° 掉头、半径只有半个道距。
        # 按"跨到对侧端点"算会把半径算成半个对角线（实测 15 m），轨迹直接断开。
        self.r_turn = self.lane_dy / 2.0
        self.t_turn = (math.pi * self.r_turn / max(spec.speed, 1e-6)
                       if self.lane_dy > 0 else 0.0)
        self.seg = self.t_lane + self.t_turn
        self.period = spec.lanes * self.t_lane + (spec.lanes - 1) * self.t_turn

    def _arc(self, u: float, lane: int):
        """横道端点处的半圆掉头，``u`` ∈ [0,1]。切向自动给出机头方向。

        绕行方向必须让**入口切向等于本道的行进方向**：偶数道沿 +x，奇数道沿 -x。
        ``da = +π`` 时入口切向是 +x，``da = -π`` 时是 -x，两者出口都在下一道上。
        """
        s = self.s
        x_end = s.start_x + (s.depth_m if lane % 2 == 0 else 0.0)
        y0 = s.start_y + lane * self.lane_dy
        cy = y0 + self.lane_dy / 2.0
        r = self.r_turn
        a0 = math.atan2(y0 - cy, 0.0)          # -π/2
        da = math.pi if lane % 2 == 0 else -math.pi
        a = a0 + da * u
        x = x_end + r * math.cos(a)
        y = cy + r * math.sin(a)
        tx, ty = -math.sin(a) * da, math.cos(a) * da
        n = math.hypot(tx, ty) or 1.0
        v = np.array([tx / n, ty / n, 0.0]) * s.speed
        return x, y, v, math.atan2(ty, tx)

    def eval(self, t: float):
        s = self.s
        t = min(max(float(t), 0.0), self.period)
        lane = min(int(t // self.seg) if self.seg > 0 else 0, s.lanes - 1)
        rem = t - lane * self.seg
        if rem <= self.t_lane or lane + 1 >= s.lanes or self.t_turn <= 0:
            u = min(rem / self.t_lane, 1.0) if self.t_lane > 0 else 1.0
            fwd = (lane % 2 == 0)
            x = s.start_x + s.depth_m * (u if fwd else 1.0 - u)
            y = s.start_y + lane * self.lane_dy
            v = np.array([s.speed * (1.0 if fwd else -1.0), 0.0, 0.0])
            yaw = 0.0 if fwd else math.pi
        else:
            x, y, v, yaw = self._arc((rem - self.t_lane) / self.t_turn, lane)
        return (np.array([x, y, s.height]), v, np.zeros(3), yaw, 0.0)

    def sample(self, n: int) -> np.ndarray:
        return np.asarray([self.eval(self.period * k / n)[0] for k in range(n)])


# --------------------------------------------------------------------------- #
def collect_episode(spec: EpisodeSpec, hw: Tuple[int, int] = DEFAULT_HW,
                    rate_hz: float = 5.0, verbose: bool = False) -> SegEpisode:
    """按 ``spec`` 扫一遍并录一集。"""
    cam = CameraConfig(width=hw[1], height=hw[0], quality="fast",
                       depth_max_range=MAP_DEPTH_MAX)
    traj = ScanTrajectory(spec)
    scene = make_scene(spec.scene, seed=spec.seed)
    quad = Quadrotor(scene=scene, dt=SIM_DT, substeps=4)
    ctrl = GeometricController(quad.params)
    vision = VisionSensors(quad.model, cam)

    p0, _, _, yaw0, _ = traj.eval(0.0)
    quad.reset(pos=tuple(p0), yaw=float(yaw0))
    per = max(int(round(1.0 / (rate_hz * SIM_DT))), 1)
    n_steps = int(round(traj.period / SIM_DT))

    rec = {k: [] for k in ("rgb", "seg", "depth", "pos", "yaw")}
    try:
        for k in range(n_steps):
            t = k * SIM_DT
            pd, vd, ad, yd, _ = traj.eval(t)
            quad.step_with_control(ctrl.act(quad, pd, vd, ad, yd))
            if (k + 1) % per:
                continue
            st = quad.state()
            rec["rgb"].append(vision.rgb(quad.data))
            rec["seg"].append(vision.segmentation(quad.data))
            rec["depth"].append(depth_to_u8(vision.depth(quad.data)))
            rec["pos"].append(st.pos.astype(np.float32))
            rec["yaw"].append(float(st.yaw))
    finally:
        vision.close()

    ep = SegEpisode(rgb=np.asarray(rec["rgb"]), seg=np.asarray(rec["seg"]),
                    depth=np.asarray(rec["depth"]),
                    pos=np.asarray(rec["pos"], np.float32),
                    yaw=np.asarray(rec["yaw"], np.float32),
                    meta={"spec": spec.label(), "period": traj.period})
    # 范围守卫：起始点落进障碍时几何控制器会直接发散，而且帧率、类别占比
    # 看上去都"正常"，只有轨迹飞到几公里外。不拦的话要等到看位置统计才发现。
    # SceneObject.size 的长度随几何类型变化（球只有半径一项），统一取最大值
    reach = max(max(abs(o.pos[0]), abs(o.pos[1])) + max(o.size)
                for o in scene.objects) if scene.objects else 20.0
    lim = reach + 15.0
    far = float(np.abs(ep.pos[:, :2]).max())
    if far > lim:
        raise RuntimeError(
            "航线发散：最远到 %.0f m，超出场景范围 %.0f m（%s）。"
            "起始点或某条横道很可能落在障碍里。"
            % (far, lim, spec.label()))
    if verbose:
        cnt = np.bincount(ep.seg.ravel(), minlength=N_CLASS)
        top = ", ".join("%s %.1f%%" % (SEMANTIC_NAMES[i], 100.0 * cnt[i] / cnt.sum())
                        for i in np.argsort(cnt)[::-1][:4])
        print("  %-28s %4d 帧  类别占比：%s" % (spec.label(), len(ep), top))
    return ep


# 航线按场景的**实际可通行空间**给：urban_block 的建筑外沿在 |y| ≥ 5.25
# （中心 ±(7~13)、半深 ≤1.75），树冠顶约 5.1 m，所以走廊取 |y| ≤ 4.5、z ≥ 6.0；
# gate_track 的 8 个门沿 +x 排在 x = 5…40，开口中心 z ≈ 2.0。起始点落进障碍时
# 几何控制器会发散（实测飞到 3.5 km 外），collect_episode 末尾的范围守卫拦这个。
DEFAULT_SPECS = [
    EpisodeSpec(0, "urban_block", span=9.0, depth_m=30.0, lanes=4, height=6.5,
                start_x=-20.0, start_y=-4.5),
    EpisodeSpec(0, "urban_block", span=9.0, depth_m=24.0, lanes=3, height=7.5,
                start_x=-16.0, start_y=-4.5),
    EpisodeSpec(0, "gate_track", span=0.0, depth_m=40.0, lanes=2, height=2.0,
                start_x=0.0, start_y=0.0),
    EpisodeSpec(0, "open_ground", span=16.0, depth_m=24.0, lanes=3, height=4.0,
                start_x=-12.0, start_y=-8.0),
]


def collect_dataset(specs: Optional[Sequence[EpisodeSpec]] = None,
                    verbose: bool = True) -> SegDataset:
    eps = []
    for s in (specs if specs is not None else DEFAULT_SPECS):
        eps.append(collect_episode(s, verbose=verbose))
    return SegDataset(eps)
