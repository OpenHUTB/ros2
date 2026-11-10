"""BSD（Blind Spot Detection，盲区监测）决策核心。

纯离线逻辑：不依赖 ROS 与 CARLA，可独立单元测试。

盲区定义（传感器系，LiDAR 安装于车顶中心：x 向前、y 向左、z 向上）：

  左盲区：x ∈ [x_min, x_max]（默认 [-25, 2]，斜后方 25 m 到斜前方 2 m，
          覆盖邻车道来车的接近段——与实车 BSD 角雷达 25~60 m 后向探测一致）
          y ∈ [y_near, y_far]（默认 [2.9, 4.2]，相邻车道。近界 2.9 m 排除
          路缘/路侧灌木（实测 Town01 y≈1.8~2.5），远界 4.2 m 排除更外侧的
          低矮植被（实测 y≈4.6~4.9、世界高 0.3~0.6 m，探针定位）；邻车道
          车身 y≈2.4~4.0，窗口 [2.9, 4.2] 完整覆盖其近侧车身）
  右盲区：y 取负对称
  z ∈ [z_min, z_max] 为车身高度带（安装高 2.0 m 时对应世界高度约 0.3~1.6 m），
  同时滤掉地面（世界 z=0 → 传感器 z≈-2.0）与路侧 2 m 标志（传感器 z≈0）——
  该过滤依据与 carla_aeb_braking 模块的实测结论一致。

检测思路（侧向窗口 + 车身高度带 + 径向距离直方图聚类）：

  1) 按侧别取横向窗口内的点；
  2) 以径向距离 r = hypot(x, y) 分 bin_width(1m) 一箱计数；
  3) 取“点数 ≥ min_points 的最近一箱”的中心作为盲区目标距离——
     车身是连续表面（同距离多点），杂波是孤立点，聚类可稳健区分；
  4) 无目标时返回 valid_max 哨兵值。

决策为左右两条独立状态线（normal → warn，滞回阈值防临界抖动），
另有 approach 提前预警位：目标尚在盲区外但正快速接近时提前提醒，
两者解耦，便于变道辅助逻辑分别消费。
"""

from dataclasses import dataclass

import numpy as np

NORMAL = "normal"
WARN = "warn"


def parse_lidar_points(data):
    """CARLA LiDAR 原始字节 → (N,4) float32 数组（x, y, z, intensity）。"""
    pts = np.frombuffer(bytes(data), dtype=np.float32).reshape(-1, 4)
    return pts.copy()


def blind_zone_distance(points, side="left", x_min=-25.0, x_max=2.0,
                        y_near=2.9, y_far=4.2, z_min=-1.7, z_max=-0.4,
                        bin_width=1.0, min_points=6, valid_max=40.0):
    """侧向点云 → 盲区目标距离（米）。

    points: (N,4) 传感器系点云；side: "left" / "right"。

    过滤顺序：横向侧别窗口 → 纵向 x 窗口 → 车身高度带 → 径向直方图聚类。
    无目标返回 valid_max。
    """
    if side not in ("left", "right"):
        raise ValueError(f"side 必须是 left/right，得到 {side!r}")
    if len(points) == 0:
        return float(valid_max)
    ys = points[:, 1]
    if side == "left":
        sel = (ys >= y_near) & (ys <= y_far)
    else:
        sel = (ys <= -y_near) & (ys >= -y_far)
    sel &= (points[:, 0] >= x_min) & (points[:, 0] <= x_max)
    sel &= (points[:, 2] >= z_min) & (points[:, 2] <= z_max)
    p = points[sel]
    if len(p) == 0:
        return float(valid_max)
    r = np.hypot(p[:, 0], p[:, 1])
    bins = np.floor(r / bin_width).astype(int)
    counts = np.bincount(bins)
    good = np.flatnonzero(counts >= min_points)
    if len(good) == 0:
        return float(valid_max)
    nearest = int(good.min())
    center = nearest * bin_width + bin_width / 2.0
    return float(min(center, valid_max))


def estimate_closing(prev_d, curr_d, dt):
    """帧间差分接近速度（m/s，正值=正在接近）。"""
    if dt <= 0:
        return 0.0
    return (prev_d - curr_d) / dt


@dataclass
class BSDParams:
    """BSD 决策参数（与 config/bsd_params.yaml 一一对应）。"""
    warn_enter: float = 12.0    # 盲区目标近于此距离 → warn
    warn_exit: float = 15.0     # warn 中目标远于此距离 → 恢复 normal（滞回带）
    approach_enter: float = 25.0  # 目标近于此距离且正在接近 → approach 预警
    close_min: float = 0.5      # approach 的接近速度阈值（m/s）
    valid_max: float = 40.0


def bsd_decide(state, distance, closing, p=None):
    """单步决策（纯函数）：state ∈ {normal, warn} → (new_state, approach)。

    approach 与滞回状态解耦：d < approach_enter 且 closing > close_min
    即为 True，用于“有车正快速从后方逼近盲区”的提前预警。
    """
    p = p or BSDParams()
    if state == NORMAL:
        state = WARN if distance < p.warn_enter else NORMAL
    elif state == WARN:
        state = NORMAL if distance > p.warn_exit else WARN
    else:
        raise ValueError(f"非法状态 {state!r}，应为 normal/warn")
    approach = bool(distance < p.approach_enter and closing > p.close_min)
    return state, approach


class BSDController:
    """BSD 有状态封装：左右盲区各一条独立状态线。

    tick() 每周期喂入左右盲区距离（内部按帧差估计接近速度），
    返回每侧 {state, approach, distance, closing, event}；
    event 仅在状态跃迁时非空（如 "left_warn"），供上层报警去重。
    """

    def __init__(self, params=None):
        self.p = params or BSDParams()
        self._state = {"left": NORMAL, "right": NORMAL}
        self._prev_d = {"left": None, "right": None}

    def tick(self, left_d, right_d, dt=0.05):
        out = {}
        for side, d in (("left", float(left_d)), ("right", float(right_d))):
            prev = self._prev_d[side]
            closing = estimate_closing(prev, d, dt) if prev is not None else 0.0
            self._prev_d[side] = d
            old = self._state[side]
            state, approach = bsd_decide(old, d, closing, self.p)
            self._state[side] = state
            event = ""
            if old != WARN and state == WARN:
                event = f"{side}_warn"
            elif old == WARN and state == NORMAL:
                event = f"{side}_clear"
            out[side] = {
                "state": state,
                "approach": approach,
                "distance": d,
                "closing": float(closing),
                "event": event,
            }
        return out

    def reset(self):
        """复位到 normal 并清空测距历史。"""
        self._state = {"left": NORMAL, "right": NORMAL}
        self._prev_d = {"left": None, "right": None}
