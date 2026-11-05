#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""城市动态交通避障共享逻辑：观测构造 / 动作映射 / 车辆特征 / 纯 numpy 推理.

训练环境与 CarlaAir 部署节点**共用本模块**，保证两侧观测与动作接口完全一致。

坐标与接口约定（与 ``uav_ppo_nav`` 保持一致）
--------------------------------------------
* 全部使用世界系 ENU（东-北-天），因为 CarlaAir 里 SimpleFlight 的偏航保持不生效；
* 动作 ``[vx, vy, vz, yaw_rate]`` 为世界系速度 + 偏航角速度，归一化到 ``[-1, 1]``；
* 观测 = 16 扇区激光直方图 + 目标相对位置(3) + 世界系速度(3)
  + **最近 3 辆车的相对位置与速度(12)**。

动态交通特征
------------
静态障碍已经体现在激光直方图里，但**移动车辆**还需要显式给出相对速度，
否则策略只能看到"当前距离"，无法预判 1 秒后的位置——这正是本模块与
``uav_ppo_nav``（纯静态障碍）的区别。
"""
from __future__ import annotations

import math
from typing import Dict, Iterable, List, Sequence

import numpy as np

# ---- 常量 ----
N_SECTORS = 16          # 激光雷达水平扇形数量（世界系固定方向，扇形 0 = 正东）
MAX_RANGE = 12.0        # 激光雷达有效距离 (m)
MAX_HORIZ_SPEED = 3.0   # 水平速度上限 (m/s)
MAX_VERT_SPEED = 1.5    # 垂直速度上限 (m/s)
MAX_YAW_RATE = 1.0      # 偏航角速度上限 (rad/s)
GOAL_SCALE = 8.0        # 目标相对位置归一化尺度 (m)

N_TRAFFIC = 3           # 观测里显式建模的最近车辆数量
TRAFFIC_POS_SCALE = 12.0
TRAFFIC_VEL_SCALE = 10.0

OBS_DIM_BASE = N_SECTORS + 3 + 3          # 22：不含交通特征
OBS_DIM = OBS_DIM_BASE + N_TRAFFIC * 4    # 34：含交通特征
ACT_DIM = 4

_Z_BAND = 1.5           # 直方图只统计无人机高度 ± 该范围的障碍点


def lidar_points_to_histogram(points_world, drone_xy=(0.0, 0.0), drone_z=0.0,
                              n_sectors=N_SECTORS, max_range=MAX_RANGE) -> np.ndarray:
    """世界系点云 (N, 3) ENU -> 水平扇形直方图 (n_sectors,).

    以无人机所在位置为原点，按世界系固定方向分扇（扇形 0 = 正东），
    每个扇形返回最近障碍距离，归一化到 [0, 1]（1 = 无遮挡）。
    只统计无人机高度 ``±_Z_BAND`` 内的点，避免地面被当成障碍。
    """
    pts = np.asarray(points_world, dtype=np.float64).reshape(-1, 3)
    hist = np.ones(n_sectors, dtype=np.float32)
    if pts.shape[0] == 0:
        return hist
    pts = pts[np.abs(pts[:, 2] - float(drone_z)) < _Z_BAND]
    if pts.shape[0] == 0:
        return hist
    dx = pts[:, 0] - float(drone_xy[0])
    dy = pts[:, 1] - float(drone_xy[1])
    r = np.hypot(dx, dy)
    ang = np.arctan2(dy, dx)
    sector = 2.0 * math.pi / n_sectors
    idx = np.floor((ang + sector / 2.0) / sector).astype(np.int64) % n_sectors
    for i in range(n_sectors):
        mask = idx == i
        if mask.any():
            hist[i] = float(np.clip(r[mask].min() / max_range, 0.0, 1.0))
    return hist


def traffic_features(cars: Iterable[Sequence[float]], drone_xy=(0.0, 0.0),
                     drone_vel=(0.0, 0.0), k: int = N_TRAFFIC,
                     pos_scale: float = TRAFFIC_POS_SCALE,
                     vel_scale: float = TRAFFIC_VEL_SCALE) -> np.ndarray:
    """最近 k 辆车的 (相对位置 2 + 相对速度 2) 特征，缺失补零.

    ``cars`` 每项为 ``(x, y, vx, vy)``（米、米/秒，世界系 ENU）。
    车辆按到无人机水平距离升序取前 k 辆；不足 k 辆时用零向量补齐，
    这样观测维度固定，且"没有车"与"车在原点"不会混淆（原点相对距离为 0，
    而车辆实际不会与无人机同位置——训练时以碰撞终止保证）。
    """
    out = np.zeros(k * 4, dtype=np.float32)
    dx = float(drone_xy[0])
    dy = float(drone_xy[1])
    vx = float(drone_vel[0])
    vy = float(drone_vel[1])
    ranked = []
    for car in cars:
        cx, cy, cvx, cvy = (float(car[0]), float(car[1]), float(car[2]), float(car[3]))
        dist = math.hypot(cx - dx, cy - dy)
        ranked.append((dist, cx, cy, cvx, cvy))
    ranked.sort(key=lambda item: item[0])
    for i, (_, cx, cy, cvx, cvy) in enumerate(ranked[:k]):
        out[4 * i + 0] = np.clip((cx - dx) / pos_scale, -1.0, 1.0)
        out[4 * i + 1] = np.clip((cy - dy) / pos_scale, -1.0, 1.0)
        out[4 * i + 2] = np.clip((cvx - vx) / vel_scale, -1.0, 1.0)
        out[4 * i + 3] = np.clip((cvy - vy) / vel_scale, -1.0, 1.0)
    return out


def build_observation(hist, goal_rel, vel_world, traffic=None,
                      include_traffic: bool = True) -> np.ndarray:
    """拼成观测（34 维含交通；``include_traffic=False`` 时为 22 维）.

    hist       : (n_sectors,) 扇形直方图 [0,1]
    goal_rel   : (3,) 目标相对位置（世界系，米）
    vel_world  : (3,) 世界系速度（米/秒）
    traffic    : ``traffic_features()`` 的输出 (12,)；``include_traffic`` 为真时必需
    """
    hist = np.asarray(hist, dtype=np.float32).reshape(-1)
    goal = np.asarray(goal_rel, dtype=np.float32).reshape(-1)[:3] / GOAL_SCALE
    vel = np.asarray(vel_world, dtype=np.float32).reshape(-1)[:3]
    vel = np.array([vel[0] / MAX_HORIZ_SPEED,
                    vel[1] / MAX_HORIZ_SPEED,
                    vel[2] / MAX_VERT_SPEED], dtype=np.float32)
    parts = [hist, goal, vel]
    if include_traffic:
        feat = np.zeros(N_TRAFFIC * 4, dtype=np.float32) if traffic is None \
            else np.asarray(traffic, dtype=np.float32).reshape(-1)[:N_TRAFFIC * 4]
        parts.append(feat)
    return np.concatenate(parts).astype(np.float32)


def observation_dim(include_traffic: bool = True) -> int:
    return OBS_DIM if include_traffic else OBS_DIM_BASE


def action_to_velocity(action) -> tuple:
    """[-1,1]^4 动作 -> (vx, vy, vz, yaw_rate)：世界系 ENU 速度 + 偏航角速度."""
    a = np.clip(np.asarray(action, dtype=np.float64).reshape(-1)[:4], -1.0, 1.0)
    return (float(a[0] * MAX_HORIZ_SPEED),
            float(a[1] * MAX_HORIZ_SPEED),
            float(a[2] * MAX_VERT_SPEED),
            float(a[3] * MAX_YAW_RATE))


def velocity_to_action(vx, vy, vz=0.0, yaw_rate=0.0) -> np.ndarray:
    """世界系速度 -> 归一化动作（评测脚本里的基线控制器用）."""
    return np.clip(np.array([vx / MAX_HORIZ_SPEED,
                             vy / MAX_HORIZ_SPEED,
                             vz / MAX_VERT_SPEED,
                             yaw_rate / MAX_YAW_RATE], dtype=np.float32), -1.0, 1.0)


def time_to_collision(pos_xy, vel_xy, car_xy, car_vel_xy, max_time: float = 2.0):
    """二维匀速模型下的碰撞时间 (s)；不接近或已分离时返回 ``None``.

    相对运动 ``r = car - drone``，相对速度 ``v = car_vel - drone_vel``。
    最近接近时刻 ``t* = -(r·v)/|v|²``；``0 < t* <= max_time`` 且最近距离
    小于无人机半径与车半径之和时才认为有碰撞风险。
    """
    rx = float(car_xy[0]) - float(pos_xy[0])
    ry = float(car_xy[1]) - float(pos_xy[1])
    vx = float(car_vel_xy[0]) - float(vel_xy[0])
    vy = float(car_vel_xy[1]) - float(vel_xy[1])
    vv = vx * vx + vy * vy
    if vv < 1e-9:
        return None
    t = -(rx * vx + ry * vy) / vv
    if t < 0.0 or t > max_time:
        return None
    return float(t)


class MlpPolicy(object):
    """训练好的 MLP 策略的纯 numpy 推理（部署侧无需 torch / sb3）."""

    def __init__(self, d: Dict[str, np.ndarray]):
        self.n = int(d["n_layers"])
        self.hidden = [(np.asarray(d["h%d_w" % i], dtype=np.float32),
                        np.asarray(d["h%d_b" % i], dtype=np.float32))
                       for i in range(self.n)]
        self.out_w = np.asarray(d["out_w"], dtype=np.float32)
        self.out_b = np.asarray(d["out_b"], dtype=np.float32)

    @classmethod
    def from_npz(cls, path: str) -> "MlpPolicy":
        return cls(np.load(path))

    def forward(self, obs) -> np.ndarray:
        x = np.asarray(obs, dtype=np.float32).reshape(-1)
        for w, b in self.hidden:
            x = np.tanh(w @ x + b)
        return np.clip(self.out_w @ x + self.out_b, -1.0, 1.0).astype(np.float32)


def extract_sb3_weights(model) -> Dict[str, np.ndarray]:
    """从 SB3 PPO 模型提取 MLP 权重 -> dict（保存为 .npz 供部署侧加载）."""
    import torch  # noqa: WPS433 (训练期按需导入)

    p = model.policy
    hidden = []
    for m in p.mlp_extractor.policy_net:
        if isinstance(m, torch.nn.Linear):
            hidden.append((m.weight.detach().cpu().numpy(),
                           m.bias.detach().cpu().numpy()))
    out = p.action_net
    d = {"n_layers": len(hidden)}
    for i, (w, b) in enumerate(hidden):
        d["h%d_w" % i] = w
        d["h%d_b" % i] = b
    d["out_w"] = out.weight.detach().cpu().numpy()
    d["out_b"] = out.bias.detach().cpu().numpy()
    return d
