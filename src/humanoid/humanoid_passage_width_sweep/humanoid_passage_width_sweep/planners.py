# -*- coding: utf-8 -*-
"""局部规划器：MPPI 与反应式斥力法，各配「足迹感知 / 质点假设」两种膨胀半径。

实验设计是 2x2 单变量对比::

              膨胀 0.38 m（含足迹+裕度）   膨胀 0.18 m（质点假设）
    MPPI             mppi_footprint              mppi_point
    反应式斥力        reactive_footprint           reactive_point

这样「算法」和「足迹建模」两个因素可以分开看：如果同一算法只改膨胀半径就
从全失败变成全通过，那结论就落在足迹建模上，而不是算法优劣上。

纯 numpy，不依赖 MuJoCo / ROS，便于离线单测。
"""

import numpy as np

import humanoid_passage_width_sweep.config as config


def _clean(scan_points):
    """去掉 NaN 行（未命中的射线）。"""
    if scan_points is None or len(scan_points) == 0:
        return np.zeros((0, 2))
    p = np.asarray(scan_points, dtype=float).reshape(-1, 2)
    return p[np.isfinite(p).all(axis=1)]


def _wrap(a):
    """把角度折到 (-pi, pi]。"""
    return (a + np.pi) % (2 * np.pi) - np.pi


class PlannerBase:
    """规划器基类。"""

    name = "base"

    def __init__(self, goal=config.GOAL, inflate=config.INFLATE_FOOTPRINT):
        self.goal = np.asarray(goal, dtype=float)
        self.inflate = float(inflate)

    def reset(self):
        """清空内部状态（每个实验开始前调用）。"""

    def compute(self, x, y, yaw, scan_points):
        raise NotImplementedError


class MppiPlanner(PlannerBase):
    """采样型 MPC（MPPI）。

    每次采样 ``samples`` 条 ``(v, w)`` 序列，无约束单轮车前向推演 ``horizon`` 步，
    按「障碍 / 进度 / 朝向」三项代价 softmax 加权，取加权均值的第一段控制量。

    障碍代价用**到最近激光点的距离**与 ``inflate`` 比较：``inflate`` 就是本模块
    唯一被操纵的变量 —— 它应当等于「足迹半径 + 安全裕度」，若按质点假设取小值，
    规划器就会把「贴着自己的身体擦过去」判成可行。
    """

    def __init__(self, goal=config.GOAL, inflate=config.INFLATE_FOOTPRINT,
                 name="MPPI", samples=config.MPPI_SAMPLES,
                 horizon=config.MPPI_HORIZON, dt=config.MPPI_DT,
                 temp=config.MPPI_TEMP, seed=config.SEED):
        super().__init__(goal, inflate)
        self.name = name
        self.samples = samples
        self.horizon = horizon
        self.dt = dt
        self.temp = temp
        self.seed = seed
        self.rng = np.random.default_rng(seed)
        self.nominal = (0.0, 0.0)

    def reset(self):
        self.nominal = (0.0, 0.0)
        self.rng = np.random.default_rng(self.seed)

    def _rollout(self, x, y, yaw, vs, ws):
        S, H = vs.shape
        traj = np.empty((S, H, 2))
        px = np.full(S, x)
        py = np.full(S, y)
        th = np.full(S, yaw)
        for k in range(H):
            th = th + ws[:, k] * self.dt
            px = px + vs[:, k] * np.cos(th) * self.dt
            py = py + vs[:, k] * np.sin(th) * self.dt
            traj[:, k, 0] = px
            traj[:, k, 1] = py
        return traj, th

    def compute(self, x, y, yaw, scan_points):
        pts = _clean(scan_points)
        S, H = self.samples, self.horizon
        v0, w0 = self.nominal
        vs = np.clip(v0 + self.rng.normal(0.0, config.MPPI_SIGMA_V, (S, H)),
                     0.0, config.MAX_V)
        ws = np.clip(w0 + self.rng.normal(0.0, config.MPPI_SIGMA_W, (S, H)),
                     -config.MAX_W, config.MAX_W)
        # 保留一条「直冲目标」的确定性采样，避免纯随机在窄门里全军覆没
        vs[0, :] = config.MAX_V
        ws[0, :] = float(np.clip(_wrap(np.arctan2(self.goal[1] - y,
                                                  self.goal[0] - x) - yaw) * 2.0,
                                 -config.MAX_W, config.MAX_W))

        traj, th_end = self._rollout(x, y, yaw, vs, ws)

        if len(pts):
            d2 = ((traj[:, :, None, :] - pts[None, None, :, :]) ** 2).sum(-1)
            dmin = np.sqrt(d2.min(axis=2))
            obs = np.square(np.maximum(self.inflate - dmin, 0.0)).sum(axis=1)
        else:
            obs = np.zeros(S)

        end = traj[:, -1, :]
        prog = np.linalg.norm(self.goal[None, :] - end, axis=1)
        want = np.arctan2(self.goal[1] - end[:, 1], self.goal[0] - end[:, 0])
        head = np.abs(_wrap(want - th_end))

        cost = (config.MPPI_W_OBS * obs + config.MPPI_W_PROGRESS * prog
                + config.MPPI_W_HEADING * head)
        cost = cost - cost.min()
        wgt = np.exp(-cost / max(self.temp, 1e-6))
        s = wgt.sum()
        wgt = wgt / s if s > 1e-12 else np.full(S, 1.0 / S)

        v = float(np.clip((wgt * vs[:, 0]).sum(), 0.0, config.MAX_V))
        w = float(np.clip((wgt * ws[:, 0]).sum(), -config.MAX_W, config.MAX_W))
        self.nominal = (v, w)
        return v, w


class ReactivePlanner(PlannerBase):
    """反应式斥力法（目标引力 + 障碍斥力，APF 的单层版本）。

    与 MPPI 共用同一个 ``inflate``，因此可以直接对比「换成更简单的算法，
    结论是否仍然取决于足迹建模」。
    """

    def __init__(self, goal=config.GOAL, inflate=config.INFLATE_FOOTPRINT,
                 name="reactive", v_des=config.MAX_V, k_rep=0.55):
        super().__init__(goal, inflate)
        self.name = name
        self.v_des = v_des
        self.k_rep = k_rep
        self.k_tang = 0.9

    def compute(self, x, y, yaw, scan_points):
        pts = _clean(scan_points)
        pos = np.array([x, y])
        to_goal = self.goal - pos
        dg = float(np.linalg.norm(to_goal))
        if dg < 1e-6:
            return 0.0, 0.0
        heading = to_goal / dg
        push = np.zeros(2)
        if len(pts):
            d = np.linalg.norm(pts - pos, axis=1)
            near = d < self.inflate
            if near.any():
                dn = np.maximum(d[near], 1e-3)
                away = (pos - pts[near]) / dn[:, None]
                mag = self.k_rep * (self.inflate - d[near]) / self.inflate
                push = (away * mag[:, None]).sum(axis=0)
        # 纯径向斥力会与目标引力正面顶牛：机器人被挡在障碍前反复抖动，
        # 直到贴上去（实测 min_clear≈-0.002）。加一个切向分量，让它沿侧向滑过去。
        des = heading + push
        if np.linalg.norm(push) > 1e-6:
            tang = np.array([-push[1], push[0]])
            if float(np.dot(tang, heading)) < 0.0:
                tang = -tang
            des = des + self.k_tang * tang
        ang = float(np.arctan2(des[1], des[0]))
        w = float(np.clip(2.0 * _wrap(ang - yaw), -config.MAX_W, config.MAX_W))
        v = self.v_des * (1.0 - 0.6 * min(abs(w) / config.MAX_W, 1.0))
        v = min(v, dg)
        return float(np.clip(v, 0.0, config.MAX_V)), w


# variant 名 -> (类, 膨胀半径, 显示名)
VARIANTS = {
    "mppi_footprint": (MppiPlanner, config.INFLATE_FOOTPRINT, "MPPI（足迹感知）"),
    "mppi_point": (MppiPlanner, config.INFLATE_POINT, "MPPI（质点假设）"),
    "reactive_footprint": (ReactivePlanner, config.INFLATE_FOOTPRINT, "斥力法（足迹感知）"),
    "reactive_point": (ReactivePlanner, config.INFLATE_POINT, "斥力法（质点假设）"),
}
VARIANT_ORDER = ("mppi_footprint", "mppi_point",
                 "reactive_footprint", "reactive_point")


def make(variant):
    """按 variant 名构造规划器。"""
    key = str(variant).lower()
    if key not in VARIANTS:
        raise ValueError(f"未知变体 {variant!r}，可选：{list(VARIANTS)}")
    cls, inflate, label = VARIANTS[key]
    return cls(inflate=inflate, name=label)


def label_of(variant):
    """变体对应的中文显示名。"""
    return VARIANTS[str(variant).lower()][2]