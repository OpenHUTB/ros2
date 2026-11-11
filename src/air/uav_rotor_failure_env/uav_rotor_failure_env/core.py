# -*- coding: utf-8 -*-
"""多旋翼单桨失效后的悬停可行域（control allocation 视角）。

问题
----
四旋翼掉一个桨还能悬停吗？能带多少载？要不要自旋？换成六旋翼、八旋翼呢？
这些问题不需要飞控仿真，只需要回答一个线性规划：

    在 f_i ∈ [0, f_max]、失效桨 f_k = 0 的约束下，
    是否存在 f 使得  Σf_i = mg,  Σ y_i f_i = 0,  Σ x_i f_i = 0 ？

其中后两个等式就是"姿态力矩为零"。把可行域的顶点枚举一遍，就能得到
**该失效模式下能产生的最大总推力**，也就是"还能吊多重的临界值"。

模型
----
* 每个桨只产生沿机体 +z 的推力 f_i，力臂 (x_i, y_i) 在机体 x-y 平面内；
* 偏航力矩 Mz = Σ s_i·κ·f_i（s_i 为旋向，κ 为"单位推力的反扭矩"系数）；
  悬停时 Mz 不要求为零——车体会自旋，稳态自旋靠气动阻尼平衡；
* 忽略桨间干扰、地面效应、姿态角限制（倾斜会让情况更宽松，见文档"边界"）。

顶点枚举为什么够
----------------
可行域是 {f ≥ 0, f ≤ f_max, A·f = 0}（A 为 2 行的力矩约束）——
一个凸多面体，维数 n−2。线性目标（总推力）的最大值必在顶点取得，
而顶点就是"n−2 个变量取边界值"的那些组合，C(n,2) 个，逐个验证即可。
n ≤ 8 时最多 28 种组合，无需通用 LP 求解器。

单位：N、m、N·m。
"""

import itertools
import math
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Sequence, Set, Tuple

import numpy as np

G = 9.81


@dataclass
class Rotor:
    """一个桨：位置（机体 x-y，m）、旋向（+1 逆时针 / −1 顺时针）。"""

    x: float
    y: float
    spin: int = 1


@dataclass
class Config:
    """整机配置：桨布局 + 单桨最大推力 + 反扭矩系数 + 机重。"""

    name: str
    rotors: List[Rotor]
    thrust_max: float = 10.0      # 单桨最大推力 N
    kappa: float = 0.02           # 反扭矩系数 N·m / N（典型小桨 0.015~0.03）
    mass: float = 2.0             # kg

    def n(self) -> int:
        return len(self.rotors)

    def positions(self) -> np.ndarray:
        return np.array([[r.x, r.y] for r in self.rotors], dtype=float)

    def spins(self) -> np.ndarray:
        return np.array([r.spin for r in self.rotors], dtype=float)

    def nominal_thrust(self) -> float:
        """全部桨满推力的总推力（理论上限，通常受姿态约束而达不到）。"""
        return self.n() * self.thrust_max


# --------------------------------------------------------------------- 布局
def quad_x(arm: float = 0.25, **kw) -> Config:
    """四旋翼 X 型：四个桨在 (±d, ±d)，d = arm/√2。"""
    d = arm / math.sqrt(2.0)
    rotors = [Rotor(d, -d, +1), Rotor(d, d, -1), Rotor(-d, d, +1), Rotor(-d, -d, -1)]
    return Config("quad-X", rotors, **kw)


def hexa(radius: float = 0.30, **kw) -> Config:
    """六旋翼：等角分布，旋向交替。"""
    rotors = []
    for i in range(6):
        th = 2 * math.pi * i / 6.0
        rotors.append(Rotor(radius * math.cos(th), radius * math.sin(th),
                            +1 if i % 2 == 0 else -1))
    return Config("hexa", rotors, **kw)


def octo(radius: float = 0.35, **kw) -> Config:
    """八旋翼：等角分布，旋向交替。"""
    rotors = []
    for i in range(8):
        th = 2 * math.pi * i / 8.0
        rotors.append(Rotor(radius * math.cos(th), radius * math.sin(th),
                            +1 if i % 2 == 0 else -1))
    return Config("octo", rotors, **kw)


def nominal_mass_limit(cfg: Config) -> float:
    """没有姿态约束时的载重上限（全部桨可用）。"""
    return cfg.nominal_thrust() / G


# --------------------------------------------------------------------- 求解
def _feasible_vertices(cfg: Config, failed: Set[int]) -> List[np.ndarray]:
    """枚举可行域顶点：n−2 个变量压到下界/上界，解剩下 2 个。

    约束：Σ f = ? 不参与枚举（作为目标），只有 Mx=0、My=0 两条等式：
        Σ y_i f_i = 0,  Σ (−x_i) f_i = 0
    """
    n = cfg.n()
    pos = cfg.positions()
    A = np.vstack([pos[:, 1], -pos[:, 0]])       # 2×n：Mx、My
    alive = [i for i in range(n) if i not in failed]
    fixed_free = [i for i in alive]

    verts = []
    # 选 2 个"自由"变量，其余 alive 变量取 0 或 f_max（失效桨恒为 0）
    for free in itertools.combinations(fixed_free, 2):
        others = [i for i in fixed_free if i not in free]
        for bits in itertools.product((0.0, cfg.thrust_max), repeat=len(others)):
            f = np.zeros(n)
            for i, v in zip(others, bits):
                f[i] = v
            for i in failed:
                f[i] = 0.0
            A_free = A[:, list(free)]            # 2×2
            if abs(np.linalg.det(A_free)) < 1e-12:
                continue
            rhs = -A @ f
            try:
                sol = np.linalg.solve(A_free, rhs)
            except np.linalg.LinAlgError:
                continue
            if np.any(sol < -1e-9) or np.any(sol > cfg.thrust_max + 1e-9):
                continue
            for i, v in zip(free, sol):
                f[i] = min(max(v, 0.0), cfg.thrust_max)
            verts.append(f)
    return verts


def max_hover(cfg: Config, failed: Iterable[int] = ()) -> Dict[str, object]:
    """该失效模式下，能产生的**最大零力矩总推力**及所需桨推力。

    返回 total_thrust、mass_limit、margin（相对额定）、yaw_torque、feasible。
    """
    failed = set(failed)
    if failed and (min(failed) < 0 or max(failed) >= cfg.n()):
        raise ValueError("失效桨编号越界")

    verts = _feasible_vertices(cfg, failed)
    if not verts:
        return {"feasible": False, "total_thrust": 0.0, "mass_limit": 0.0,
                "margin": 0.0, "yaw_torque": 0.0, "thrusts": [0.0] * cfg.n(),
                "rank_after": 0}

    totals = [float(np.sum(f)) for f in verts]
    best_total = max(totals)
    # 主目标：总推力最大；次目标：在等推力顶点里取 |Mz| 最小（能不自旋就不自旋）
    cand = [f for f, tot in zip(verts, totals) if tot >= best_total - 1e-9]
    spins = cfg.spins()
    best = min(cand, key=lambda f: abs(float(np.dot(spins, f))))
    yaw_free = any(abs(float(np.dot(spins, f))) < 1e-9 for f in cand)
    mz = float(cfg.kappa * np.dot(spins, best))
    nominal = cfg.nominal_thrust()
    return {
        "feasible": True,
        "total_thrust": best_total,
        "mass_limit": best_total / G,
        "margin": best_total / nominal,
        "yaw_torque": mz,
        "thrusts": best.tolist(),
        "yaw_free": yaw_free,
        "alive": cfg.n() - len(failed),
    }


def max_hover_zero_yaw(cfg: Config, failed: Iterable[int] = ()) -> Dict[str, float]:
    """附加"零偏航力矩"约束时的最大悬停推力（不能靠自旋抵消）。

    约束从 2 条（Mx=My=0）变成 3 条（再加 Mz=0），可行域维数 n−3，
    顶点是 n−3 个变量取边界、3 个自由变量解 3×3 方程组。
    """
    failed = set(failed)
    pos = cfg.positions()
    spins = cfg.spins()
    A = np.vstack([pos[:, 1], -pos[:, 0], cfg.kappa * spins])   # 3×n
    alive = [i for i in range(cfg.n()) if i not in failed]
    best_total, best_f = 0.0, None
    for free in itertools.combinations(alive, 3):
        others = [i for i in alive if i not in free]
        for bits in itertools.product((0.0, cfg.thrust_max), repeat=len(others)):
            f = np.zeros(cfg.n())
            for i, v in zip(others, bits):
                f[i] = v
            A_free = A[:, list(free)]
            if abs(np.linalg.det(A_free)) < 1e-15:
                continue
            try:
                sol = np.linalg.solve(A_free, -A @ f)
            except np.linalg.LinAlgError:
                continue
            if np.any(sol < -1e-9) or np.any(sol > cfg.thrust_max + 1e-9):
                continue
            for i, v in zip(free, sol):
                f[i] = min(max(v, 0.0), cfg.thrust_max)
            tot = float(np.sum(f))
            if tot > best_total:
                best_total, best_f = tot, f
    if best_f is None:
        return {"feasible": False, "total_thrust": 0.0, "mass_limit": 0.0,
                "margin": 0.0, "thrusts": [0.0] * cfg.n()}
    return {"feasible": True, "total_thrust": best_total,
            "mass_limit": best_total / G,
            "margin": best_total / cfg.nominal_thrust(),
            "thrusts": best_f.tolist()}


def wrench_rank(cfg: Config, failed: Iterable[int] = ()) -> int:
    """剩余桨对 (Fz, Mx, My, Mz) 四个方向的秩——能看出失效后丢了几个自由度。"""
    failed = set(failed)
    pos = cfg.positions()
    spins = cfg.spins()
    cols = []
    for i in range(cfg.n()):
        if i in failed:
            continue
        cols.append([1.0, pos[i, 1], -pos[i, 0], cfg.kappa * spins[i]])
    if not cols:
        return 0
    return int(np.linalg.matrix_rank(np.array(cols).T))


def hover_margin_table(configs: Sequence[Config]) -> List[Dict[str, object]]:
    """对每种机型，扫描"无失效 / 单桨失效"两种情况的悬停能力。"""
    rows = []
    for cfg in configs:
        base = max_hover(cfg, failed=())
        rows.append({"config": cfg.name, "case": "正常", "n": cfg.n(),
                     "total": base["total_thrust"], "mass": base["mass_limit"],
                     "margin": base["margin"], "yaw": base["yaw_torque"],
                     "rank": wrench_rank(cfg)})
        for k in range(cfg.n()):
            r = max_hover(cfg, failed={k})
            rows.append({"config": cfg.name, "case": "失效 #%d" % k, "n": cfg.n(),
                         "total": r["total_thrust"], "mass": r["mass_limit"],
                         "margin": r["margin"], "yaw": r["yaw_torque"],
                         "rank": wrench_rank(cfg, failed={k})})
    return rows


def spin_rate_for(mz: float, drag_coeff: float = 0.004) -> float:
    """稳态自旋角速度：Mz = drag_coeff·ω² 的近似解（rad/s）。

    偏航阻尼常用 ω|ω| 形式；这里给量级估计，不是精确气动模型。
    """
    if drag_coeff <= 0:
        raise ValueError("drag_coeff 必须为正")
    return math.sqrt(abs(mz) / drag_coeff)
