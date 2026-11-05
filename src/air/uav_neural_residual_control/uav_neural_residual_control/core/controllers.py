#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""四个跟踪控制器，共用同一套内环与同一份标称参数。

    pid       位置 PID（含积分）            —— 经典基线，靠积分项硬扛常值扰动
    geo       SE(3) 几何控制（无积分）      —— 强基线，论文里常用的对照
    geo_res   几何 + 离线残差网络            —— 本模块提出
    geo_rls   几何 + 离线残差 + 在线 RLS     —— 本模块提出

四者**只差在 ``a_cmd`` 怎么算**：内环姿态控制、混控、限倾角、饱和处理全部复用
底座的 ``GeometricController``，因此性能差异可以完全归因于外环/残差补偿，
不会混入实现差异。做法是把底座的 kp/kd 置零，让它只当"加速度→推力/力矩"的
执行器用。
"""
from __future__ import annotations

import copy
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

import numpy as np

from .attitude import GeometricController, QuadParams, QuadrotorState, mixer, quat_to_mat
from .residual import ResidualNet
from .rls import RecursiveLeastSquares
from .trajectories import Trajectory

__all__ = ["Gains", "Tracker", "PIDTracker", "GeometricTracker",
           "ResidualTracker", "ResidualRLSTracker", "TRACKERS", "make_tracker"]


@dataclass
class Gains:
    """全部沿用底座的符号约定：误差取 ``当前值 − 期望值``，增益取负。

    ``kp/kd`` 只给 PID 用（几何控制器走底座默认增益，见 ``GeometricTracker``）。
    """

    #: 外环（位置/速度/积分），PID 用
    kp: Tuple[float, float, float] = (-16.0, -16.0, -20.0)
    kd: Tuple[float, float, float] = (-8.0, -8.0, -9.0)
    ki: Tuple[float, float, float] = (-4.0, -4.0, -6.0)
    #: 内环姿态（几何误差 PD），四个控制器共用。
    #: 角速率阻尼取底座默认值的 2 倍：底座是按 0.95 kg 空机整定的，挂到
    #: +0.75 kg 时转动惯量涨约 1.6 倍、姿态环阻尼比从 0.41 掉到 0.32，
    #: 起步的 35° 倾角阶跃会过冲到 50° 以上并触发翻机（实测倾角 177°）。
    #: 阻尼加倍后 0.95~1.7 kg 全载荷域都稳定，且跟踪误差随载荷单调。
    kr: Tuple[float, float, float] = (-1.5, -1.5, -0.6)
    kw: Tuple[float, float, float] = (-0.20, -0.20, -0.14)
    #: 积分限幅，防饱和期积分飞掉
    i_limit: float = 2.0


# --------------------------------------------------------------------------- #
class Tracker:
    """控制器基类：一次 ``act`` 一步，``observe`` 回收实测加速度。

    分成两步是因为残差网络的**输入**要包含上一步下达的加速度、**标签**要包含
    本步的实测加速度，二者之间隔了一次物理步进。
    """

    name = "tracker"
    has_residual = False
    online = False

    def __init__(self, plant, traj: Trajectory, gains: Optional[Gains] = None,
                 params: Optional[QuadParams] = None):
        self.plant = plant
        self.traj = traj
        self.g = gains or Gains()
        self.params = params if params is not None else plant.nominal_params
        # 底座几何控制器退化为纯执行器：外环 kp/kd 置零，只保留姿态环
        self.geo = GeometricController(self.params, kp=(0.0, 0.0, 0.0),
                                       kd=(0.0, 0.0, 0.0),
                                       kr=self.g.kr, kw=self.g.kw)
        self.reset()

    # ------------------------------------------------------------ 生命周期
    def reset(self) -> None:
        self.t = 0.0
        self.last_thrust = np.zeros(4)
        self.last_a_cmd = np.zeros(3)        # 世界系，未含残差修正
        self.last_residual = np.zeros(3)     # 世界系，网络给出的修正量
        self.last_delta = np.zeros(3)        # 机体系，实测残差（仅用于记录）
        self.last_residual_b = np.zeros(3)   # 机体系，网络预测的残差
        self.phi: Optional[np.ndarray] = None
        #: 采集数据时叠加的激励加速度（世界系），控制器本身运行时保持 0
        self.excitation = np.zeros(3)
        self._quat = np.array([1.0, 0.0, 0.0, 0.0])
        self._R = np.eye(3)
        self._integral = np.zeros(3)
        self._saturated = False

    # ------------------------------------------------------------ 外环
    def desired_accel(self, st, pos_d, vel_d, acc_d) -> np.ndarray:
        """子类实现：返回世界系期望加速度（未扣残差）。"""
        raise NotImplementedError

    def residual_world(self, st, a_cmd: np.ndarray) -> np.ndarray:
        """子类实现：返回世界系残差补偿量，基类为 0。"""
        return np.zeros(3)

    # ------------------------------------------------------------ 一步
    def act(self, t: float) -> np.ndarray:
        st = self.plant.state()
        pos_d, vel_d, acc_d, yaw_d, _ = self.traj.eval(t)

        self._quat = st.quat.copy()
        self._R = quat_to_mat(st.quat)
        a_cmd = self.desired_accel(st, pos_d, vel_d, acc_d) + self.excitation
        self.last_a_cmd = a_cmd

        # 残差网络在机体系里工作：网络输出 R·Δa_b，因为标称模型的偏差
        # (阻力、载荷) 都随姿态一起旋转，机体系表示才能跨姿态泛化。
        r_w = self.residual_world(st, a_cmd)
        self.last_residual = r_w
        self.last_residual_b = self._R.T @ r_w

        T, tau = self.geo.compute(st, pos_d, vel_d, a_cmd - r_w, yaw_d)
        u = mixer(T, tau, self.params)
        self.last_thrust = u
        self._saturated = bool(np.any(u > 0.98 * self.params.max_thrust))
        self.t = t
        return u

    def observe(self) -> None:
        """物理步进之后调用，回收本步的实测残差。"""
        d_w = self.plant.last_acc - self.plant.nominal_accel(quat=self._quat)
        self.last_delta = self._R.T @ d_w

    # ------------------------------------------------------------ 诊断
    def diagnostics(self) -> Dict[str, float]:
        return {"saturated": float(self._saturated),
                "residual_norm": float(np.linalg.norm(self.last_residual))}


# --------------------------------------------------------------------------- #
class PIDTracker(Tracker):
    """位置 PID + 姿态 PD。

    积分项是它唯一比几何控制器强的地方，也是它唯一的软肋：常值扰动（载荷）
    可以靠积分消掉，但积分同时把**阵风**的低频分量也当成了需要消除的偏差，
    于是姿态持续追风，抗扰动的相位裕度被吃掉。这正是下面残差方案要替代的东西。
    """

    name = "pid"

    def desired_accel(self, st, pos_d, vel_d, acc_d) -> np.ndarray:
        g = self.g
        e_p = st.pos - pos_d
        e_v = st.vel - vel_d
        # 条件积分：推力饱和时停止积分，否则饱和期积分会一路累到限幅
        if not self._saturated:
            self._integral = self._integral + e_p * self.plant.dt
            if g.i_limit > 0.0:
                np.clip(self._integral, -g.i_limit, g.i_limit, out=self._integral)
        return acc_d + np.asarray(g.kp) * e_p + np.asarray(g.kd) * e_v \
            + np.asarray(g.ki) * self._integral


# --------------------------------------------------------------------------- #
class GeometricTracker(Tracker):
    """底座的 SE(3) 几何控制器，使用标称参数、无积分、无残差补偿。

    这是最公平的对照：同一位姿环、同一套增益，唯一差别是它把载荷与气动偏差
    全部留给反馈去扛，稳态必然留下与扰动同量级的静差。
    """

    name = "geo"

    def __init__(self, plant, traj, gains=None, params=None, kp=None, kd=None):
        super().__init__(plant, traj, gains, params)
        kp = kp if kp is not None else (-16.0, -16.0, -20.0)
        kd = kd if kd is not None else (-8.0, -8.0, -9.0)
        self.geo = GeometricController(self.params, kp=kp, kd=kd,
                                       kr=self.g.kr, kw=self.g.kw)

    def desired_accel(self, st, pos_d, vel_d, acc_d) -> np.ndarray:
        # 反馈已由底座控制器内部完成，这里只提供前馈，避免重复计入
        return acc_d


# --------------------------------------------------------------------------- #
class ResidualTracker(Tracker):
    """几何控制器 + 离线训练的残差网络。

    ``a_cmd`` 依旧由底座控制器内部的 PD 给出；本类只做一件事：把网络预测的
    Δa 从前馈里减掉。因为标称模型对加速度是线性的，减在前馈上等价于换了一个
    更准的模型，不需要动控制器内部任何一行。
    """

    name = "geo_res"
    has_residual = True

    def __init__(self, plant, traj, net: ResidualNet, gains=None, params=None,
                 kp=None, kd=None):
        super().__init__(plant, traj, gains, params)
        self.net = net
        kp = kp if kp is not None else (-16.0, -16.0, -20.0)
        kd = kd if kd is not None else (-8.0, -8.0, -9.0)
        self.geo = GeometricController(self.params, kp=kp, kd=kd,
                                       kr=self.g.kr, kw=self.g.kw)

    def desired_accel(self, st, pos_d, vel_d, acc_d) -> np.ndarray:
        return acc_d

    def residual_world(self, st, a_cmd: np.ndarray) -> np.ndarray:
        v_b = self._R.T @ st.vel
        a_b = self._R.T @ self.last_a_cmd      # 上一步的指令，避免与本次求解循环依赖
        x = self.net.features(v_b, a_b, st.omega, float(st.pos[2]),
                              float(np.sum(self.last_thrust)) / self.params.mass)
        self.phi = self.net.last_layer_input(x)
        return self._R @ self.net.predict(x)


# --------------------------------------------------------------------------- #
class ResidualRLSTracker(ResidualTracker):
    """在离线网络之上，用 RLS 在线重写最后一层。

    离线网只见过标称档位，遇到没见过的载荷/风只能靠泛化；RLS 用飞行中实测的
    Δa 直接修正最后一层，把"泛化误差"变成"辨识误差"。只动最后一层是关键：
    在线数据时序强相关，整网微调会把隐层带偏。
    """

    name = "geo_rls"
    online = True

    def __init__(self, plant, traj, net: ResidualNet, gains=None, params=None,
                 lam: float = 0.998, delta: float = 1.0e2, warmup: int = 100,
                 leak: float = 0.0):
        # ⚠️ 必须深拷贝：RLS 是**原地改写**最后一层的（set_last_layer），
        # 直接改传入的 net 会让后面每一次运行都从上一轮被改写过的权重出发。
        # 实测：不当拷贝时同一份模型连跑 9 个工况，RMSE 出现 4.52 m 的乱跳，
        # 而单独跑同一工况只有 0.02 m —— 全部是权重被前一轮污染导致的。
        super().__init__(plant, traj, copy.deepcopy(net), gains, params)
        h = self.net.mlp.W[-1].shape[0] + 1        # 隐层 + 常数列（偏置）
        # theta 初始化成离线网络的最后一层（单位：m/s²），RLS 从"离线解"出发
        W, b = self.net.get_last_layer()
        self.rls = RecursiveLeastSquares(h, n_out=3, lam=lam, delta=delta,
                                         leak=leak)
        self._theta0 = np.vstack([W, b[None, :]])
        self.rls.load_state(self._theta0)
        self.warmup = int(warmup)
        self.n_seen = 0

    def reset(self) -> None:
        super().reset()
        self.n_seen = 0
        if hasattr(self, "rls"):
            self.rls.reset()
            self.rls.load_state(self._theta0)

    def observe(self) -> None:
        """先跑 ``warmup`` 步只辨识不施加，再接管最后一层。

        RLS 的第一次更新会把 θ 精确拟合到**单个**样本上（P 初值很大时
        增益 k≈φ/φᵀφ，与 P₀ 无关）。起飞段恰好是最大最脏的一个样本，
        直接施加会把最后一层一次性推到饱和限幅上、当场翻机（实测 +0.75 kg
        档 RMSE 4.2 m）。先空转 1 s 让 θ 在真实数据上收敛，再接管就没有跳变。
        """
        super().observe()
        self.n_seen += 1
        if self.phi is None:
            return
        ok = self.rls.update(self.phi, self.last_delta)
        if ok and self.n_seen >= self.warmup:
            self.net.set_last_layer(self.rls.theta[:-1], self.rls.theta[-1])

    @property
    def theta_drift(self) -> float:
        """最后一层相对离线解漂了多远（诊断用）。"""
        return float(np.linalg.norm(self.rls.theta - self._theta0))

    def diagnostics(self) -> Dict[str, float]:
        d = super().diagnostics()
        d.update({"rls_updates": float(self.rls.updates),
                  "rls_ready": float(self.rls.ready),
                  "theta_drift": self.theta_drift})
        return d


# --------------------------------------------------------------------------- #
TRACKERS: Dict[str, type] = {
    "pid": PIDTracker,
    "geo": GeometricTracker,
    "geo_res": ResidualTracker,
    "geo_rls": ResidualRLSTracker,
}


def make_tracker(name: str, plant, traj: Trajectory, net: Optional[ResidualNet] = None,
                 gains: Optional[Gains] = None, params: Optional[QuadParams] = None,
                 **kwargs) -> Tracker:
    if name not in TRACKERS:
        raise KeyError("未知控制器 %r，可选 %s" % (name, list(TRACKERS)))
    cls = TRACKERS[name]
    if issubclass(cls, ResidualTracker):
        if net is None:
            raise ValueError("控制器 %r 需要残差网络" % name)
        return cls(plant, traj, net, gains=gains, params=params, **kwargs)
    return cls(plant, traj, gains=gains, params=params, **kwargs)
