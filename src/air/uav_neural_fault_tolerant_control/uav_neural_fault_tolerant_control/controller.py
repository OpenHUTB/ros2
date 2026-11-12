"""容错控制器：几何控制器给出期望 (T, τ)，再由分配层分给四个桨。

四种模式放在同一个类里，实验时只换 ``mode``，保证"除分配方式外其它都一样"：

``geometric``
    完全不考虑故障：期望量直接过底座 ``mixer``。这是健康时调好的做法，也是基线。
``naive``
    还是底座 ``mixer``，再把坏掉的桨丢掉的推力平均分给其余桨。
``ls``
    用带健康度的加权最小二乘分配（"加权伪逆"）。
``neural``
    在 ``ls`` 之上加一个小型 MLP 残差，学的是精确箱约束解与 ``ls`` 解的差。
"""

from __future__ import annotations

import numpy as np

from . import allocation as A
from . import features as F

#: 支持的控制模式
MODES = ("geometric", "naive", "ls", "neural")
#: 残差的最大修正量（占单桨满推力的比例）。网络在训练分布之外可能给出
#: 很大的修正，一旦贴着箱约束边界就会把反馈余量吃光，所以部署时限幅。
MAX_RESIDUAL = 0.25


class FaultTolerantController:
    """与底座 ``GeometricController`` 同签名（``act(quad, p_d, v_d, a_d, yaw_d)``）。"""

    def __init__(self, params, mixer, base_controller, effectiveness,
                 mode: str = "neural", net=None, gains=None, bias=None,
                 warm_start: bool = True, override_weights=None):
        if mode not in MODES:
            raise ValueError(f"未知控制模式 {mode!r}，可选：{MODES}")
        if mode == "neural" and net is None:
            raise ValueError("neural 模式需要传入训练好的 net")
        self.params = params
        self.mixer = mixer
        self.base = base_controller
        self.B = np.asarray(effectiveness, dtype=float)
        self.mode = mode
        self.net = net
        self.gains = np.ones(4) if gains is None else np.asarray(gains, dtype=float)
        self.bias = np.zeros(4) if bias is None else np.asarray(bias, dtype=float)
        self.warm_start = bool(warm_start)
        self.override_weights = override_weights
        self._v_last = None
        self.last_desired = None
        self.last_u = None
        self.last_u_ls = None
        self.last_features = None

    # -- 健康度 -----------------------------------------------------------
    def set_health(self, gains, bias=None) -> None:
        """接入健康度（真值，或来自故障检测网络的估计）。"""
        self.gains = np.asarray(gains, dtype=float)
        if bias is not None:
            self.bias = np.asarray(bias, dtype=float)

    # -- 分配权重 ---------------------------------------------------------
    def row_weights(self) -> np.ndarray:
        """按可控制桨数选择优先级。

        四桨健康时四个期望量都能精确满足，权重无所谓；一旦有桨彻底失效，
        期望量之间出现约束（例如单桨失效时 ``T = τx/a + τy/a + τz/k_m``），
        这时必须主动放弃偏航——这正是 Mueller & D'Andrea (ICRA 2014) 的结论：
        少一个桨就放弃偏航控制，让机身自转，否则力矩互相抢预算，姿态先崩。
        """
        if self.override_weights is not None:
            return np.asarray(self.override_weights, dtype=float)
        controllable = int(np.sum(self.gains > A.GAIN_EPS))
        if controllable >= 4:
            return np.array([A.THRUST_WEIGHT, 1.0, 1.0, 1.0])
        return np.array([A.THRUST_WEIGHT, 10.0, 10.0, 0.0])

    # -- 主流程 -----------------------------------------------------------
    def allocate(self, desired) -> np.ndarray:
        """期望量 → 四桨推力指令（按 ``mode`` 分支）。"""
        d = np.asarray(desired, dtype=float)
        self.last_desired = d
        t_max = float(self.params.max_thrust)
        rw = self.row_weights()

        if self.mode == "geometric":
            u = np.clip(np.asarray(self.mixer(float(d[0]), d[1:], self.params),
                                   dtype=float), 0.0, t_max)
            self.last_u = u
            self._v_last = A.effective_thrust(u, self.gains, self.bias)
            return u

        if self.mode == "naive":
            u_honest = np.asarray(self.mixer(float(d[0]), d[1:], self.params),
                                  dtype=float)
            u = A.naive_redistribute(u_honest, self.params, self.gains)
            self.last_u = u
            self._v_last = A.effective_thrust(u, self.gains, self.bias)
            return u

        v_ref = self._v_last if (self.warm_start and self._v_last is not None) else None
        u_ls = A.ls_allocation(self.B, d, self.params, self.gains,
                               v_ref=v_ref, bias=self.bias, row_weights=rw)
        self.last_u_ls = u_ls
        if self.mode == "ls":
            self.last_u = u_ls
            self._v_last = A.effective_thrust(u_ls, self.gains, self.bias)
            return u_ls

        x = self._features(d, u_ls)
        residual = np.clip(self._residual(x), -MAX_RESIDUAL, MAX_RESIDUAL)
        u = A.clamp_thrusts(F.apply_residual(u_ls, residual, self.params), self.params)
        self.last_u = u
        self._v_last = A.effective_thrust(u, self.gains, self.bias)
        return u

    def _features(self, desired, u_ls) -> np.ndarray:
        x = F.build_features(desired, self.gains, u_ls, self.params)
        self.last_features = x
        return x

    def _residual(self, x: np.ndarray) -> np.ndarray:
        from . import model as M
        return np.asarray(M.predict_numpy(self.net, x), dtype=float)

    def reset(self) -> None:
        self._v_last = None
        self.last_desired = None
        self.last_u = None
        self.last_u_ls = None
        self.last_features = None

    # -- 与底座一致的入口 -------------------------------------------------
    def act(self, quad, p_d, v_d, a_d, yaw_d) -> np.ndarray:
        state = quad.state() if hasattr(quad, "state") else quad
        thrust, torque = self.base.compute(
            state, np.asarray(p_d, dtype=float), np.asarray(v_d, dtype=float),
            np.asarray(a_d, dtype=float), float(yaw_d))
        return self.allocate(np.array([thrust, *np.asarray(torque, dtype=float)]))
