"""专家策略：几何控制器 + 已知健康度的加权最小二乘分配。

专家只用来打标签（DAgger 的教师），部署时不跑。它和端到端策略的差别就是"手工链路"：
状态 → 期望推力/力矩 → 按健康度分配到四桨。策略要把这一整条链路用一张网络替代。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from . import allocation as A


@dataclass(frozen=True)
class TaskConfig:
    """走廊任务参数（专家和评估共用，保证两者在做同一件事）。"""

    start_altitude: float = 1.5
    goal_x: float = 10.0
    cruise_speed: float = 1.0
    duration: float = 14.0
    dt: float = 0.01
    substeps: int = 4
    crash_altitude: float = 0.3
    max_lateral: float = 3.0
    scene: str = "corridor"


def _trapezoid(t: float, distance: float, cruise: float,
               accel: float) -> tuple[float, float, float]:
    """梯形速度剖面，返回 ``(位置, 速度, 加速度)``。"""
    distance = float(distance)
    cruise = float(cruise)
    accel = max(float(accel), 1e-3)
    if distance <= 0.0:
        return 0.0, 0.0, 0.0
    v_top = min(cruise, np.sqrt(distance * accel))     # 加速+减速恰好走完时也成立
    t_acc = v_top / accel
    d_acc = 0.5 * accel * t_acc ** 2
    d_cruise = max(0.0, distance - 2.0 * d_acc)
    t_cruise = d_cruise / v_top if v_top > 1e-9 else 0.0
    if t <= 0.0:
        return 0.0, 0.0, 0.0
    if t < t_acc:
        return 0.5 * accel * t * t, accel * t, accel
    t1 = t_acc
    if t < t1 + t_cruise:
        return d_acc + v_top * (t - t1), v_top, 0.0
    t2 = t1 + t_cruise
    tau = t - t2
    t_dec = v_top / accel
    if tau < t_dec:
        return (d_acc + d_cruise + v_top * tau - 0.5 * accel * tau * tau,
                v_top - accel * tau, -accel)
    return distance, 0.0, 0.0


class ThrustExpert:
    """输入飞行器状态与真实健康度，输出 4 个归一化桨推力指令。"""

    def __init__(self, params, mixer, base_controller, effectiveness,
                 task: TaskConfig | None = None, accel_limit: float = 0.8):
        self.params = params
        self.mixer = mixer
        self.base = base_controller
        self.effectiveness = np.asarray(effectiveness, dtype=float)
        self.task = task or TaskConfig()
        self.accel_limit = float(accel_limit)

    def reference(self, t: float, position=None) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """梯形速度剖面的参考：位置、速度、加速度一起给（前瞻量不再是"胡萝卜"）。

        为什么必须给加速度前馈：底座 ``GeometricController`` 只把 ``v_d`` 当速度误差
        用 ``kd=-8`` 放大。若把 ``v_d`` 直接设成 1.2 m/s，0 时刻就要求 9.6 m/s² 的
        减速度，机体瞬间要 45° 以上的倾角；反过来 ``v_d=0`` + 位置前瞻又会在目标点
        附近抖。用梯形剖面（起停各 1.5 s、加速度上限 0.8 m/s²）后，倾角只在起停的
        一两秒里到十几度，其余时间接近水平——专家演示才像"正常飞行"。
        """
        cfg = self.task
        x, v, a = _trapezoid(t, cfg.goal_x, cfg.cruise_speed, self.accel_limit)
        p_d = np.array([x, 0.0, cfg.start_altitude], dtype=float)
        v_d = np.array([v, 0.0, 0.0], dtype=float)
        a_d = np.array([a, 0.0, 0.0], dtype=float)
        return p_d, v_d, a_d

    def command(self, state, gains: np.ndarray, bias: np.ndarray | None,
                t: float) -> np.ndarray:
        """返回归一化指令（0~1，乘 ``max_thrust`` 后才是牛顿）。"""
        p_d, v_d, a_d = self.reference(t, np.asarray(state.pos, dtype=float))
        thrust, tau = self.base.compute(state, p_d, v_d, a_d, 0.0)
        desired = np.concatenate([[thrust], np.asarray(tau, dtype=float)])
        rows = [A.THRUST_WEIGHT, 1.0, 1.0, 1.0]
        if float(np.sum(np.asarray(gains) > A.GAIN_EPS)) < 4:
            # 有桨彻底失效：放弃偏航，把预算留给推力与滚转/俯仰
            rows = [A.THRUST_WEIGHT, 10.0, 10.0, 0.0]
        u = A.ls_allocation(
            self.effectiveness, desired, self.params, gains,
            row_weights=np.asarray(rows, dtype=float), bias=bias,
        )
        u = np.clip(np.nan_to_num(u, nan=0.0), 0.0, self.params.max_thrust)
        return (u / self.params.max_thrust).astype(np.float32)
