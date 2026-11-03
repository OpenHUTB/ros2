"""
domain_rand_core.py —— Sim-to-Real 域随机化（Domain Randomization）共享核心
================================================================================
同一套「纵向动力学车辆 + PI 速度控制器 + 标称训练 / 域随机化训练」逻辑被两个平台复用：
  - mujoco_domain_rand.py : MuJoCo 差速小车域随机化（本机运行）
  - sim_2d_domain_rand.py : 2D 轻量域随机化仿真（可直接跑，生成效果动图）

背景（sim-to-real gap）：
  在仿真里训练的控制器直接搬到真实车往往失效，因为真实环境与仿真存在『参数差异』
  ——质量、摩擦、电机增益、传感器噪声等。『域随机化』的做法是：训练时不只用一套
  标称参数，而是在一组随机化的参数分布上反复训练/整定，让策略对参数变化鲁棒，
  从而缩小仿真到真实的鸿沟。

本核心展示两种训练范式的对比：
  - 标称训练：只在一组固定参数 (m0, k0, c0) 上整定 PI 速度控制器增益；
  - 域随机化训练：在 m/k/c 各自分布内随机采样多组参数，整定出『对整段分布平均最优』的增益。
  - 评估：在一批『测试参数』（含大幅扰动）上比较两者速度跟踪误差 —— 域随机化应显著更鲁棒。

只依赖 numpy。
"""

import numpy as np


# ================================================================ 二阶位置动力学车辆
class SecondOrderVehicle:
    """二阶位置动力学：m·s'' = u·k − c·s'
    s 为沿路径的弧长位置，s' 即车速 v。参数 m/k/c 的差异即『域随机化』要覆盖
    的动力学不确定性。二阶 + PD 控制的组合使『增益过大会在低阻尼参数下振荡』，
    从而让标称最优增益在参数扰动下退化，凸显域随机化的鲁棒性。"""

    def __init__(self, m=1.0, k=1.0, c=0.5, v0=0.0, s0=0.0):
        self.m = float(m)
        self.k = float(k)
        self.c = float(c)
        self.v = float(v0)
        self.s = float(s0)

    def set_params(self, m=None, k=None, c=None):
        if m is not None:
            self.m = float(m)
        if k is not None:
            self.k = float(k)
        if c is not None:
            self.c = float(c)

    def step(self, u, dt):
        a = (self.k * u - self.c * self.v) / self.m
        self.v = max(0.0, self.v + a * dt)
        self.s += self.v * dt
        return self.v, self.s


# ================================================================ PD 位置控制器
class PDController:
    """PD 位置控制器：u = sat( Kp·(s_ref−s) − Kd·v )
    Kp 提供位置回复力，Kd 提供阻尼（抑制振荡）。
    增益 Kp/Kd 的取值来自两种训练范式：标称整定 vs 域随机化整定。"""

    def __init__(self, kp, kd, sat=2.0):
        self.kp = float(kp)
        self.kd = float(kd)
        self.sat = float(sat)

    def step(self, s_ref, s, v):
        u = self.kp * (s_ref - s) - self.kd * v
        return float(np.clip(u, 0.0, self.sat))


# ================================================================ 参考速度剖面
def ref_speed(t):
    """参考速度剖面：匀速 → 加速斜坡 → 匀速 → 减速 → 巡航。"""
    if t < 2.0:
        return 0.6
    if t < 4.0:
        return 0.6 + (1.3 - 0.6) * (t - 2.0) / 2.0     # 加速
    if t < 7.0:
        return 1.3
    if t < 8.0:
        return 1.3 - 0.5 * (t - 7.0)                    # 减速
    return 0.8


# ================================================================ 参考路径（弧长参数化）
def build_path():
    """折线路径（直线段 + 转弯），按弧长 s 参数化，返回插值函数。"""
    way = np.array([[-3.0, 0.0], [1.0, 0.0], [2.2, -1.5], [5.5, -1.5],
                    [6.5, 0.0], [8.0, 0.0]])
    seg = np.diff(way, axis=0)
    lens = np.hypot(seg[:, 0], seg[:, 1])
    cum = np.concatenate([[0.0], np.cumsum(lens)])
    total = float(cum[-1])

    def interp(s):
        s = float(np.clip(s, 0.0, total))
        k = int(np.searchsorted(cum, s, side='right') - 1)
        k = min(max(k, 0), len(way) - 2)
        t = (s - cum[k]) / lens[k] if lens[k] > 0 else 0.0
        return way[k] + t * seg[k]

    return way, interp, total


# ================================================================ 域随机化采样器
class DomainRandomizer:
    """动力学参数域随机化采样器。

    语义（sim-to-real gap）：`nominal` 是『仿真里以为的标称参数』，用来做
    标称标定；`real_range` 是『真实环境可能的参数范围』（仿真与真实之间的差异），
    域随机化在 real_range 内随机采样来训练/标定，使控制器对整段真实分布鲁棒。
    参数：m 质量、k 电机增益、c 阻力系数。
    """

    def __init__(self, nominal=(1.0, 1.2, 0.3),
                 real_range=(0.7, 0.7, 0.5, 1.5, 1.0, 1.0), seed=0):
        self.m0, self.k0, self.c0 = map(float, nominal)          # 仿真标称
        self.m_lo, self.k_lo, self.c_lo, self.m_hi, self.k_hi, self.c_hi = \
            map(float, real_range)                               # 真实分布（绝对范围）
        self.rng = np.random.default_rng(seed)

    def sample(self):
        """从『真实分布』采样一组参数 (m, k, c)。"""
        m = self.rng.uniform(self.m_lo, self.m_hi)
        k = self.rng.uniform(self.k_lo, self.k_hi)
        c = self.rng.uniform(self.c_lo, self.c_hi)
        return m, k, c

    def nominal(self):
        return self.m0, self.k0, self.c0


# ================================================================ 前馈控制器（开环标定）
class FeedforwardController:
    """开环前馈控制器：u = g · v_ref
    g 是『速度→电机指令』的标定增益（g ≈ c/k，由模型参数标定）。
    仿真里标定的 g 直接搬到真实车会失效，因为真实车的 k/c 与仿真不同 ——
    这正是 sim-to-real gap 的根源。域随机化在参数分布上标定 g，使其对分布平均鲁棒。"""

    def __init__(self, g):
        self.g = float(g)

    def step(self, v_ref, v):
        return float(np.clip(self.g * v_ref, 0.0, 2.0))


def calibrate_nominal(sampler):
    """标称标定：用标称模型的电机增益/阻力求 g_nom = c0/k0（只对标称准）。"""
    m0, k0, c0 = sampler.nominal()
    return c0 / k0


def calibrate_domain_randomized(sampler, n_samples=60):
    """域随机化标定：在参数分布上采样，对每个参数求『该参数下稳态无偏的 g_i=c_i/k_i』，
    取平均得到 g_dr —— 对整段分布平均最优，不偏向任何单点。"""
    gs = []
    for _ in range(n_samples):
        m, k, c = sampler.sample()
        gs.append(c / k)
    return float(np.mean(gs))


# ================================================================ episode 运行与评估
def run_episode(vehicle, ctrl, dt=0.02, T=10.0):
    """跑一个 episode：用前馈控制器沿参考速度剖面行驶。
    返回 (速度RMSE, 位置误差, 时间, 速度, 参考速度, 位置, 参考位置)。
    前馈增益标定不准 → 稳态速度偏离参考 → 速度RMSE与位置累积误差增大。"""
    t = 0.0
    s = 0.0
    s_ref = 0.0
    ts, vs, vrefs, ss, srefs = [], [], [], [], []
    while t <= T:
        v_ref = ref_speed(t)
        s_ref += v_ref * dt
        u = ctrl.step(v_ref, vehicle.v)
        v, s = vehicle.step(u, dt)
        ts.append(t); vs.append(v); vrefs.append(v_ref)
        ss.append(s); srefs.append(s_ref)
        t += dt
    vel_rmse = float(np.sqrt(np.mean((np.array(vs) - np.array(vrefs)) ** 2)))
    pos_err = float(np.mean(np.abs(np.array(ss) - np.array(srefs))))
    return (vel_rmse, pos_err, np.array(ts), np.array(vs), np.array(vrefs),
            np.array(ss), np.array(srefs))


# ================================================================ 评估
def evaluate(g_nominal, g_dr, test_params, dt=0.02):
    """在一批测试参数（含大幅扰动）上比较标称标定 / 域随机化标定的前馈控制器。
    返回逐参数的 (标签, 标称速度RMSE, DR速度RMSE)。"""
    rows = []
    for (label, (m, k, c)) in test_params:
        vh = SecondOrderVehicle(m, k, c)
        rmse_n, *_ = run_episode(vh, FeedforwardController(g_nominal), dt)
        vh = SecondOrderVehicle(m, k, c)
        rmse_d, *_ = run_episode(vh, FeedforwardController(g_dr), dt)
        rows.append((label, rmse_n, rmse_d))
    return rows
