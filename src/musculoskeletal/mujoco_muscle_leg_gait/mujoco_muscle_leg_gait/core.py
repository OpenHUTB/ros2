"""核心：Hill 肌肉模型、MJCF 生成、肌肉力分配。"""

from __future__ import annotations

import itertools

import numpy as np

# ---------------- 全局参数（缩放的人体量级） ----------------
DT = 0.002                 # 仿真步长 (s)
F_MAX = 600.0              # 单条肌肉最大等长力 (N)
L0_MUS = 0.30              # 关节中立位时的肌肉长度 (m)
R_HIP = 0.050              # 髋关节矩臂 (m)
R_KNEE = 0.040             # 膝关节矩臂 (m)

# 单关节实验参数
SJ_F_MAX = 120.0
SJ_L0 = 0.32
SJ_R = 0.030
SJ_MASS = 1.5
SJ_COM_X = 0.02

# 腿模型参数
THIGH_L, SHANK_L = 0.20, 0.20
M_PELVIS, M_THIGH, M_SHANK, M_FOOT = 8.0, 1.5, 1.0, 0.4
GROUND_Z = -0.46


class HillMuscle:
    """Hill 三元件肌肉：激活动力学 + 力-长度 + 力-速度 + 被动力。

    与仓库 muscle_joint_control 的实现保持一致（同一组曲线形式），
    便于跨模块对照。
    """

    def __init__(self, l_opt: float, f_max: float = F_MAX):
        self.l_opt = float(l_opt)
        self.f_max = float(f_max)
        self.v_max = 10.0 * self.l_opt     # 最大缩短速度 (m/s)
        self.a = 0.0                       # 当前激活度

    @staticmethod
    def fl(ln: float) -> float:
        """归一化力-长度曲线（高斯型，峰值在 ln=1）。"""
        return float(np.exp(-((ln - 1.0) / 0.45) ** 2))

    @staticmethod
    def fl_prime(ln: float) -> float:
        """d(fl)/d(ln)，用于刚度解析校验。"""
        return float(-2.0 * (ln - 1.0) / 0.45 ** 2 * np.exp(-((ln - 1.0) / 0.45) ** 2))

    @staticmethod
    def fp(ln: float) -> float:
        """被动力：仅在 ln>1 时存在。"""
        if ln <= 1.0:
            return 0.0
        return float((np.exp(5.0 * (ln - 1.0)) - 1.0) / (np.exp(5.0 * 0.6) - 1.0))

    @staticmethod
    def fp_prime(ln: float) -> float:
        if ln <= 1.0:
            return 0.0
        return float(5.0 * np.exp(5.0 * (ln - 1.0)) / (np.exp(5.0 * 0.6) - 1.0))

    @staticmethod
    def fv(vn: float) -> float:
        """力-速度：向心段按 Hill 方程，离心段略高于等长。"""
        if vn >= 0:
            return float(0.3125 / (vn + 0.25) - 0.25)
        x = min(1.0, -vn)
        return float(1.0 + 0.4 * x / (0.25 + x))

    def update_activation(self, u: float, dt: float,
                          tau_act: float = 0.01, tau_deact: float = 0.04) -> float:
        """一阶激活动力学：激活快于放松。"""
        tau = tau_act if u > self.a else tau_deact
        self.a = float(np.clip(self.a + (u - self.a) * min(1.0, dt / tau), 0.0, 1.0))
        return self.a

    def force(self, length: float, length_dot: float) -> float:
        ln = length / self.l_opt
        vn = -length_dot / self.v_max       # 缩短为正
        return self.f_max * (self.a * self.fl(ln) * self.fv(vn) + self.fp(ln))


# ---------------- MJCF 生成 ----------------

def single_joint_xml(com_x: float = SJ_COM_X, gravity: str = "0 0 -9.81") -> str:
    """单髋关节模型（L1-L3 用）。关节 range 必须显式声明弧度单位。"""
    return f"""
<mujoco model="single_hip">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="{DT}" integrator="implicitfast" gravity="{gravity}"/>
  <worldbody>
    <body name="pelvis">
      <body name="thigh">
        <joint name="hip" type="hinge" axis="0 1 0" range="-1.2 1.2"
               damping="0.15" armature="0.002"/>
        <geom type="capsule" size="0.022"
              fromto="{com_x} 0 0 {com_x} 0 -0.20" mass="{SJ_MASS}"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"""


def two_joint_xml() -> str:
    """髋 + 膝两关节模型（L4 用）。"""
    return f"""
<mujoco model="two_joint_leg">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="{DT}" integrator="implicitfast" gravity="0 0 -9.81"/>
  <worldbody>
    <body name="pelvis">
      <body name="thigh">
        <joint name="hip" type="hinge" axis="0 1 0" range="-1.2 1.2"
               damping="0.15" armature="0.002"/>
        <geom type="capsule" size="0.022" fromto="0 0 0 0 0 -{THIGH_L}"
              mass="{M_THIGH}"/>
        <body name="shank" pos="0 0 -{THIGH_L}">
          <joint name="knee" type="hinge" axis="0 1 0" range="-0.05 1.8"
                 damping="0.12" armature="0.001"/>
          <geom type="capsule" size="0.020" fromto="0 0 0 0 0 -{SHANK_L}"
                mass="{M_SHANK}"/>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>
"""


def _leg_body(side: str, y: float) -> str:
    return f"""
      <body name="thigh_{side}" pos="0 {y} 0">
        <joint name="hip_{side}" type="hinge" axis="0 1 0" range="-1.0 1.0"
               damping="0.6" armature="0.01"/>
        <geom type="capsule" size="0.030" fromto="0 0 0 0 0 -{THIGH_L}"
              mass="{M_THIGH}" rgba="0.20 0.55 0.95 1"/>
        <body name="shank_{side}" pos="0 0 -{THIGH_L}">
          <joint name="knee_{side}" type="hinge" axis="0 1 0" range="0.0 1.6"
                 damping="0.5" armature="0.008"/>
          <geom type="capsule" size="0.026" fromto="0 0 0 0 0 -{SHANK_L}"
                mass="{M_SHANK}" rgba="0.95 0.55 0.20 1"/>
          <body name="foot_{side}" pos="0 0 -{SHANK_L}">
            <geom name="foot_g_{side}" type="box" size="0.075 0.035 0.025"
                  pos="0.02 0 -0.025" mass="{M_FOOT}" rgba="0.30 0.30 0.34 1"/>
          </body>
        </body>
      </body>"""


def legs_xml(nlegs: int = 1, gscale: float = 1.0, hip_y: float = 0.09) -> str:
    """负重站立 / 踏步模型（L5-L6 用）。

    pelvis 有两个滑轨自由度：sway(x) 允许屈膝时骨盆后移，lif(tz) 承重。
    这两个滑轨相当于「体重支撑架」，是 L5/L6 的已知简化。
    """
    legs = _leg_body("L", 0.0) if nlegs == 1 else (
        _leg_body("L", hip_y) + _leg_body("R", -hip_y))
    return f"""
<mujoco model="muscle_leg_stand">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="{DT}" integrator="implicitfast" gravity="0 0 {-9.81 * gscale}"/>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.26 0.26 0.30"
             rgb2="0.17 0.17 0.20" width="512" height="512"/>
    <material name="grid" texture="grid" texrepeat="8 8" reflectance="0.05"/>
  </asset>
  <worldbody>
    <light pos="0.8 -1.0 1.6" dir="-0.3 0.5 -1" directional="true"/>
    <geom name="floor" type="plane" size="2 2 0.01" pos="0 0 {GROUND_Z}"
          material="grid" friction="1.0 0.01 0.001"/>
    <body name="pelvis" pos="0 0 0">
      <joint name="sway" type="slide" axis="1 0 0" damping="6.0" range="-0.25 0.25"/>
      <joint name="lift" type="slide" axis="0 0 1" damping="2.0" range="-0.30 0.02"/>
      <geom type="box" size="0.09 0.13 0.05" mass="{M_PELVIS}"
            rgba="0.55 0.57 0.62 1"/>
      {legs}
    </body>
  </worldbody>
</mujoco>
"""


# ---------------- 肌肉分配 ----------------

def joint_pd(q, qd, q_des, qd_des, kp: float, kd: float):
    return kp * (np.asarray(q_des) - np.asarray(q)) + kd * (np.asarray(qd_des) - np.asarray(qd))


def allocate_pair(tau: float, arm: float, f_max: float, base: float = 0.02):
    """单关节两肌分配：按力矩符号选主肌，另一条维持 base 共激活。

    返回 [主肌激活, 对侧激活]，顺序与 (正力矩方向, 负力矩方向) 对应。
    """
    a = np.array([base, base])
    a[0 if tau >= 0 else 1] = float(np.clip(abs(tau) / (abs(arm) * f_max), 0.0, 1.0))
    return a


def solve_joint_qp(A, tau, a_max: float = 1.0):
    """min Σa²  s.t. A·a = tau, 0 ≤ a ≤ a_max（肌肉数很少，枚举子集即精确解）。"""
    A = np.asarray(A, dtype=float)
    tau = np.asarray(tau, dtype=float)
    n = A.shape[1]
    if np.allclose(tau, 0.0):
        return np.zeros(n)
    # 只用子集大小 >= 2：单条肌肉无法同时满足两个关节的力矩
    for k in (2, 3):
        best = None
        for combo in itertools.combinations(range(n), k):
            As = A[:, combo]
            if np.linalg.matrix_rank(As) < 2:
                continue
            a, *_ = np.linalg.lstsq(As, tau, rcond=None)
            if np.all(a >= -1e-9) and np.all(a <= a_max + 1e-9):
                cost = float(np.sum(a ** 2))
                if best is None or cost < best[0]:
                    best = (cost, combo, a)
        if best is not None:
            out = np.zeros(n)
            out[list(best[1])] = np.clip(best[2], 0.0, a_max)
            return out
    a, *_ = np.linalg.lstsq(A, tau, rcond=None)
    return np.clip(a, 0.0, a_max)


class Leg:
    """一条腿的 4 条单关节肌肉（髋屈/髋伸/膝伸/膝屈）。"""

    NAMES = ("hip_flex", "hip_ext", "knee_ext", "knee_flex")

    def __init__(self, f_max: float = F_MAX, l0: float = L0_MUS,
                 r_hip: float = R_HIP, r_knee: float = R_KNEE):
        self.arms = np.array([
            [r_hip, 0.0], [-r_hip, 0.0],
            [0.0, -r_knee], [0.0, r_knee],
        ])
        self.muscles = [HillMuscle(l0 * 1.10, f_max) for _ in self.NAMES]
        self.l0 = l0
        self.r_hip, self.r_knee = r_hip, r_knee

    def lengths(self, q):
        return self.l0 - self.arms @ np.asarray(q)

    def torques(self, F):
        return self.arms.T @ np.asarray(F)

    def activations_for(self, tau, base: float = 0.02):
        """把 [髋力矩, 膝力矩] 分给 4 条肌肉。注意膝肌序是 [ext, flex]。"""
        u = np.zeros(4)
        u[0:2] = allocate_pair(tau[0], self.r_hip, self.muscles[0].f_max, base)
        ak = allocate_pair(tau[1], self.r_knee, self.muscles[2].f_max, base)
        u[2], u[3] = ak[1], ak[0]
        return u
