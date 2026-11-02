#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""四旋翼模型、混控（mixer）与几何控制器，仅依赖 mujoco + numpy。

世界系 ENU（重力沿 -z）、机体系 x 前 y 左 z 上，与 ``/uav/odom`` 一致。
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Tuple

import mujoco
import numpy as np

# 模型文件位置：优先走 paths 定位（兼容源码与 colcon 安装布局）
try:
    from .paths import quadrotor_xml as _quadrotor_xml
    DEFAULT_MODEL = _quadrotor_xml()
except Exception:                                    # noqa: BLE001
    # 定位失败时退回源码布局
    _PKG_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    DEFAULT_MODEL = os.path.join(os.path.dirname(_PKG_ROOT), "models", "quadrotor.xml")


# --------------------------------------------------------------------------- #
# 姿态工具
# --------------------------------------------------------------------------- #
def quat_to_mat(q: np.ndarray) -> np.ndarray:
    """四元数 (w,x,y,z) → 3x3 旋转矩阵（机体→世界）。"""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def mat_to_quat(R: np.ndarray) -> np.ndarray:
    """3x3 旋转矩阵 → 四元数 (w,x,y,z)（数值稳定版）。"""
    tr = np.trace(R)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def quat_to_euler(q: np.ndarray) -> Tuple[float, float, float]:
    """四元数 → (roll, pitch, yaw)，绕机体系 x,y,z 的 ZYX 欧拉角，单位弧度。"""
    R = quat_to_mat(q)
    pitch = np.arcsin(np.clip(-R[2, 0], -1.0, 1.0))
    roll = np.arctan2(R[2, 1], R[2, 2])
    yaw = np.arctan2(R[1, 0], R[0, 0])
    return float(roll), float(pitch), float(yaw)


def vec_to_skew(v: np.ndarray) -> np.ndarray:
    """三维向量 → 反对称矩阵。"""
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])


def vee(M: np.ndarray) -> np.ndarray:
    """反对称矩阵 → 三维向量。"""
    return np.array([M[2, 1] - M[1, 2], M[0, 2] - M[2, 0], M[1, 0] - M[0, 1]]) * 0.5


# --------------------------------------------------------------------------- #
# 机体参数
# --------------------------------------------------------------------------- #
@dataclass
class QuadParams:
    """从模型读出的机体参数（随 XML 变化，不硬编码）。"""

    mass: float
    arm: float          # a = L/(2√2)，单桨到机心的 x/y 分量
    k_m: float          # 反扭矩系数：τz = k_m * ( -T1 - T2 + T3 + T4 )
    max_thrust: float   # 单桨最大推力（与 XML ctrlrange 一致）
    hover_thrust: float # 单桨悬停推力 = m*g/4

    @property
    def gravity(self) -> float:
        return 9.81


def mixer(T: float, tau: np.ndarray, p: QuadParams) -> np.ndarray:
    """期望合推力 T 与力矩 tau=[τx,τy,τz] → 四个旋翼推力。

    u = τx/a, v = τy/a, w = τz/k_m，四桨推力形如 (T ± u ± v ± w)/4，
    全部落在 [0, T_max] 的充要条件是 |u| + |v| + |w| ≤ min(4·T_max − T, T)。
    饱和时整体等比缩小 (u,v,w)；逐个 clip 会破坏力矩方向（实测 17 mm → 474 m）。
    """
    u = tau[0] / p.arm
    v = tau[1] / p.arm
    w = tau[2] / p.k_m

    # 合推力上限受四个电机约束
    T = float(np.clip(T, 0.0, 4.0 * p.max_thrust))

    budget = min(4.0 * p.max_thrust - T, T)
    spread = abs(u) + abs(v) + abs(w)
    if spread > budget and spread > 1e-12:
        k = budget / spread
        u, v, w = u * k, v * k, w * k

    out = np.array([
        (T - u - v - w) * 0.25,
        (T + u + v - w) * 0.25,
        (T + u + w - v) * 0.25,
        (T + v + w - u) * 0.25,
    ])
    # 理论已在范围内，再 clip 一次兜浮点误差
    return np.clip(out, 0.0, p.max_thrust)


# --------------------------------------------------------------------------- #
# 四旋翼本体
# --------------------------------------------------------------------------- #
@dataclass
class QuadrotorState:
    """一帧机体状态（全部为世界系 ENU，角速度为机体系）。"""

    t: float
    pos: np.ndarray          # (3,) 世界系位置
    quat: np.ndarray         # (4,) (w,x,y,z) 机体→世界
    vel: np.ndarray          # (3,) 世界系线速度
    omega: np.ndarray        # (3,) 机体系角速度
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    rotor_thrust: np.ndarray = field(default_factory=lambda: np.zeros(4))


class Quadrotor:
    """四旋翼仿真主体：加载模型、推进物理、写入旋翼推力、读取状态。"""

    def __init__(self, model_path: str | None = None, dt: float = 0.01,
                 substeps: int = 4, body_name: str = "base_link",
                 xml: str | None = None, scene=None, k_m: float | None = None):
        """三种构造方式，优先级 scene > xml > model_path：场景对象 / MJCF 字符串 / 模板 XML 路径。"""
        self._k_m_override = k_m
        if scene is not None:
            from .scenes import scene_to_xml, make_scene
            if isinstance(scene, str):
                scene = make_scene(scene)
            self.scene = scene
            self.model_path = f"<scene:{scene.name}>"
            xml = scene_to_xml(scene)
            self.model = mujoco.MjModel.from_xml_string(xml)
        elif xml is not None:
            self.scene = None
            self.model_path = "<xml-string>"
            self.model = mujoco.MjModel.from_xml_string(xml)
        else:
            self.model_path = model_path or DEFAULT_MODEL
            if not os.path.exists(self.model_path):
                raise FileNotFoundError(f"找不到四旋翼模型文件：{self.model_path}")
            self.scene = None
            self.model = mujoco.MjModel.from_xml_path(self.model_path)

        self.data = mujoco.MjData(self.model)

        self.dt = float(dt)
        self.substeps = int(substeps)
        self.time = 0.0

        self.body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if self.body_id < 0:
            raise ValueError(f"模型里找不到机体 {body_name!r}")
        self.lidar_site = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "lidar")

        self.params = self._read_params()
        self._free_qadr = self.model.jnt_qposadr[
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "root")
        ]
        self._free_vadr = self.model.jnt_dofadr[
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "root")
        ]
        # 风扰力（抗风扰对比实验用）
        self.wind = np.zeros(3)

        # mj_forward 必须先调：site_xpos / site_xmat / sensordata 等派生量在
        # MjData 构造后全为零
        mujoco.mj_forward(self.model, self.data)

    # ---------------------------------------------------------------- 参数
    def _read_params(self) -> QuadParams:
        m = self.model
        # 子树总质量；body_mass 不含旋翼子刚体（实测低估 1.29 m）
        body_mass = float(m.body_subtreemass[self.body_id])
        # a：XML 中旋翼 1 相对机体的位置
        r1 = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "rotor1")
        a = abs(float(m.body_pos[r1][0])) if r1 >= 0 else 0.106
        # 单桨最大推力：执行器 ctrlrange
        act = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "thrust1")
        max_t = float(m.actuator_ctrlrange[act][1]) if act >= 0 else 6.0
        # k_m = τ_yaw / T ≈ (C_Q/C_T)·D；0.128 m 桨径对应 0.015 m
        k_m = 0.015 if self._k_m_override is None else float(self._k_m_override)
        return QuadParams(
            mass=body_mass,
            arm=a,
            k_m=k_m,
            max_thrust=max_t,
            hover_thrust=body_mass * 9.81 / 4.0,
        )

    # ---------------------------------------------------------------- 状态
    @property
    def qpos(self) -> np.ndarray:
        return self.data.qpos[self._free_qadr:self._free_qadr + 7]

    @property
    def qvel(self) -> np.ndarray:
        return self.data.qvel[self._free_vadr:self._free_vadr + 6]

    def state(self) -> QuadrotorState:
        qp, qv = self.qpos, self.qvel
        roll, pitch, yaw = quat_to_euler(qp[3:7])
        return QuadrotorState(
            t=self.time,
            pos=qp[0:3].copy(),
            quat=qp[3:7].copy(),
            vel=qv[0:3].copy(),
            omega=qv[3:6].copy(),
            roll=roll, pitch=pitch, yaw=yaw,
        )

    def reset(self, pos=(0.0, 0.0, 1.5), yaw: float = 0.0) -> None:
        """把无人机放回指定位置并清零速度。"""
        mujoco.mj_resetData(self.model, self.data)
        self._free_qadr = self.model.jnt_qposadr[
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "root")
        ]
        self.data.qpos[self._free_qadr + 0:self._free_qadr + 3] = np.asarray(pos, dtype=float)
        self.data.qpos[self._free_qadr + 3:self._free_qadr + 7] = np.array(
            [np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)]
        )
        self.data.qvel[:] = 0.0
        self.time = 0.0
        mujoco.mj_forward(self.model, self.data)

    # ---------------------------------------------------------------- 推进
    def apply_rotor_thrust(self, thrusts: np.ndarray) -> None:
        """把四个旋翼推力写入执行器 ctrl；偏航反扭矩补到 xfrc_applied

        τz = k_m·(−T1 − T2 + T3 + T4)，执行器 gear 只出力不出力矩。
        """
        self.data.ctrl[:] = np.clip(thrusts, 0.0, self.params.max_thrust)
        t1, t2, t3, t4 = self.data.ctrl[:4]
        tau_z = self.params.k_m * (-t1 - t2 + t3 + t4)
        # xfrc_applied 力矩分量是世界系，需转换绕机体 z 的偏航力矩
        R = quat_to_mat(self.qpos[3:7])
        self.data.xfrc_applied[self.body_id, 3:6] = R @ np.array([0.0, 0.0, tau_z])

    def apply_wind(self) -> None:
        """把风力作为机体上的外力施加（供抗风扰对比实验使用）。"""
        if np.any(self.wind):
            self.data.xfrc_applied[self.body_id, 0:3] += self.wind

    def step(self, n: int = 1) -> None:
        """推进 n 个控制周期（每个周期含 substeps 个物理子步）。"""
        for _ in range(n):
            for _ in range(self.substeps):
                mujoco.mj_step(self.model, self.data)
            self.time += self.dt

    def step_with_control(self, thrusts: np.ndarray, n: int = 1) -> None:
        """写入推力并推进（每个子步前重加外力：xfrc_applied 会被 mj_step 清零）。"""
        for _ in range(n):
            self.apply_rotor_thrust(thrusts)
            for _ in range(self.substeps):
                self.apply_wind()
                mujoco.mj_step(self.model, self.data)
            self.time += self.dt


# --------------------------------------------------------------------------- #
# 几何控制器（位置 + 偏航 → 合推力/力矩）
# --------------------------------------------------------------------------- #
class GeometricController:
    """几何控制器：位置 + 偏航指令 → 合推力/力矩。

    外环 f_des = m·(−g·e_z + a_d + Kp⊙(p_d − p) + Kd⊙(v_d − v))，由此构造 R_des；
    内环用 e_R = ½·vee(R_desᵀR − RᵀR_des) 做 PD。
    """

    def __init__(self, params: QuadParams,
                 kp=(-16.0, -16.0, -20.0), kd=(-8.0, -8.0, -9.0),
                 kr=(-1.5, -1.5, -0.6), kw=(-0.10, -0.10, -0.07)):
        # 增益来自扫描实验：kp=-3 滑行 0.57 m / 超调 0.24 m，kp=-25 发散；
        # 默认 kp=-16 滑行 0.13 m、零超调

        # 负号约定：误差项按 K·(p − p_d) 写，K 取负值
        self.p = params
        self.kp = np.asarray(kp, dtype=float)
        self.kd = np.asarray(kd, dtype=float)
        self.kr = np.asarray(kr, dtype=float)
        self.kw = np.asarray(kw, dtype=float)
        self.max_tilt = np.deg2rad(35.0)   # 倾角上限
        #: 期望推力上限占 4·T_max 的比例
        self.thrust_alloc = 0.7

    def compute(self, st: QuadrotorState,
                p_d: np.ndarray, v_d: np.ndarray, a_d: np.ndarray,
                yaw_d: float) -> Tuple[float, np.ndarray]:
        """返回 (合推力 T, 力矩 [τx,τy,τz])。"""
        e_p = st.pos - p_d
        e_v = st.vel - v_d

        # 1) 期望推力矢量（世界系）
        acc = a_d + self.kp * e_p + self.kd * e_v
        f = self.p.mass * (acc + np.array([0.0, 0.0, 9.81]))
        # 限总推力：无上限时竞速场景实测 |f| 峰值 2269 N（上限 24 N），混控力矩
        # 预算 min(4T_max−T, T) 归零、姿态翻转。
        f_max = self.thrust_alloc * 4.0 * self.p.max_thrust
        nf = float(np.linalg.norm(f))
        if nf > f_max:
            f *= f_max / nf
        # 限倾角：水平分量不超过 tan(max_tilt) × 垂直分量
        fz = max(f[2], 0.15 * self.p.mass * 9.81)
        fh = np.linalg.norm(f[:2])
        lim = np.tan(self.max_tilt) * fz
        if fh > lim and fh > 1e-9:
            f[:2] *= lim / fh

        # 2) 由 f 与期望偏航构造期望姿态
        z_b = f / max(np.linalg.norm(f), 1e-9)
        x_c = np.array([np.cos(yaw_d), np.sin(yaw_d), 0.0])
        y_b = np.cross(z_b, x_c)
        n = np.linalg.norm(y_b)
        if n < 1e-6:                       # 期望偏航与推力方向共线时的退化处理
            y_b = np.array([-np.sin(yaw_d), np.cos(yaw_d), 0.0])
        else:
            y_b = y_b / n
        x_b = np.cross(y_b, z_b)
        R_des = np.column_stack([x_b, y_b, z_b])

        # 3) 姿态误差（机体系）
        R = quat_to_mat(st.quat)
        e_R = vee(R_des.T @ R - R.T @ R_des)
        e_w = st.omega

        tau_b = self.kr * e_R + self.kw * e_w
        T = float(np.dot(f, R[:, 2]))      # 合推力沿机体 z 的投影
        return T, tau_b

    def act(self, quad: Quadrotor, p_d, v_d, a_d, yaw_d) -> np.ndarray:
        """一步：读状态 → 算力矩 → 逆混控 → 返回四桨推力。"""
        st = quad.state()
        T, tau = self.compute(st, np.asarray(p_d, float), np.asarray(v_d, float),
                              np.asarray(a_d, float), float(yaw_d))
        return mixer(T, tau, quad.params)
