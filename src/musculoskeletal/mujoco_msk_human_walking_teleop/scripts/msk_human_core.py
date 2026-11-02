#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MuJoCo 平面肌肉骨骼人仿真核心。

模型：models/scene_*.xml（平地 / 5 度坡 / 负载），浮动基座 3 DoF +
下肢 6 + 上肢 4，共 13 DoF，由 22 条 Hill 型肌肉驱动。肌肉力学在
Python 中显式实现（一阶激活动力学 + Hill 力-长度-速度曲线），XML 中
每个肌肉对应一个 motor 执行器（gear=±1 力臂系数），ctrl 即张力 T。

分层控制（见文档）：
  动作编排器 Composer  —— 键盘指令 / 预设动作 -> 关节参考轨迹 q_ref(t)
  姿势反射层 Reflex    —— q_ref 与当前关节角的误差 -> 拮抗肌激活（脊髓反射）
  键盘手动激活          —— 叠加在反射输出上，直接兴奋指定肌肉
  MuJoCo 肌肉动力学     —— 一阶激活动力学 + Hill 力-长度-速度曲线 -> 力矩

本文件直接运行执行模型自检：python3 msk_human_core.py
"""
import json
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_MODEL_DIR = os.path.join(os.path.dirname(_HERE), "models")
if not os.path.isdir(_MODEL_DIR):  # catkin 安装后 models 与脚本同级
    _MODEL_DIR = os.path.join(os.path.dirname(_HERE), "models")

import mujoco

SCENES = ("flat", "slope5", "load")

MUSCLE_NAMES = [
    "abs", "erector",
    "hipfl_l", "hipe_l", "kneef_l", "kneee_l", "ankp_l", "ankd_l",
    "hipfl_r", "hipe_r", "kneef_r", "kneee_r", "ankp_r", "ankd_r",
    "shof_l", "shoe_l", "elbf_l", "elbe_l",
    "shof_r", "shoe_r", "elbf_r", "elbe_r",
]
N_MUSCLE = len(MUSCLE_NAMES)
MUSCLE_ID = {n: i for i, n in enumerate(MUSCLE_NAMES)}

JOINT_NAMES = ["root_x", "root_z", "root_pitch",
               "hip_l", "knee_l", "ankle_l", "hip_r", "knee_r", "ankle_r",
               "shoulder_l", "elbow_l", "shoulder_r", "elbow_r"]
N_JOINT = len(JOINT_NAMES)
JOINT_ID = {n: i for i, n in enumerate(JOINT_NAMES)}

# 各关节活动范围（与 XML 一致），用于肌肉最优长度/半宽参数
JOINT_RANGE = {
    "root_pitch": (-0.7, 0.7), "hip_l": (-0.9, 1.8), "knee_l": (-0.05, 2.2),
    "ankle_l": (-0.75, 0.45), "hip_r": (-0.9, 1.8), "knee_r": (-0.05, 2.2),
    "ankle_r": (-0.75, 0.45), "shoulder_l": (-0.7, 2.0), "elbow_l": (0.0, 2.4),
    "shoulder_r": (-0.7, 2.0), "elbow_r": (0.0, 2.4),
}

# Hill 肌肉参数：(关节, 力臂系数 coef, 峰值力 F0 [N·m], 最大缩短速度 [rad/s])
# coef=+1 屈肌（正力矩=屈），coef=-1 伸肌；q_opt 取关节范围中点，sigma=range/2
_MUSCLE_SPEC = {
    "abs":     ("root_pitch", +1, 280.0, 8.0),
    "erector": ("root_pitch", -1, 320.0, 8.0),
    "hipfl_l": ("hip_l", +1, 180.0, 12.0), "hipe_l": ("hip_l", -1, 240.0, 12.0),
    "kneef_l": ("knee_l", +1, 140.0, 14.0), "kneee_l": ("knee_l", -1, 290.0, 14.0),
    "ankp_l":  ("ankle_l", -1, 200.0, 12.0), "ankd_l": ("ankle_l", +1, 60.0, 12.0),
    "hipfl_r": ("hip_r", +1, 180.0, 12.0), "hipe_r": ("hip_r", -1, 240.0, 12.0),
    "kneef_r": ("knee_r", +1, 140.0, 14.0), "kneee_r": ("knee_r", -1, 290.0, 14.0),
    "ankp_r":  ("ankle_r", -1, 200.0, 12.0), "ankd_r": ("ankle_r", +1, 60.0, 12.0),
    "shof_l":  ("shoulder_l", +1, 50.0, 14.0), "shoe_l": ("shoulder_l", -1, 50.0, 14.0),
    "elbf_l":  ("elbow_l", +1, 35.0, 14.0), "elbe_l": ("elbow_l", -1, 35.0, 14.0),
    "shof_r":  ("shoulder_r", +1, 50.0, 14.0), "shoe_r": ("shoulder_r", -1, 50.0, 14.0),
    "elbf_r":  ("elbow_r", +1, 35.0, 14.0), "elbe_r": ("elbow_r", -1, 35.0, 14.0),
}

TAU_ACT, TAU_DEACT = 0.012, 0.05        # 激活/去激活时间常数 s

# 拮抗肌对：(关节, 屈肌, 伸肌)
ANTAGONIST = [
    ("root_pitch", "abs", "erector"),
    ("hip_l", "hipfl_l", "hipe_l"), ("knee_l", "kneef_l", "kneee_l"),
    ("ankle_l", "ankd_l", "ankp_l"),
    ("hip_r", "hipfl_r", "hipe_r"), ("knee_r", "kneef_r", "kneee_r"),
    ("ankle_r", "ankd_r", "ankp_r"),
    ("shoulder_l", "shof_l", "shoe_l"), ("elbow_l", "elbf_l", "elbe_l"),
    ("shoulder_r", "shof_r", "shoe_r"), ("elbow_r", "elbf_r", "elbe_r"),
]

# 反射层使用的关节（肢体 + 躯干俯仰），不含浮动基座平移
REFLEX_JOINTS = [a[0] for a in ANTAGONIST]

THIGH_L, SHIN_L = 0.42, 0.40           # 大腿/小腿连杆长（与 XML 一致）
ANKLE_HEIGHT = 0.055                    # 踝关节离地高度


def _default_params():
    return {
        "reflex": {
            "kp": {"root_pitch": 320.0, "hip_l": 320.0, "knee_l": 300.0,
                   "ankle_l": 400.0, "hip_r": 320.0, "knee_r": 300.0,
                   "ankle_r": 400.0, "shoulder_l": 40.0, "elbow_l": 30.0,
                   "shoulder_r": 40.0, "elbow_r": 30.0},
            "kd": {"root_pitch": 26.0, "hip_l": 10.0, "knee_l": 6.0,
                   "ankle_l": 30.0, "hip_r": 10.0, "knee_r": 6.0,
                   "ankle_r": 30.0, "shoulder_l": 3.0, "elbow_l": 2.0,
                   "shoulder_r": 3.0, "elbow_r": 2.0},
            "cocon": {"root_pitch": 18.0, "hip_l": 10.0, "knee_l": 12.0,
                      "ankle_l": 10.0, "hip_r": 10.0, "knee_r": 12.0,
                      "ankle_r": 10.0, "shoulder_l": 1.5, "elbow_l": 1.2,
                      "shoulder_r": 1.5, "elbow_r": 1.2},
        },
        "walk": {
            "freq": 0.8, "hip_amp": 0.50, "knee_amp": 0.75, "knee_off": 0.10,
            "ankle_amp": 0.25, "ankle_off": -0.05, "ankle_phase": 0.4,
            "arm_amp": 0.30, "pitch_ref": 0.10,
        },
        "squat": {"freq": 0.5, "depth": 0.32, "z_stand": 0.875},
        "kick": {"T": 1.6},
    }


def load_params(path=None):
    if path is None:
        # 1) 源码树内直接查找（离线运行 check/eval/demo）
        path = os.path.join(os.path.dirname(_HERE), "config", "params.yaml")
        # 2) catkin 安装后 _HERE 位于 devel/lib/<pkg>，其下没有 config；
        #    通过 roslib 定位源码包目录，保证 roslaunch 时 yaml 真正生效
        if not os.path.isfile(path):
            try:
                import roslib.packages
                pkg = roslib.packages.get_pkg_dir(
                    "mujoco_msk_human_walking_teleop")
                path = os.path.join(pkg, "config", "params.yaml")
            except Exception:
                path = None
    if path and os.path.isfile(path):
        try:
            import yaml
            with open(path) as f:
                user = yaml.safe_load(f) or {}
            params = _default_params()
            for k, v in user.items():
                if isinstance(v, dict) and k in params:
                    params[k].update(v)
                else:
                    params[k] = v
            return params
        except Exception:
            pass
    else:
        print("[msk_human] 未找到 params.yaml，使用内置默认参数",
              file=sys.stderr)
    return _default_params()


def _model_path(scene):
    name = "scene_flat.xml" if scene == "flat" else \
           "scene_slope5.xml" if scene == "slope5" else "scene_load.xml"
    return os.path.join(_MODEL_DIR, name)


class MskHumanSim:
    """MuJoCo 仿真封装：加载场景、步进、状态读取、负载添加、摔倒判定。"""

    def __init__(self, scene="flat", viewer=False, load_mass=10.0):
        self.scene = scene
        self.model = mujoco.MjModel.from_xml_path(_model_path(scene))
        self.data = mujoco.MjData(self.model)
        if scene == "load":           # 10 kg 背包 -> 躯干质量与惯量
            i = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "torso")
            m_old = self.model.body_mass[i]
            m_new = m_old + load_mass
            self.model.body_mass[i] = m_new
            self.model.body_inertia[i] *= m_new / m_old
            mujoco.mj_setConst(self.model, self.data)
        mujoco.mj_resetData(self.model, self.data)
        self._viewer = None
        if viewer:
            from mujoco import viewer as mj_viewer
            self._viewer = mj_viewer.launch_passive(self.model, self.data)

        # 关节地址
        self.q_adr = [self.model.jnt_qposadr[
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)]
            for n in JOINT_NAMES]
        self.v_adr = [self.model.jnt_dofadr[
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)]
            for n in JOINT_NAMES]
        self.muscle_adr = [mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in MUSCLE_NAMES]
        self.torso_bid = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "torso")
        self.foot_id = {s: mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "foot_%s_box" % s)
            for s in ("l", "r")}

        # Hill 肌肉参数预计算：q_opt 取关节范围中点，sigma = range/2
        # （宽高斯保证全程力-长度因子 >= ~0.6，避免肢体在伸展位"没力气"）
        self._q_opt = np.zeros(N_MUSCLE)
        self._sigma2 = np.zeros(N_MUSCLE)
        self._coef = np.zeros(N_MUSCLE)
        self._F0 = np.zeros(N_MUSCLE)
        self._vmax = np.zeros(N_MUSCLE)
        for i, name in enumerate(MUSCLE_NAMES):
            jname, coef, F0, vmax = _MUSCLE_SPEC[name]
            lo, hi = JOINT_RANGE[jname]
            self._q_opt[i] = 0.5 * (lo + hi)
            self._sigma2[i] = ((hi - lo) / 2.0) ** 2
            self._coef[i] = coef
            self._F0[i] = F0
            self._vmax[i] = vmax
        self._joint_of_muscle = np.array(
            [JOINT_ID[_MUSCLE_SPEC[n][0]] for n in MUSCLE_NAMES])
        self.act = np.zeros(N_MUSCLE)       # 激活状态 a in [0,1]
        self.tension = np.zeros(N_MUSCLE)   # 肌肉张力 T [N·m]（恒 >= 0）

    # ---------- 状态 ----------
    def q(self):
        return np.array([self.data.qpos[a] for a in self.q_adr])

    def qd(self):
        return np.array([self.data.qvel[a] for a in self.v_adr])

    def pitch(self):
        return self.data.qpos[self.q_adr[2]]

    def pelvis_z(self):
        return float(self.data.xpos[self.torso_bid][2])

    def pelvis_x(self):
        return float(self.data.xpos[self.torso_bid][0])

    @property
    def time(self):
        """仿真时钟 [s]。"""
        return float(self.data.time)

    def toe_pos(self, side="r"):
        """脚尖（脚盒最前点）世界坐标 z。"""
        gid = self.foot_id[side]
        pos = self.data.geom_xpos[gid]
        mat = self.data.geom_xmat[gid].reshape(3, 3)
        tip_local = np.array([0.115, 0.0, -0.030])
        return pos + mat.dot(tip_local)

    def fallen(self):
        return self.pelvis_z() < 0.50 or abs(self.pitch()) > 0.9

    def muscle_forces(self):
        """22 维肌肉张力 [N·m]（Hill 模型显式状态）。"""
        return self.tension.copy()

    # ---------- 步进 ----------
    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        self.act[:] = 0.0
        self.tension[:] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def _hill_update(self, u, dt):
        """显式 Hill 肌肉力学：激活动力学 + 力-长度-速度曲线 -> 张力 ctrl。

        激活动力学：  a' = (u - a) / tau，tau = TAU_ACT(u>a) / TAU_DEACT(u<=a)
        力-长度：    fl(q) = exp(-(q - q_opt)^2 / (2 sigma^2))
        力-速度：    缩短 fv = (1 - v^) / (1 + 3 v^)；拉长 fv = min(1.4, 1 - 1.8 v^)
        张力：       T = a * F0 * fl * fv >= 0（肌肉只能拉不能推）
        施加：       motor ctrl = T，gear=coef 使关节力矩 tau = coef * T
        """
        q = self.q()
        qd = self.qd()
        tau_t = np.where(u > self.act, TAU_ACT, TAU_DEACT)
        self.act = np.clip(self.act + (u - self.act) / tau_t * dt, 0.0, 1.0)
        jm = self._joint_of_muscle
        v = self._coef * qd[jm]                    # 肌肉缩短速度 [rad/s]
        vh = np.clip(v / self._vmax, -3.0, 3.0)    # 归一化缩短速度
        fv = np.where(
            vh > 0.0,
            (1.0 - vh) / np.where(vh > 0.0, 1.0 + 3.0 * vh, 1.0),
            np.minimum(1.4, 1.0 - 1.8 * vh))
        dq = q[jm] - self._q_opt
        fl = np.exp(-dq * dq / (2.0 * self._sigma2))
        self.tension = np.clip(
            self.act * self._F0 * fl * np.clip(fv, 0.0, 1.4), 0.0, None)
        for a, val in zip(self.muscle_adr, self.tension):
            self.data.ctrl[a] = val

    def step(self, u, n=1):
        """u: 22 维肌肉激活目标（0-1），经 Hill 力学滤波后由 motor 施加。"""
        u = np.clip(np.asarray(u, dtype=float), 0.0, 1.0)
        dt = self.model.opt.timestep
        for _ in range(n):
            self._hill_update(u, dt)
            mujoco.mj_step(self.model, self.data)
        if self._viewer is not None and self._viewer.is_running():
            self._viewer.sync()

    def close(self):
        if self._viewer is not None:
            self._viewer.close()
            self._viewer = None

    # ---------- 指标 ----------
    def effort(self, u, dt):
        """激活努力 ∫|u|dt 与机械功 ∫|τ·q̇|⁺dt（正功）。"""
        qd = self.qd()
        tau = np.zeros(N_JOINT)                    # tau = Σ coef * T（gear 施加）
        for i, name in enumerate(MUSCLE_NAMES):
            tau[self._joint_of_muscle[i]] += self._coef[i] * self.tension[i]
        power = sum(max(0.0, tau[j] * qd[j]) for j in range(2, N_JOINT))
        self._acc = getattr(self, "_acc", {"act": 0.0, "work": 0.0})
        self._acc["act"] += float(np.abs(u).sum()) * dt
        self._acc["work"] += power * dt
        return self._acc


def leg_ik_squat(z_pelvis, torso_pitch=0.0):
    """给定骨盆高度，求双脚支撑蹲姿的髋/膝/踝角（双脚平放、躯干竖直）。

    平面二连杆逆解：h 为髋关节到踝的竖直距离。
    """
    h = z_pelvis - ANKLE_HEIGHT
    h = max(min(h, THIGH_L + SHIN_L - 1e-4), abs(THIGH_L - SHIN_L) + 1e-4)
    cos_g = (THIGH_L ** 2 + SHIN_L ** 2 - h ** 2) / (2 * THIGH_L * SHIN_L)
    gamma = math.acos(max(-1.0, min(1.0, cos_g)))     # 膝内角
    knee = math.pi - gamma                              # 屈曲角
    cos_a = (THIGH_L ** 2 + h ** 2 - SHIN_L ** 2) / (2 * THIGH_L * h)
    alpha = math.acos(max(-1.0, min(1.0, cos_a)))       # 大腿与竖直线夹角
    hip = alpha                                          # 前屈为正
    ankle = knee - hip                                   # 脚平放
    return hip, knee, ankle


class Composer:
    """动作编排 + 反射层 + 手动叠加 -> 22 维肌肉激活目标。

    动作：stand / walk / squat / kick(单次) / relax
    """

    def __init__(self, params=None):
        self.p = params or load_params()
        self.action = "stand"
        self.phase = 0.0
        self.freq_scale = 1.0
        self.t_action = 0.0
        self.kick_done = True
        self.manual = np.zeros(N_MUSCLE)   # 手动激活叠加

    # ---------- 指令 ----------
    def set_action(self, name):
        if name == "kick":
            self.t_action, self.kick_done = 0.0, False
        elif name != self.action:
            self.t_action = 0.0
            self.phase = 0.0
        if name in ("stand", "walk", "squat", "relax", "kick"):
            self.action = name
        if name == "relax":
            self.manual[:] = 0.0

    def nudge_speed(self, d):
        self.freq_scale = float(np.clip(self.freq_scale + d, 0.5, 1.5))

    def set_manual(self, idx, value):
        self.manual[idx] = float(np.clip(value, 0.0, 1.0))

    def clear_manual(self):
        self.manual[:] = 0.0

    # ---------- 参考轨迹 ----------
    def _q_ref(self, dt):
        w = self.p["walk"]
        ref = {j: 0.0 for j in REFLEX_JOINTS}
        ref["knee_l"] = ref["knee_r"] = 0.03
        ref["elbow_l"] = ref["elbow_r"] = 0.25
        if self.action == "walk":
            self.phase += 2 * math.pi * w["freq"] * self.freq_scale * dt
            phi_l, phi_r = self.phase, self.phase + math.pi
            for phi, s in ((phi_l, "l"), (phi_r, "r")):
                ref["hip_" + s] = w["hip_amp"] * math.sin(phi)
                ref["knee_" + s] = w["knee_off"] + w["knee_amp"] * \
                    max(0.0, math.sin(phi)) ** 1.2
                ref["ankle_" + s] = w["ankle_off"] + w["ankle_amp"] * \
                    math.sin(phi + w["ankle_phase"])
            ref["shoulder_l"] = -w["arm_amp"] * math.sin(phi_l)
            ref["shoulder_r"] = -w["arm_amp"] * math.sin(phi_r)
            ref["root_pitch"] = w["pitch_ref"]
        elif self.action == "squat":
            sq = self.p["squat"]
            self.t_action += dt
            z = sq["z_stand"] - 0.5 * sq["depth"] * \
                (1.0 - math.cos(2 * math.pi * sq["freq"] * self.t_action))
            hip, knee, ankle = leg_ik_squat(z)
            for s in ("l", "r"):
                ref["hip_" + s], ref["knee_" + s], ref["ankle_" + s] = hip, knee, ankle
            ref["root_pitch"] = 0.0
            ref["shoulder_l"] = ref["shoulder_r"] = 1.2   # 前平举配重
        elif self.action == "kick" and not self.kick_done:
            self.t_action += dt
            t = self.t_action
            T = self.p["kick"]["T"]
            if t >= T:
                self.kick_done = True
            else:
                # 四阶段：重心转移 -> 提膝蓄力 -> 伸膝踢击 -> 收腿还原
                # 支撑腿（左）保持微屈稳定支撑；踢击腿（右）先高屈膝蓄力，
                # 再快速伸膝（股四头肌爆发），踝跖屈绷脚尖，脚尖高度由
                # 髋屈曲角与膝伸展角共同决定
                if t < 0.30:                 # 重心转移，右腿准备
                    k = t / 0.30
                    ref["knee_l"] = 0.20
                    ref["hip_r"], ref["knee_r"] = 0.05 * k, 0.30 * k
                    ref["shoulder_l"], ref["shoulder_r"] = 0.4 * k, -0.4 * k
                elif t < 0.65:               # 提膝蓄力（髋屈 0.95、膝屈 1.55）
                    k = (t - 0.30) / 0.35
                    ref["knee_l"] = 0.20
                    ref["hip_r"] = 0.05 + (0.95 - 0.05) * k
                    ref["knee_r"] = 0.30 + (1.55 - 0.30) * k
                    ref["ankle_r"] = -0.30 * k
                    ref["shoulder_l"], ref["shoulder_r"] = 0.4, -0.4
                elif t < 0.95:               # 伸膝踢击：膝 1.55 -> 0.15 快速伸展
                    k = (t - 0.65) / 0.30
                    ref["knee_l"] = 0.20
                    ref["hip_r"] = 0.95 + (1.20 - 0.95) * k
                    ref["knee_r"] = 1.55 + (0.15 - 1.55) * k
                    ref["ankle_r"] = -0.30 + (-0.45 + 0.30) * k
                    ref["root_pitch"] = 0.04 * k   # 轻微前倾配重
                    ref["shoulder_l"], ref["shoulder_r"] = 0.4, -0.4
                else:                        # 收腿还原（平滑 s 曲线）
                    k = min(1.0, (t - 0.95) / (T - 0.95))
                    e = 0.5 * (1.0 - math.cos(math.pi * k))
                    ref["knee_l"] = 0.20 * (1 - e) + 0.03 * e
                    ref["hip_r"] = 1.20 * (1 - e)
                    ref["knee_r"] = 0.15 + (0.03 - 0.15) * e
                    ref["ankle_r"] = -0.45 * (1 - e)
                    ref["root_pitch"] = 0.04 * (1 - e)
                    ref["shoulder_l"] = 0.4 * (1 - e)
                    ref["shoulder_r"] = -0.4 * (1 - e)
        return ref

    # ---------- 反射层 ----------
    def _reflex(self, sim):
        q, qd = sim.q(), sim.qd()
        r = self.p["reflex"]
        u = np.zeros(N_MUSCLE)
        for jname, flex, ext in ANTAGONIST:
            j = JOINT_NAMES.index(jname)
            e = self._ref[jname] - q[j]
            tau = r["kp"][jname] * e - r["kd"][jname] * qd[j]
            # 共收缩：对拮抗肌施加等大小力矩 b（按各自 F0 换算激活），
            # 净力矩恒为 0，只提供关节刚度；反射力矩 tau 叠加在上限方向
            b = r["cocon"][jname]
            a_f = (b + max(0.0, tau)) / _MUSCLE_SPEC[flex][2]
            a_e = (b + max(0.0, -tau)) / _MUSCLE_SPEC[ext][2]
            u[MUSCLE_ID[flex]] = min(1.0, a_f)
            u[MUSCLE_ID[ext]] = min(1.0, a_e)
        return u

    # ---------- 组合 ----------
    def update(self, sim, dt):
        self._ref = self._q_ref(dt)
        u = self._reflex(sim)
        if self.action == "relax":
            u[:] = 0.0
        return np.clip(u + self.manual, 0.0, 1.0)


# ============================== 自检 ==============================
def run_check():
    import sys
    results = []

    def check(name, ok, detail=""):
        results.append((name, bool(ok), detail))
        print("[%s] %s %s" % ("PASS" if ok else "FAIL", name, detail))

    p = load_params()
    sims = {}
    for sc in SCENES:
        try:
            sims[sc] = MskHumanSim(sc)
            check("load_scene_%s" % sc, True, "")
        except Exception as e:
            check("load_scene_%s" % sc, False, str(e))
            return results

    m = sims["flat"].model
    check("n_muscle", m.nu == N_MUSCLE, "nu=%d" % m.nu)
    check("n_joint", m.njnt == N_JOINT, "njnt=%d" % m.njnt)

    # 每块肌肉的兴奋方向：单独激活 0.5 s，关节应朝定义方向运动
    sim = sims["flat"]
    comp = Composer(p)
    comp.set_action("relax")
    sign_map = {
        "abs": ("root_pitch", +1), "erector": ("root_pitch", -1),
        "hipfl_l": ("hip_l", +1), "hipe_l": ("hip_l", -1),
        "kneef_l": ("knee_l", +1), "kneee_l": ("knee_l", -1),
        "ankp_l": ("ankle_l", -1), "ankd_l": ("ankle_l", +1),
        "hipfl_r": ("hip_r", +1), "hipe_r": ("hip_r", -1),
        "kneef_r": ("knee_r", +1), "kneee_r": ("knee_r", -1),
        "ankp_r": ("ankle_r", -1), "ankd_r": ("ankle_r", +1),
        "shof_l": ("shoulder_l", +1), "shoe_l": ("shoulder_l", -1),
        "elbf_l": ("elbow_l", +1), "elbe_l": ("elbow_l", -1),
        "shof_r": ("shoulder_r", +1), "shoe_r": ("shoulder_r", -1),
        "elbf_r": ("elbow_r", +1), "elbe_r": ("elbow_r", -1),
    }
    bad = []
    g_backup = sim.model.opt.gravity.copy()
    sim.model.opt.gravity[:] = 0.0      # 零重力：只看肌肉力引起的运动
    # 各关节测试起始角：默认 range 中点；肘用小角度（大屈肘会与前臂-大腿
    # 自碰撞，污染测试）
    test_at = {"elbow_l": 0.2, "elbow_r": 0.2}
    for name, (jn, want) in sign_map.items():
        sim.reset()
        # 抬离地面 0.2 m：避免力矩经足-地接触短路（伸膝=蹬地）
        sim.data.qpos[sim.q_adr[1]] += 0.2
        lo, hi = JOINT_RANGE[jn]
        sim.data.qpos[sim.q_adr[JOINT_NAMES.index(jn)]] = \
            test_at.get(jn, 0.5 * (lo + hi))
        mujoco.mj_forward(sim.model, sim.data)
        q0 = sim.q()[JOINT_NAMES.index(jn)]
        u = np.zeros(N_MUSCLE)
        u[MUSCLE_ID[name]] = 0.6
        j = JOINT_NAMES.index(jn)
        for _ in range(75):             # 最多 0.15 s，方向一明确即停
            sim.step(u)
            if abs(sim.q()[j] - q0) > 0.05:   # 避免摆到限位被弹回
                break
        q1 = sim.q()[j]
        if (q1 - q0) * want <= 0.0:
            bad.append("%s(%+.3f)" % (name, q1 - q0))
    sim.model.opt.gravity[:] = g_backup
    check("muscle_sign", not bad, ",".join(bad) if bad else "22/22 方向正确")

    # 站立稳定性：反射层站立 6 s
    for sc in SCENES:
        sim = sims[sc]
        sim.reset()
        comp = Composer(p)
        comp.set_action("stand")
        dt, n = 0.002, 3000
        fell = False
        for i in range(n):
            u = comp.update(sim, dt)
            sim.step(u)
            if sim.fallen():
                fell = True
                break
        z, pit = sim.pelvis_z(), sim.pitch()
        check("stand_6s_%s" % sc, not fell and abs(pit) < 0.25,
              "z=%.3f pitch=%+.3f t=%.1f" % (z, pit, i * dt))

    # 蹲起 IK 校验：z=0.63 时膝角应显著大于 0
    hip, knee, ankle = leg_ik_squat(0.63)
    check("squat_ik", 0.4 < knee < 2.0 and abs(hip + ankle - knee) < 1e-9,
          "hip=%.2f knee=%.2f ankle=%.2f" % (hip, knee, ankle))

    # 负载场景质量校验
    m0 = mujoco.MjModel.from_xml_path(_model_path("flat"))
    m1 = MskHumanSim("load").model
    i = mujoco.mj_name2id(m1, mujoco.mjtObj.mjOBJ_BODY, "torso")
    j = mujoco.mj_name2id(m0, mujoco.mjtObj.mjOBJ_BODY, "torso")
    check("load_mass", abs((m1.body_mass[i] - m0.body_mass[j]) - 10.0) < 1e-6,
          "torso mass %.1f -> %.1f" % (m0.body_mass[j], m1.body_mass[i]))

    n_fail = sum(1 for _, ok, _ in results if not ok)
    print("---- check: %d/%d passed ----" % (len(results) - n_fail, len(results)))
    for s in sims.values():
        s.close()
    return results


if __name__ == "__main__":
    run_check()
