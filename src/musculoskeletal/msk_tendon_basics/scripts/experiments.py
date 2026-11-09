# -*- coding: utf-8 -*-
"""三组实验：等长传递、肌腱刚度权衡、松手释放。

所有实验都调用 ``muscle_tendon`` / ``joint_tendon`` 里的模型，不在这里
重复物理公式。输出是普通 Python 列表 / dict，方便画图和写 CSV。
"""

from __future__ import annotations

try:
    from .muscle_tendon import TendonParams, activation_step, mtu_state
    from .joint_tendon import JointParams, TendonJoint, _mtu_lengths
except ImportError:  # pragma: no cover
    from muscle_tendon import TendonParams, activation_step, mtu_state
    from joint_tendon import JointParams, TendonJoint, _mtu_lengths

__all__ = [
    "isometric_sweep",
    "tendon_stiffness_sweep",
    "release_experiment",
    "release_pair",
]

#: 近似刚性的肌腱刚度，用于对比
RIGID_KT = 1.0e12


def _sweep_activation(n):
    if n < 2:
        raise ValueError("n 至少为 2")
    return [i / float(n - 1) for i in range(n)]


def isometric_sweep(muscle_params=None, joint_params=None, n=21,
                    theta=0.0, kt_rigid=RIGID_KT):
    """固定关节角，扫描激活度，比较刚性肌腱与弹性肌腱。

    返回 dict：
        ``acts``       激活度列表
        ``compliant``  {force, l_m, l_t, strain, energy, fiber_length_norm}
        ``rigid``      同样字段（能量近似为 0）
    """
    mp = muscle_params if muscle_params is not None else TendonParams()
    jp = joint_params if joint_params is not None else JointParams()
    acts = _sweep_activation(n)

    out = {"acts": acts, "compliant": _empty_series(), "rigid": _empty_series()}
    for a in acts:
        for key, k_t in (("compliant", mp.tendon_stiffness),
                         ("rigid", kt_rigid)):
            flex, _ext = _both_at(jp, mp, theta, a, a, k_t)
            _append(out[key], flex)
    return out


def _both_at(jp, mp, theta, a_flex, a_ext, k_t):
    l_flex, l_ext = _mtu_lengths(jp, theta)
    return (mtu_state(l_flex, a_flex, mp, k_t),
            mtu_state(l_ext, a_ext, mp, k_t))


def _empty_series():
    return {"force": [], "l_m": [], "l_t": [], "strain": [],
            "energy": [], "fiber_length_norm": []}


def _append(series, state):
    series["force"].append(state["force"])
    series["l_m"].append(state["l_m"])
    series["l_t"].append(state["l_t"])
    series["strain"].append(state["tendon_strain"])
    series["energy"].append(state["energy"])
    series["fiber_length_norm"].append(state["fiber_length_norm"])


def tendon_stiffness_sweep(muscle_params=None, joint_params=None,
                           factors=None, activation=1.0, theta=0.0):
    """扫描肌腱刚度，看"传力"和"储能"怎么此消彼长。

    factors 是相对名义刚度 k_t 的倍数。返回 dict：
        ``factors``、``k_t``、``force``、``strain``、``energy``。
    """
    mp = muscle_params if muscle_params is not None else TendonParams()
    jp = joint_params if joint_params is not None else JointParams()
    if factors is None:
        factors = [0.5, 0.75, 1.0, 1.5, 2.0, 3.0, 5.0, 10.0]
    if not factors:
        raise ValueError("factors 不能为空")
    if any(f <= 0 for f in factors):
        raise ValueError("factors 必须为正")

    out = {"factors": [], "k_t": [], "force": [], "strain": [], "energy": []}
    for factor in factors:
        k_t = mp.tendon_stiffness * float(factor)
        flex, _ext = _both_at(jp, mp, theta, activation, activation, k_t)
        out["factors"].append(float(factor))
        out["k_t"].append(k_t)
        out["force"].append(flex["force"])
        out["strain"].append(flex["tendon_strain"])
        out["energy"].append(flex["energy"])
    return out


def _state_at(jp, mp, joint, theta, k_t):
    flex, ext = joint.muscles(theta=theta, k_t=k_t)
    tau = jp.moment_arm * (flex["force"] - ext["force"])
    return {
        "theta": theta,
        "theta_dot": 0.0,
        "a_flex": joint.a_flex,
        "a_ext": joint.a_ext,
        "f_flex": flex["force"],
        "f_ext": ext["force"],
        "tau": tau,
        "energy": flex["energy"] + ext["energy"],
        "strain_flex": flex["tendon_strain"],
        "strain_ext": ext["tendon_strain"],
    }


def release_experiment(muscle_params=None, joint_params=None, k_t=None,
                       hold_time=0.4, duration=1.2, dt=1e-3,
                       theta0=0.05, u_flex=0.9, u_ext=0.05):
    """等长保持 -> 松开，观察肌腱能量释放与关节运动。

    前 ``hold_time`` 秒关节被"刹车"固定，只让激活度上升；随后松开，
    关节在负载弹簧和肌肉力共同作用下运动。返回 history + metrics。
    """
    mp = muscle_params if muscle_params is not None else TendonParams()
    jp = joint_params if joint_params is not None else JointParams()
    if k_t is None:
        k_t = mp.tendon_stiffness
    if dt <= 0:
        raise ValueError("dt 必须为正")
    if duration <= hold_time:
        raise ValueError("duration 必须大于 hold_time")

    joint = TendonJoint(jp, mp, theta=theta0)
    n = int(round(duration / dt))
    for i in range(n):
        t = i * dt
        if t < hold_time:
            # 刹车：角度不动，只更新激活
            joint.a_flex = activation_step(u_flex, joint.a_flex, dt,
                                           mp.tau_act, mp.tau_deact)
            joint.a_ext = activation_step(u_ext, joint.a_ext, dt,
                                          mp.tau_act, mp.tau_deact)
            state = _state_at(jp, mp, joint, theta0, k_t)
        else:
            load = -jp.load_stiffness * (joint.theta - jp.load_rest)
            state = joint.step(u_flex, u_ext, dt, load_torque=load, k_t=k_t)
        joint.record(t, state)

    return {"history": joint.history, "metrics": _release_metrics(
        joint.history, hold_time, dt)}


def _release_metrics(history, hold_time, dt):
    t = history["t"]
    hold_idx = max(0, min(len(t) - 1, int(round(hold_time / dt)) - 1))
    e_hold = history["energy"][hold_idx]
    e_final = history["energy"][-1]
    peak_theta = max(history["theta"])
    release_idx = min(len(t) - 1, hold_idx + 1)
    e_after = min(history["energy"][release_idx:]) if release_idx < len(t) else e_final
    released = (e_hold - e_after) / e_hold if e_hold > 1e-7 else 0.0
    return {
        "hold_force": history["f_flex"][hold_idx],
        "hold_energy": e_hold,
        "final_energy": e_final,
        "min_energy_after_release": e_after,
        "released_fraction": released,
        "peak_theta": peak_theta,
        "final_theta": history["theta"][-1],
    }


def release_pair(muscle_params=None, joint_params=None, **kwargs):
    """同一场景下跑"弹性肌腱"和"刚性肌腱"两条曲线，方便对比。"""
    mp = muscle_params if muscle_params is not None else TendonParams()
    jp = joint_params if joint_params is not None else JointParams()
    compliant = release_experiment(mp, jp, k_t=mp.tendon_stiffness, **kwargs)
    rigid = release_experiment(mp, jp, k_t=RIGID_KT, **kwargs)
    return {"compliant": compliant, "rigid": rigid}
