"""MuJoCo 关节版正模型：真实限位 + 重力，用于参数辨识。

与解析版的区别：关节限位由 MuJoCo 约束求解处理（平滑的软约束），
重力、惯量也由 MuJoCo 计算，因此激励设计实验（正弦/chirp 推到限位）
在物理上是成立的。
"""

from __future__ import annotations

import numpy as np

DT = 0.002
T_END = 2.0
N_STEPS = int(T_END / DT)

L0 = 0.25
M_FORE = 1.2
L_FORE = 0.25
THETA_LIMIT = 1.40          # 关节限位 (rad)

TRUE = {"fmax": 320.0, "l_opt": 0.275, "tau_act": 0.010,
        "r": 0.030, "b": 0.050}
PARAM_NAMES = ["fmax", "l_opt", "tau_act", "r", "b"]
# 归一化尺度：各参数除以标称值，使 LM 在等尺度空间中搜索
SCALE = np.array([TRUE[k] for k in PARAM_NAMES])
BOUNDS = {"fmax": (150.0, 600.0), "l_opt": (0.20, 0.36),
          "tau_act": (0.003, 0.030), "r": (0.015, 0.050),
          "b": (0.005, 0.200)}

XML = f"""
<mujoco model="elbow_ident">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="{DT}" integrator="implicitfast" gravity="0 0 -9.81"/>
  <worldbody>
    <body name="upper">
      <geom type="capsule" size="0.03" fromto="0 0 0 0 0 -0.02"
            mass="2.0" contype="0" conaffinity="0"/>
      <body name="fore" pos="0 0 0">
        <joint name="elbow" type="hinge" axis="0 1 0"
               range="-{THETA_LIMIT} {THETA_LIMIT}" damping="0.001"/>
        <geom type="capsule" size="0.028" fromto="0 0 0 0 0 -{L_FORE}"
              mass="{M_FORE}" contype="0" conaffinity="0"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"""


def fl(ln):
    return float(np.exp(-((ln - 1.0) / 0.45) ** 2))


def fp(ln):
    if ln <= 1.0:
        return 0.0
    return float((np.exp(5.0 * (ln - 1.0)) - 1.0) / (np.exp(5.0 * 0.6) - 1.0))


def fv(vn):
    if vn >= 0:
        return float(0.3125 / (vn + 0.25) - 0.25)
    x = min(1.0, -vn)
    return float(1.0 + 0.4 * x / (0.25 + x))


# ---------------- 激励设计 ----------------

def profile_square(t, scale=0.15):
    sq = 0.5 * (1.0 + np.sign(np.sin(2 * np.pi * 1.2 * t)))
    uf = 0.10 + scale * sq * (0.6 + 0.4 * np.sin(2 * np.pi * 0.7 * t))
    ue = 0.05 + 0.08 * (1.0 - sq)
    return float(np.clip(uf, 0, 1)), float(np.clip(ue, 0, 1))


def profile_sine(t, scale=0.35):
    w = 2 * np.pi * 1.0
    uf = 0.10 + scale * 0.9 * (0.5 + 0.5 * np.sin(w * t))
    ue = 0.05 + scale * 0.5 * (0.5 - 0.5 * np.sin(w * t))
    return float(np.clip(uf, 0, 1)), float(np.clip(ue, 0, 1))


def profile_chirp(t, scale=0.30, f0=0.4, f1=2.4, T=T_END):
    k = (f1 - f0) / T
    phase = 2 * np.pi * (f0 * t + 0.5 * k * t * t)
    uf = 0.10 + scale * 0.9 * (0.5 + 0.5 * np.sin(phase))
    ue = 0.05 + scale * 0.4 * (0.5 - 0.5 * np.sin(phase))
    return float(np.clip(uf, 0, 1)), float(np.clip(ue, 0, 1))


PROFILES = {"square+sine": profile_square, "sine": profile_sine,
            "chirp": profile_chirp}


def simulate(params, theta0=0.35, dt=DT, n=N_STEPS, u_fn=profile_square):
    """用 MuJoCo 积分，返回关节角序列。"""
    import mujoco

    m = mujoco.MjModel.from_xml_string(XML)
    d = mujoco.MjData(m)
    d.qpos[0] = theta0
    mujoco.mj_forward(m, d)

    fmax, l_opt, tau_act = params["fmax"], params["l_opt"], params["tau_act"]
    r, b = params["r"], params["b"]
    v_max = 10.0 * l_opt

    a_flex, a_ext = 0.2, 0.05
    out = np.empty(n)
    for i in range(n):
        t = i * dt
        theta = float(d.qpos[0])
        omega = float(d.qvel[0])
        u_flex, u_ext = u_fn(t)

        L = L0 - r * theta
        Ld = -r * omega
        ln, vn = L / l_opt, -Ld / v_max

        k_f = min(1.0, dt / tau_act) if u_flex > a_flex else min(1.0, dt / (4 * tau_act))
        k_e = min(1.0, dt / tau_act) if u_ext > a_ext else min(1.0, dt / (4 * tau_act))
        a_flex = float(np.clip(a_flex + (u_flex - a_flex) * k_f, 0, 1))
        a_ext = float(np.clip(a_ext + (u_ext - a_ext) * k_e, 0, 1))

        f_flex = fmax * (a_flex * fl(ln) * fv(vn) + fp(ln))
        f_ext = fmax * (a_ext * fl(ln) * fv(vn) + fp(ln))
        d.qfrc_applied[0] = (f_flex - f_ext) * r - b * omega

        mujoco.mj_step(m, d)
        out[i] = float(d.qpos[0])
    return out


def make_measurement(u_fn=profile_square, noise_pct=0.0, seed=0, **overrides):
    params = dict(TRUE)
    params.update(overrides)
    clean = simulate(params, u_fn=u_fn)
    rng = np.random.default_rng(seed)
    if noise_pct > 0:
        measured = clean + rng.normal(0.0, noise_pct / 100.0 * np.std(clean),
                                      size=clean.shape)
    else:
        measured = clean
    return measured, clean


def unpack(vec):
    return {k: float(v) for k, v in zip(PARAM_NAMES, vec)}


def clip_params(vec):
    return np.array([np.clip(v, *BOUNDS[k]) for v, k in zip(vec, PARAM_NAMES)])


def residuals(vec, measured, theta0=0.35, u_fn=profile_square):
    return simulate(unpack(clip_params(vec)), theta0=theta0, u_fn=u_fn) - measured


def lm_fit(p0, measured, theta0=0.35, u_fn=profile_square,
           max_iter=40, lam0=1e-2):
    """在归一化参数空间 (p / SCALE) 里做 LM，改善数值条件。"""
    p = clip_params(np.asarray(p0, dtype=float))
    lam = lam0
    x = p / SCALE                       # 归一化变量（各分量约等于 1）

    def res_x(xv):
        return residuals(xv * SCALE, measured, theta0, u_fn)

    r = res_x(x)
    cost = float(r @ r)
    n_par = len(p)
    A = None
    for it in range(max_iter):
        J = np.empty((len(r), n_par))
        for k in range(n_par):
            dx = np.zeros(n_par)
            dx[k] = 1e-5
            J[:, k] = (res_x(x + dx) - r) / dx[k]
        A = J.T @ J
        g = J.T @ r
        improved = False
        delta = np.zeros(n_par)
        for _ in range(8):
            try:
                delta = np.linalg.solve(A + lam * np.diag(np.diag(A) + 1e-12), -g)
            except np.linalg.LinAlgError:
                lam *= 10.0
                continue
            x_new = x + delta
            p_new = clip_params(x_new * SCALE)
            r_new = res_x(p_new / SCALE)
            if float(r_new @ r_new) < cost:
                x, r, cost = p_new / SCALE, r_new, float(r_new @ r_new)
                lam = max(lam * 0.3, 1e-8)
                improved = True
                break
            lam *= 10.0
        if not improved or np.max(np.abs(delta)) < 1e-6:
            break
    p = clip_params(x * SCALE)
    dof = max(1, len(r) - n_par)
    cov = np.linalg.pinv(A) * (cost / dof) if A is not None else np.zeros((n_par, n_par))
    return unpack(p), cov, it + 1


def fisher_info(vec, measured, theta0=0.35, u_fn=profile_square):
    base = residuals(vec, measured, theta0, u_fn)
    n_par = len(vec)
    J = np.empty((len(base), n_par))
    for k in range(n_par):
        dp = np.zeros(n_par)
        step = 1e-4 * max(1.0, abs(vec[k]))
        dp[k] = step
        J[:, k] = (residuals(vec + dp, measured, theta0, u_fn) - base) / step
    F = J.T @ J
    sign, logdet = np.linalg.slogdet(F)
    d_opt = float(np.exp(logdet / n_par)) if sign > 0 else 0.0
    return F, d_opt, float(np.linalg.cond(F))
