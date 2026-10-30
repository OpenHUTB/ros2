#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把自检数据画成曲线图（论文/文档用）。

两张子图：
    左：PD 与 MLP 在同一工况下的倾角曲线（看"学生"是否追上了"老师"）
    右：外力扰动后的恢复过程（标注最大倾角与回正时刻）

依赖 matplotlib（可选）：核心仿真只要有 mujoco + numpy。
    pip3 install matplotlib
    python3 tools/plot_curves.py
"""

import argparse
import math
import sys
from pathlib import Path

PKG_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PKG_ROOT / "src"))

from balance_bot import config                                   # noqa: E402
from balance_bot.control import BalanceController                # noqa: E402
from balance_bot.env import BalanceBotEnv                        # noqa: E402
from balance_bot.policy_nn import PolicyMLP, StandardScaler, state_features   # noqa: E402


def run_pd(env, seconds, v_target=0.0, tilt=0.02):
    ctrl = BalanceController()
    env.reset(tilt=tilt)
    ts, th = [], []
    for _ in range(int(seconds * config.CONTROL_HZ)):
        st = env.state()
        tl, tr = ctrl(st, v_target, 0.0)
        env.set_torque(tl, tr)
        st = env.step()
        ts.append(st.t)
        th.append(math.degrees(st.theta))
        if not env.is_upright():
            break
    return ts, th


def run_nn(env, net, scaler, seconds, v_target=0.0, tilt=0.02):
    env.reset(tilt=tilt)
    ts, th = [], []
    for _ in range(int(seconds * config.CONTROL_HZ)):
        st = env.state()
        tau = float(net.predict(scaler.transform(
            state_features(st, v_target).reshape(1, -1)))[0, 0])
        tau = max(-config.TORQUE_MAX, min(config.TORQUE_MAX, tau))
        env.set_torque(tau, tau)
        st = env.step()
        ts.append(st.t)
        th.append(math.degrees(st.theta))
        if not env.is_upright():
            break
    return ts, th


def run_disturb(env, seconds=8.0, push=2.0, at=3.0):
    ctrl = BalanceController()
    env.reset(tilt=0.0)
    ts, th = [], []
    peak_t, peak_v, recover = None, 0.0, None
    for k in range(int(seconds * config.CONTROL_HZ)):
        if k == int(at * config.CONTROL_HZ):
            env.data.qvel[0] += push / 6.9
        st = env.state()
        tl, tr = ctrl(st, 0.0, 0.0)
        env.set_torque(tl, tr)
        st = env.step()
        ts.append(st.t)
        th.append(math.degrees(st.theta))
        if st.t > at and abs(st.theta) > peak_v:
            peak_v, peak_t = abs(st.theta), st.t
        if st.t > at + 0.2 and abs(st.theta) < math.radians(2.0) and recover is None:
            recover = st.t
    return ts, th, peak_t, math.degrees(peak_v), recover


def main(argv=None):
    ap = argparse.ArgumentParser(description="画平衡控制曲线")
    ap.add_argument("--out", default=str(PKG_ROOT / "docs_gif" / "balance_curves.png"))
    args = ap.parse_args(argv)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    env = BalanceBotEnv(config.resolve_scene_path(None), verbose=False)
    model = config.DEFAULT_OUTPUT_DIR / "balance_policy.npz"
    sc = config.DEFAULT_OUTPUT_DIR / "balance_scaler.npz"
    if not model.exists():
        print("[错误] 找不到 %s，先跑 tools/train_policy.py" % model)
        return 1
    net = PolicyMLP.load(str(model))
    scaler = StandardScaler.load(str(sc))

    t1, pd_theta = run_pd(env, 10.0, 0.30)
    t2, nn_theta = run_nn(env, net, scaler, 10.0, 0.30)
    t3, d_theta, peak_t, peak_v, recover = run_disturb(env)

    fig, ax = plt.subplots(1, 2, figsize=(11, 3.6), dpi=130)

    ax[0].plot(t1, pd_theta, lw=1.6, label="倾角 PD（老师）")
    ax[0].plot(t2, nn_theta, lw=1.6, ls="--", label="MLP（学生）")
    ax[0].axhline(0, color="0.7", lw=0.8)
    ax[0].set_xlabel("时间 / s")
    ax[0].set_ylabel("倾角 θ / °")
    ax[0].set_title("前进 0.30 m/s 时的倾角")
    ax[0].legend(fontsize=9)
    ax[0].grid(alpha=0.3)

    ax[1].plot(t3, d_theta, lw=1.6, color="tab:red")
    ax[1].axvline(3.0, color="0.5", ls=":", lw=1)
    ax[1].annotate("施加 2 N·s 冲量", xy=(3.0, 0), xytext=(3.3, max(d_theta) * 0.8 if max(d_theta) > 0 else -1.0),
                   fontsize=8, color="0.4")
    if peak_t:
        ax[1].plot([peak_t], [d_theta[int(peak_t * config.CONTROL_HZ)]], "o", ms=4, color="tab:red")
    if recover:
        ax[1].axvline(recover, color="tab:green", ls="--", lw=1)
        ax[1].text(recover + 0.1, min(d_theta) * 0.6, "%.2f s 回正" % recover,
                   fontsize=8, color="tab:green")
    ax[1].axhline(0, color="0.7", lw=0.8)
    ax[1].set_xlabel("时间 / s")
    ax[1].set_ylabel("倾角 θ / °")
    ax[1].set_title("外力扰动下的恢复（PD 控制）")
    ax[1].grid(alpha=0.3)

    fig.tight_layout()
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out)
    print("[图] %s" % out)
    print("    左图：PD 最大 %.2f°，MLP 最大 %.2f°"
          % (max(abs(v) for v in pd_theta), max(abs(v) for v in nn_theta)))
    print("    右图：峰值 %.2f° @ %.2f s，回正 %.2f s" % (peak_v, peak_t or -1, recover or -1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
