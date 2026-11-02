# -*- coding: utf-8 -*-
"""
render_gif.py
部署训练好的 MLP, 记录机械臂跟踪过程, 用 matplotlib 渲染为 gif 动图。
(无 DISPLAY 环境, 使用 Agg 后端 + pillow 写入器)
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from reacher_env import ReacherEnv, forward_kinematics
from nn import MLP


def make_policy(net, Xstat, Ystat):
    Xmean, Xstd = Xstat[0], Xstat[1]
    Ymean, Ystd = Ystat[0], Ystat[1]

    def policy(s):
        xs = (np.asarray(s, dtype=np.float64) - Xmean) / Xstd
        ys = net.forward(xs.reshape(1, -1))[0]
        return ys * Ystd + Ymean

    return policy


def render_one(env, policy, target, out_name, max_steps=150, every=2):
    q1, q2 = env.q()
    qtraj = [(q1, q2)]
    dtraj = []
    for _ in range(max_steps):
        u = policy(env.state())
        env.step(u)
        qtraj.append(env.q())
        dtraj.append(env.distance())
        if env.distance() < 0.02:
            break

    qtraj = qtraj[::every]
    tx, ty = env.target_xy()
    L1, L2 = 0.15, 0.12

    fig, ax = plt.subplots(figsize=(5, 5))
    ax.set_xlim(-0.35, 0.35)
    ax.set_ylim(-0.35, 0.35)
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.set_title("MLP neural-network target tracking")
    target_dot, = ax.plot([tx], [ty], "ro", ms=9, label="target")
    line, = ax.plot([], [], "o-", lw=3, ms=4, color="tab:blue", label="arm")
    trail, = ax.plot([], [], "--", lw=1, color="tab:orange", alpha=0.7, label="end-effector")
    info = ax.text(0.02, 0.95, "", transform=ax.transAxes)
    ax.legend(loc="upper right", fontsize=8)

    tip_traj = []

    def init():
        line.set_data([], [])
        trail.set_data([], [])
        return line, trail

    def update(i):
        q1, q2 = qtraj[i]
        x1, y1 = L1 * np.cos(q1), L1 * np.sin(q1)
        x2, y2 = forward_kinematics(q1, q2)
        line.set_data([0, x1, x2], [0, y1, y2])
        tip_traj.append((x2, y2))
        tt = np.array(tip_traj)
        trail.set_data(tt[:, 0], tt[:, 1])
        info.set_text("step=%d  err=%.3f" % (i, dtraj[min(i * every, len(dtraj) - 1)]))
        return line, trail, info

    anim = FuncAnimation(fig, update, frames=len(qtraj), init_func=init, blit=False)
    anim.save(out_name, writer="pillow", fps=20, dpi=80)
    plt.close(fig)
    print("saved", out_name, "frames=", len(qtraj))


def main(n_eps=2, max_steps=150, seed=1):
    np.random.seed(seed)
    env = ReacherEnv("models/reacher.xml")
    net = MLP([6, 128, 128, 2])
    net.load("mlp.npz")
    Xstat = np.load("Xstat.npy")
    Ystat = np.load("Ystat.npy")
    policy = make_policy(net, Xstat, Ystat)

    for ep in range(n_eps):
        q1 = np.random.uniform(-1.5, 1.5)
        q2 = np.random.uniform(-1.5, 1.5)
        target = env.reset(init_q=(q1, q2))
        render_one(env, policy, target, f"reacher_ep{ep}.gif", max_steps)


if __name__ == "__main__":
    main()
