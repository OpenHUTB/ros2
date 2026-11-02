# -*- coding: utf-8 -*-
"""
deploy_eval.py
部署评估: 加载训练好的 MLP, 在 MuJoCo 中形成
"感知(关节编码器+目标检测) -> 神经网络 -> 扭矩控制" 的闭环,
评估多回合末端跟踪误差, 并对比逆运动学+PD 控制。
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from reacher_env import ReacherEnv
from nn import MLP


def make_policy(net, Xstat, Ystat):
    Xmean, Xstd = Xstat[0], Xstat[1]
    Ymean, Ystd = Ystat[0], Ystat[1]

    def policy(s):
        xs = (np.asarray(s, dtype=np.float64) - Xmean) / Xstd
        ys = net.forward(xs.reshape(1, -1))[0]
        return ys * Ystd + Ymean

    return policy


def run_episode(env, policy, max_steps=200, use_pd=False):
    q1 = np.random.uniform(-1.5, 1.5)
    q2 = np.random.uniform(-1.5, 1.5)
    target = env.reset(init_q=(q1, q2))
    curve = []
    for _ in range(max_steps):
        if use_pd:
            u = env.ik_pd_ctrl(target)
        else:
            u = policy(env.state())
        env.step(u)
        curve.append(env.distance())
        if env.distance() < 0.02:
            break
    return curve


def main(n_eps=30, max_steps=200, seed=0):
    np.random.seed(seed)
    env = ReacherEnv("models/reacher.xml")
    net = MLP([6, 128, 128, 2])
    net.load("mlp.npz")
    Xstat = np.load("Xstat.npy")
    Ystat = np.load("Ystat.npy")
    policy = make_policy(net, Xstat, Ystat)

    mlp_curves = [run_episode(env, policy, max_steps) for _ in range(n_eps)]
    pd_curves = [run_episode(env, None, max_steps, use_pd=True) for _ in range(n_eps)]

    def stats(curves):
        fds = np.array([c[-1] for c in curves])
        steps = np.array([len(c) for c in curves])
        return fds, steps

    mlp_fd, mlp_st = stats(mlp_curves)
    pd_fd, pd_st = stats(pd_curves)
    print("=" * 50)
    print("MLP 闭环: final_dist mean=%.4f median=%.4f success(<0.05)=%.2f%%  mean_steps=%.1f"
          % (mlp_fd.mean(), np.median(mlp_fd), (mlp_fd < 0.05).mean() * 100, mlp_st.mean()))
    print("IK+PD:   final_dist mean=%.4f median=%.4f success(<0.05)=%.2f%%  mean_steps=%.1f"
          % (pd_fd.mean(), np.median(pd_fd), (pd_fd < 0.05).mean() * 100, pd_st.mean()))

    # 误差收敛曲线(前 3 条)
    plt.figure(figsize=(7, 5))
    for c in mlp_curves[:3]:
        plt.plot(np.arange(len(c)), c, alpha=0.8)
    plt.xlabel("control step")
    plt.ylabel("end-effector error (m)")
    plt.title("MLP closed-loop tracking error")
    plt.grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig("tracking_error_curve.png", dpi=120)
    print("saved: tracking_error_curve.png")

    # 统计表
    with open("eval_summary.csv", "w") as f:
        f.write("method,mean_err,median_err,success_rate,mean_steps\n")
        f.write("MLP,%.4f,%.4f,%.2f,%.1f\n" % (mlp_fd.mean(), np.median(mlp_fd),
                                                (mlp_fd < 0.05).mean() * 100, mlp_st.mean()))
        f.write("IKPD,%.4f,%.4f,%.2f,%.1f\n" % (pd_fd.mean(), np.median(pd_fd),
                                                (pd_fd < 0.05).mean() * 100, pd_st.mean()))
    print("saved: eval_summary.csv")


if __name__ == "__main__":
    main()
