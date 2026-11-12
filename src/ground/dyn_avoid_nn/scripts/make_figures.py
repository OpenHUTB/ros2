"""生成文档配图：训练损失曲线 + 专家/神经网络指标对比。"""

import json
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))


def loss_curve(path, out):
    h = np.load(path)
    fig, ax = plt.subplots(figsize=(6.4, 3.2), dpi=130)
    ax.plot(np.arange(1, len(h) + 1), h, "o-", ms=3, color="#1f77b4")
    ax.set_xlabel("epoch")
    ax.set_ylabel("validation MSE")
    ax.set_title("MLP training loss (dynamic avoidance)")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out)
    plt.close(fig)
    print("loss ->", out)


def compare(json_path, out):
    with open(json_path, encoding="utf-8") as f:
        d = json.load(f)
    fig, axes = plt.subplots(1, 2, figsize=(7.2, 3.2), dpi=130)
    keys = ["success_rate", "collision_rate"]
    names = ["success", "collision"]
    ex = [d["expert"][k] * 100 for k in keys]
    nn = [d["nn"][k] * 100 for k in keys]
    x = np.arange(2)
    w = 0.36
    axes[0].bar(x - w / 2, ex, w, label="expert", color="#1f77b4")
    axes[0].bar(x + w / 2, nn, w, label="NN", color="#d62728")
    for i, (a, b) in enumerate(zip(ex, nn)):
        axes[0].text(i - w / 2, a + 1.5, "%.0f%%" % a, ha="center", fontsize=8)
        axes[0].text(i + w / 2, b + 1.5, "%.0f%%" % b, ha="center", fontsize=8)
    axes[0].set_xticks(x)
    axes[0].set_xticklabels(names)
    axes[0].set_ylim(0, 115)
    axes[0].set_ylabel("%")
    axes[0].set_title("Outcome")
    axes[0].legend(fontsize=8)
    axes[0].grid(axis="y", alpha=0.3)

    ex_steps = d["expert"]["avg_steps"]
    nn_steps = d["nn"]["avg_steps"]
    axes[1].bar(["expert", "NN"], [ex_steps, nn_steps], color=["#1f77b4", "#d62728"], width=0.5)
    for i, v in enumerate([ex_steps, nn_steps]):
        axes[1].text(i, v + 6, "%.0f" % v, ha="center", fontsize=8)
    axes[1].set_ylabel("steps")
    axes[1].set_title("Average completion steps")
    axes[1].grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(out)
    plt.close(fig)
    print("compare ->", out)


if __name__ == "__main__":
    pkg = os.path.join(HERE, "..")
    out = os.path.join(pkg, "results")
    os.makedirs(out, exist_ok=True)
    loss_curve(os.path.join(pkg, "models", "dyn_loss.npy"), os.path.join(out, "loss.png"))
    compare(os.path.join(out, "dyn_eval.json"), os.path.join(out, "compare.png"))
