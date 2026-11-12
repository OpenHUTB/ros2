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
    ax.set_title("MLP training loss (wall cleaning)")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out)
    plt.close(fig)
    print("loss ->", out)


def compare(json_path, out):
    with open(json_path, encoding="utf-8") as f:
        d = json.load(f)
    keys = ["success_rate", "collision_rate", "wall_coverage"]
    names = ["success", "collision", "wall coverage"]
    ex = [d["expert"][k] * 100 for k in keys]
    nn = [d["nn"][k] * 100 for k in keys]

    fig, ax = plt.subplots(figsize=(6.4, 3.4), dpi=130)
    x = np.arange(len(keys))
    w = 0.36
    ax.bar(x - w / 2, ex, w, label="expert", color="#1f77b4")
    ax.bar(x + w / 2, nn, w, label="NN", color="#d62728")
    for i, (a, b) in enumerate(zip(ex, nn)):
        ax.text(i - w / 2, a + 1.5, "%.0f%%" % a, ha="center", fontsize=8)
        ax.text(i + w / 2, b + 1.5, "%.0f%%" % b, ha="center", fontsize=8)
    ax.set_xticks(x)
    ax.set_xticklabels(names)
    ax.set_ylabel("%")
    ax.set_ylim(0, 115)
    ax.set_title("Expert vs NN (10 identical starts)")
    ax.legend()
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(out)
    plt.close(fig)
    print("compare ->", out)


if __name__ == "__main__":
    pkg = os.path.join(HERE, "..")
    out = os.path.join(pkg, "results")
    os.makedirs(out, exist_ok=True)
    loss_src = os.path.join(pkg, "models", "wall_loss.npy")
    if os.path.exists(loss_src):
        loss_curve(loss_src, os.path.join(out, "loss.png"))
    else:
        print("skip loss.png: missing %s (run train.py first)" % loss_src)
    eval_src = os.path.join(out, "wall_eval.json")
    if os.path.exists(eval_src):
        compare(eval_src, os.path.join(out, "compare.png"))
    else:
        print("skip compare.png: missing %s (run evaluate.py first)" % eval_src)
