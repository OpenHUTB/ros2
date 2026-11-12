# -*- coding: utf-8 -*-
"""图表：抗噪曲线、混淆矩阵。标签用英文，避免 matplotlib 缺中文字体渲染成方块。"""

from pathlib import Path


def _plt():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"figure.dpi": 130, "font.size": 9,
                         "axes.spines.top": False, "axes.spines.right": False,
                         "axes.grid": True, "grid.alpha": 0.25, "grid.linestyle": "--"})
    return plt


def plot_snr(curve, path, train_snrs=(None, 20, 10)):
    """SNR - 准确率曲线；训练时见过的档位用实心点，未见过的用空心点。"""
    plt = _plt()
    fig, ax = plt.subplots(figsize=(6.0, 3.0))
    xs, ys, seen = [], [], []
    for snr, acc in curve:
        xs.append(30 if snr is None else snr)      # clean 画在最右侧
        ys.append(acc * 100)
        seen.append(snr in train_snrs)
    order = sorted(range(len(xs)), key=lambda i: -xs[i])
    xs = [xs[i] for i in order]; ys = [ys[i] for i in order]; seen = [seen[i] for i in order]
    ax.plot(xs, ys, "-", color="0.55", lw=1.0, zorder=1)
    ax.scatter([x for x, s in zip(xs, seen) if s], [y for y, s in zip(ys, seen) if s],
               s=42, color="#1f77b4", label="seen in training", zorder=2)
    ax.scatter([x for x, s in zip(xs, seen) if not s], [y for y, s in zip(ys, seen) if not s],
               s=42, facecolors="none", edgecolors="#d95f02", label="unseen (test only)", zorder=2)
    ax.set_xlabel("SNR (dB)   —   rightmost = clean")
    ax.set_ylabel("accuracy (%)")
    ax.set_ylim(-5, 105)
    ax.set_title("Keyword accuracy vs additive white noise")
    ax.legend(frameon=False, fontsize=8, loc="lower left")
    fig.tight_layout()
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path)
    plt.close(fig)
    return path


def plot_confusion(cm, labels, path):
    """混淆矩阵热图（行=真值，列=预测）。"""
    plt = _plt()
    fig, ax = plt.subplots(figsize=(5.6, 4.8))
    im = ax.imshow(cm, cmap="Blues")
    ax.set_xticks(range(len(labels)))
    ax.set_yticks(range(len(labels)))
    ax.set_xticklabels(labels, rotation=60, ha="right", fontsize=7)
    ax.set_yticklabels(labels, fontsize=7)
    for i in range(len(labels)):
        for j in range(len(labels)):
            if cm[i, j]:
                ax.text(j, i, str(cm[i, j]), ha="center", va="center", fontsize=7,
                        color="white" if cm[i, j] > cm.max() * 0.5 else "black")
    ax.set_xlabel("predicted")
    ax.set_ylabel("true")
    ax.set_title("Confusion matrix (clean corpus)")
    ax.grid(False)
    fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
    fig.tight_layout()
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path)
    plt.close(fig)
    return path