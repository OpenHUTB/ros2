# -*- coding: utf-8 -*-
"""生成本模块的图（需要 matplotlib）。

    python3 tools/make_figures.py --out ../../docs/img/musculoskeletal/muscle_synergy_lab

左图：VAF 随协同数 k 的变化（含相邻增量）；
右图：子空间夹角对 k 不敏感，但对"完全不同的协同结构"有区分力——
用来说明这个指标能做什么、不能做什么。
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib import font_manager  # noqa: E402

from muscle_synergy_lab.core import nmf, recovery_check, scan_k, synth_data  # noqa: E402


def _setup_cjk():
    for name in ("Microsoft YaHei", "SimHei", "Noto Sans CJK SC",
                 "WenQuanYi Zen Hei", "PingFang SC"):
        if any(f.name == name for f in font_manager.fontManager.ttflist):
            plt.rcParams["font.sans-serif"] = [name, "DejaVu Sans"]
            break
    plt.rcParams["axes.unicode_minus"] = False


_setup_cjk()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=".")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    d = synth_data(n_muscles=8, n_synergies=3, n_samples=200, noise=0.03, seed=0)
    k_true = d.W_true.shape[1]
    rows = scan_k(d.E, k_max=8, seed=0)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9.8, 4.0))

    ks = [r["k"] for r in rows]
    vafs = [100 * r["vaf"] for r in rows]
    ax1.plot(ks, vafs, "o-", color="tab:blue")
    ax1.axvline(k_true, color="red", linestyle="--", linewidth=1.2,
                label="真值 k = %d" % k_true)
    ax1.set_xlabel("协同数 k")
    ax1.set_ylabel("重建率 VAF (%)")
    ax1.set_title("重建率在 k=3 处出现断崖，之后增量趋近 0")
    ax1.set_ylim(70, 101)
    ax1.legend(fontsize=9)
    ax1.grid(alpha=0.3)

    kk = [1, 2, 3, 4, 5]
    ang = [recovery_check(d.E, d.W_true, k)["angle_deg"] for k in kk]
    rng = np.random.default_rng(3)
    scrambled = float(np.degrees(np.arccos(np.clip(
        np.linalg.svd(np.linalg.qr(d.W_true)[0].T
                      @ np.linalg.qr(rng.random(d.W_true.shape))[0],
                      compute_uv=False).min(), -1.0, 1.0))))

    ax2.bar(kk, ang, color="tab:green", label="与真值子空间的夹角")
    ax2.bar([7], [scrambled], color="tab:red", width=0.8,
            label="与“打乱结构”的夹角")
    ax2.axhline(0, color="black", linewidth=0.8)
    ax2.set_xticks(kk + [7])
    ax2.set_xticklabels([str(k) for k in kk] + ["打乱"])
    ax2.set_xlabel("协同数 k / 对照组")
    ax2.set_ylabel("最大主角 (°)")
    ax2.set_title("夹角抓不住“少选/多选”，但认得出不同结构")
    ax2.legend(fontsize=9)
    ax2.grid(alpha=0.3, axis="y")

    path = os.path.join(args.out, "vaf_vs_k.png")
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
    print(path)


if __name__ == "__main__":
    main()
