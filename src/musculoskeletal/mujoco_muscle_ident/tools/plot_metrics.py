#!/usr/bin/env python3
"""从 runs 生成五张图：I1 误差、I2 噪声、I3 相关、I4 收敛域、I5 激励。"""
import os
import sys
import json  # 修复：缺失json导入

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from mujoco_muscle_ident import core_mj as C
from mujoco_muscle_ident import runs

# ========== 修复 MOD 未定义：取项目根目录（tools的上一级） ==========
SCRIPT_PATH = os.path.abspath(__file__)
# tools/plot_metrics.py → 上一层就是项目根目录
MOD = os.path.dirname(os.path.dirname(SCRIPT_PATH))

OUT = os.path.join(MOD, "out")
os.makedirs(OUT, exist_ok=True)
print("跑 I1..I5 ...")
i1 = runs.run_i1()
i2 = runs.run_i2()
i3 = runs.run_i3()
i4 = runs.run_i4()
i5 = runs.run_i5()
data = {"i1": i1, "i2": i2, "i3": i3, "i4": i4, "i5": i5}
json.dump(data, open(os.path.join(OUT, "metrics.json"), "w", encoding="utf-8"),
          ensure_ascii=False, indent=2)
# ---------- 图 ----------
names = [p["name"] for p in i1["params"]]
errs = [p["err_pct"] for p in i1["params"]]
fig, ax = plt.subplots(figsize=(5.6, 3.0))
ax.bar(names, errs, color="#4c78a8")
ax.set_ylabel("relative error (%)")
ax.set_title("I1 identification error (MuJoCo joint model)")
for k, v in enumerate(errs):
    ax.annotate(f"{v:.2f}%", (k, v), textcoords="offset points", xytext=(0, 3),
                ha="center", fontsize=8)
ax.grid(alpha=0.3, axis="y")
fig.tight_layout()
fig.savefig(os.path.join(OUT, "i1_param_error.png"), dpi=130)
plt.close(fig)
nz = [lv["noise_pct"] for lv in i2["levels"]]
md = [lv["mean_err_pct"] for lv in i2["levels"]]
mx = [lv["max_err_pct"] for lv in i2["levels"]]
fig, ax = plt.subplots(figsize=(5.6, 3.0))
ax.plot(nz, md, "o-", label="median error", color="#4c78a8")
ax.plot(nz, mx, "s--", label="max error", color="#e45756")
ax.set_xlabel("measurement noise (%)")
ax.set_ylabel("parameter error (%)")
ax.set_title("I2 identification error vs noise (3 seeds, median)")
ax.legend()
ax.grid(alpha=0.3)
fig.tight_layout()
fig.savefig(os.path.join(OUT, "i2_noise.png"), dpi=130)
plt.close(fig)
corr = np.asarray(i3["matrix"])
labels = i3["names"]
fig, ax = plt.subplots(figsize=(4.8, 4.0))
im = ax.imshow(corr, cmap="RdBu_r", vmin=-1, vmax=1)
ax.set_xticks(range(len(labels)))
ax.set_xticklabels(labels, fontsize=8)
ax.set_yticks(range(len(labels)))
ax.set_yticklabels(labels, fontsize=8)
for i in range(len(labels)):
    for j in range(len(labels)):
        ax.text(j, i, f"{corr[i, j]:.2f}", ha="center", va="center", fontsize=7)
ax.set_title("I3 parameter correlation (Fisher information)")
fig.colorbar(im, ax=ax, shrink=0.85)
fig.tight_layout()
fig.savefig(os.path.join(OUT, "i3_corr.png"), dpi=130)
plt.close(fig)
lab = [f"{lv['range_lo']:.1f}~{lv['range_hi']:.1f}x" for lv in i4["levels"]]
sr = [lv["success_rate_pct"] for lv in i4["levels"]]
fig, ax = plt.subplots(figsize=(5.8, 3.0))
ax.bar(lab, sr, color="#54a24b")
ax.set_ylabel("convergence success rate (%)")
ax.set_xlabel("initial guess range (x true value)")
ax.set_title("I4 convergence basin of the LM fit")
ax.set_ylim(0, 110)
for k, v in enumerate(sr):
    ax.annotate(f"{v:.0f}%", (k, v), textcoords="offset points", xytext=(0, 3),
                ha="center", fontsize=8)
ax.grid(alpha=0.3, axis="y")
fig.tight_layout()
fig.savefig(os.path.join(OUT, "i4_basin.png"), dpi=130)
plt.close(fig)
ex = [lv["excitation"] for lv in i5["levels"]]
dopt = [lv["d_optimal"] for lv in i5["levels"]]
e5 = [lv["mean_err_pct"] for lv in i5["levels"]]
fig, ax = plt.subplots(figsize=(6.0, 3.2))
ax.bar(ex, [x / 1000 for x in dopt], color="#4c78a8", label="D-optimality (x1e3)")
ax2 = ax.twinx()
ax2.plot(ex, e5, "o--", color="#e45756", label="identification error")
ax.set_ylabel("D-optimality  det(JtJ)^(1/p)  [x1e3]")
ax2.set_ylabel("mean error (%)")
ax.set_title("I5 excitation design: D‑optimality predicts accuracy")
ax.grid(alpha=0.3, axis="y")
fig.tight_layout()
fig.savefig(os.path.join(OUT, "i5_excitation.png"), dpi=130)
plt.close(fig)
print("已输出 5 张图到 out/")
