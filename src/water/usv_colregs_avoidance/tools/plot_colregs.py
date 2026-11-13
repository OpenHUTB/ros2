"""生成 docs 页用的会遇轨迹对比图（真实仿真数据）。"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from usv_colregs_avoidance.colregs import DEFAULT_THRESHOLDS, ShipState
from usv_colregs_avoidance.simulation import _simulate

own = ShipState(0.0, 0.0, 0.0, 5.0)
cases = [
    ("head_on", ShipState(1000.0, 0.0, 3.141592653589793, 5.0)),
    ("crossing_give_way", ShipState(800.0, 600.0, -1.5707963267948966, 5.0)),
]

fig, axes = plt.subplots(1, 2, figsize=(11, 4.6), dpi=150)
for ax, (name, other) in zip(axes, cases):
    dist_ok, _d, trace_ok, _c = _simulate(own, other, 400.0, True,
                                          DEFAULT_THRESHOLDS)
    dist_no, _d2, trace_no, _c2 = _simulate(own, other, 400.0, False,
                                            DEFAULT_THRESHOLDS)
    ax.plot([t[1] for t in trace_ok], [t[2] for t in trace_ok], "-",
            label="own (COLREGs, d_min=%.0f m)" % dist_ok)
    ax.plot([t[3] for t in trace_ok], [t[4] for t in trace_ok], "--",
            label="target")
    ax.plot([t[1] for t in trace_no], [t[2] for t in trace_no], ":",
            label="own (no action, d_min=%.0f m)" % dist_no)
    ax.set_title(name)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.grid(True, linestyle="--", alpha=0.5)
    ax.axis("equal")
    ax.legend(loc="best", fontsize=8)
fig.suptitle("COLREGs avoidance (closed-loop decision, single maneuver)")
fig.savefig("/mnt/c/Windows/Temp/colregs_demo.png", bbox_inches="tight")
print("saved colregs_demo.png")
