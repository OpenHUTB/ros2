#!/usr/bin/env python3
"""推力分配结果：各推力器指令与利用率。"""

import argparse
import csv

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default="/tmp/rov_thruster_allocation_ls.csv")
    ap.add_argument("--out", default="/tmp/rov_thruster_allocation_ls.png")
    args = ap.parse_args()

    rows = []
    with open(args.csv, newline="") as f:
        for r in csv.DictReader(f):
            rows.append({k: float(v) for k, v in r.items()})
    if not rows:
        raise SystemExit("empty csv")

    t = [r["t"] for r in rows]
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.2))
    for i in range(8):
        ax1.plot(t, [r["t%d" % i] for r in rows], lw=1.2, label="T%d" % i)
    ax1.set_xlabel("t [s]")
    ax1.set_ylabel("thrust [N]")
    ax1.set_title("thruster commands")
    ax1.grid(alpha=0.3)
    ax1.legend(fontsize=6, ncol=2)

    ax2.plot(t, [r["utilization"] for r in rows], "#d62728", lw=1.6, label="utilization")
    sat = [r["t"] for r in rows if r["saturated"] > 0.5]
    if sat:
        ax2.axvspan(min(sat), max(sat), color="orange", alpha=0.25, label="saturated")
    ax2.axhline(1.0, color="black", ls=":", lw=1)
    ax2.set_xlabel("t [s]")
    ax2.set_ylabel("max |thrust| / limit")
    ax2.set_title("utilization and saturation")
    ax2.grid(alpha=0.3)
    ax2.legend(fontsize=8)

    fig.tight_layout()
    fig.savefig(args.out, dpi=140)
    print("saved %s rows=%d" % (args.out, len(rows)))


if __name__ == "__main__":
    main()
