from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def num(row, key, default=float("nan")):
    try:
        return float(row[key])
    except Exception:
        return default


def valid(values):
    return [v for v in values if math.isfinite(v)]


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--csv", required=True)
    p.add_argument("--outdir", required=True)
    args = p.parse_args()

    csv_path = Path(args.csv)
    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    with csv_path.open("r", encoding="utf-8-sig", newline="") as f:
        rows = list(csv.DictReader(f))

    if not rows:
        raise RuntimeError(f"No trajectory rows in {csv_path}")

    t = [num(r, "time", float(i)) for i, r in enumerate(rows)]
    ax = [num(r, "actual_x") for r in rows]
    ay = [num(r, "actual_y") for r in rows]
    tx = [num(r, "target_x") for r in rows]
    ty = [num(r, "target_y") for r in rows]

    error = []
    for r, x, y, gx, gy in zip(rows, ax, ay, tx, ty):
        e = num(r, "error")
        if not math.isfinite(e) and all(math.isfinite(v) for v in (x, y, gx, gy)):
            e = math.hypot(x - gx, y - gy)
        error.append(e)

    ev = valid(error)
    if not ev:
        raise RuntimeError("No valid tracking-error samples found.")

    stats = {
        "sample_count": len(rows),
        "duration_s": (t[-1] - t[0]) if len(t) > 1 else 0.0,
        "mae_m": sum(abs(x) for x in ev) / len(ev),
        "rmse_m": math.sqrt(sum(x * x for x in ev) / len(ev)),
        "max_error_m": max(ev),
        "final_error_m": error[-1],
        "actual_x_min_m": min(valid(ax)),
        "actual_x_max_m": max(valid(ax)),
        "actual_y_min_m": min(valid(ay)),
        "actual_y_max_m": max(valid(ay)),
        "target_x_min_m": min(valid(tx)),
        "target_x_max_m": max(valid(tx)),
        "target_y_min_m": min(valid(ty)),
        "target_y_max_m": max(valid(ty)),
    }

    stats["actual_x_span_m"] = stats["actual_x_max_m"] - stats["actual_x_min_m"]
    stats["actual_y_span_m"] = stats["actual_y_max_m"] - stats["actual_y_min_m"]
    stats["target_x_span_m"] = stats["target_x_max_m"] - stats["target_x_min_m"]
    stats["target_y_span_m"] = stats["target_y_max_m"] - stats["target_y_min_m"]

    plt.figure(figsize=(8, 6))
    plt.plot(tx, ty, label="Target trajectory")
    plt.plot(ax, ay, label="Actual trajectory")
    plt.scatter([ax[0]], [ay[0]], marker="o", label="Start")
    plt.scatter([ax[-1]], [ay[-1]], marker="x", label="End")
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.title("Figure-eight trajectory: target vs actual")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(outdir / "trajectory_xy.png", dpi=180)
    plt.close()

    plt.figure(figsize=(8, 4.5))
    plt.plot(t, error, label="Tracking error")
    plt.xlabel("Time (s)")
    plt.ylabel("Error (m)")
    plt.title("Tracking error over time")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(outdir / "tracking_error.png", dpi=180)
    plt.close()

    # Nearest-path tracking error:
    # distance from each actual point to the nearest segment of the
    # entire target trajectory polyline. This avoids waypoint-switch jumps.
    def point_to_segment_distance(px, py, ax0, ay0, bx0, by0):
        abx = bx0 - ax0
        aby = by0 - ay0
        ab2 = abx * abx + aby * aby

        if ab2 <= 1e-12:
            return math.hypot(px - ax0, py - ay0)

        apx = px - ax0
        apy = py - ay0
        u = (apx * abx + apy * aby) / ab2
        u = max(0.0, min(1.0, u))

        qx = ax0 + u * abx
        qy = ay0 + u * aby
        return math.hypot(px - qx, py - qy)

    target_path = []
    for x, y in zip(tx, ty):
        if math.isfinite(x) and math.isfinite(y):
            if not target_path or (x, y) != target_path[-1]:
                target_path.append((x, y))

    path_error = []
    for x, y in zip(ax, ay):
        if not (math.isfinite(x) and math.isfinite(y)) or len(target_path) < 2:
            path_error.append(float("nan"))
            continue

        best = float("inf")
        for j in range(len(target_path) - 1):
            ax0, ay0 = target_path[j]
            bx0, by0 = target_path[j + 1]
            d = point_to_segment_distance(
                x, y,
                ax0, ay0,
                bx0, by0,
            )
            if d < best:
                best = d

        path_error.append(best)

    plt.figure(figsize=(8, 4.5))
    plt.plot(t, path_error, label="Path tracking error")
    plt.xlabel("Time (s)")
    plt.ylabel("Error (m)")
    plt.title("Path tracking error over time")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(outdir / "path_tracking_error.png", dpi=180)
    plt.close()

    (outdir / "trajectory_stats.json").write_text(
        json.dumps(stats, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )

    report = f"""OpenHUTB Figure-eight Trajectory Report
========================================
Samples:       {stats['sample_count']}
Duration:      {stats['duration_s']:.3f} s
MAE:           {stats['mae_m']:.3f} m
RMSE:          {stats['rmse_m']:.3f} m
Max error:     {stats['max_error_m']:.3f} m
Final error:   {stats['final_error_m']:.3f} m
Actual X span: {stats['actual_x_span_m']:.3f} m
Actual Y span: {stats['actual_y_span_m']:.3f} m
Target X span: {stats['target_x_span_m']:.3f} m
Target Y span: {stats['target_y_span_m']:.3f} m
"""
    (outdir / "trajectory_report.txt").write_text(report, encoding="utf-8")

    print(report)
    print(f"Output directory: {outdir}")


if __name__ == "__main__":
    main()
