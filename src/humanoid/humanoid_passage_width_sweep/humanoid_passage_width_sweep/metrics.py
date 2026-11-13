# -*- coding: utf-8 -*-
"""指标提取：通过与否、净空、通行时间、路径长度、抖动、单帧规划耗时。纯 numpy。"""

import numpy as np

import humanoid_passage_width_sweep.config as config


def summarize(log, reached, failed, width, variant, label, repeat=0):
    """把一次实验的遥测压成一行指标。"""
    n = len(log.get("t", []))
    if n == 0:
        return dict(variant=variant, label=label, width=width, repeat=repeat,
                    reached=False, failed=True, time=np.nan, min_clear=np.nan,
                    path=np.nan, mean_speed=np.nan, jerk=np.nan, contact=np.nan)
    x, y = log["x"], log["y"]
    steps = np.hypot(np.diff(x), np.diff(y)) if n > 1 else np.array([0.0])
    dt = 1.0 / config.CONTROL_HZ
    vel = steps / dt
    acc = np.diff(vel) if len(vel) > 1 else np.array([0.0])
    return dict(
        variant=variant, label=label, width=float(width), repeat=repeat,
        reached=bool(reached), failed=bool(failed),
        time=float(log["t"][-1]),
        min_clear=float(np.nanmin(log["clearance"])),
        path=float(np.nansum(steps)),
        mean_speed=float(np.nanmean(vel)) if len(vel) else 0.0,
        # 速度一阶差分的均值绝对值，用来量化「走走停停」的抖动
        jerk=float(np.nanmean(np.abs(acc))) if len(acc) else 0.0,
        contact=float(np.nansum(log["contact"])),
    )


def aggregate(rows):
    """按 (variant, width) 聚合多次重复：通过率 + 各项均值。"""
    out = []
    keys = sorted({(r["variant"], r["width"]) for r in rows})
    for variant, width in keys:
        grp = [r for r in rows if r["variant"] == variant and r["width"] == width]
        ok = [r for r in grp if r["reached"]]
        out.append(dict(
            variant=variant, label=grp[0]["label"], width=width,
            runs=len(grp), success=len(ok),
            success_rate=len(ok) / len(grp) if grp else 0.0,
            # 时间/路径只对成功的取均值，失败的不该拉低时间
            time=float(np.mean([r["time"] for r in ok])) if ok else np.nan,
            path=float(np.mean([r["path"] for r in ok])) if ok else np.nan,
            min_clear=float(np.mean([r["min_clear"] for r in grp])),
            mean_speed=float(np.mean([r["mean_speed"] for r in ok])) if ok else np.nan,
            jerk=float(np.mean([r["jerk"] for r in ok])) if ok else np.nan,
        ))
    return out


def write_csv(path, rows):
    """写 CSV（空列表也写空文件）。"""
    import csv
    from pathlib import Path
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return path
    with path.open("w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    return path