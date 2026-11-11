# -*- coding: utf-8 -*-
"""DVS event-camera background-activity suppression and moving-target tracking.

Everything in this file uses only the Python standard library. No numpy,
no carla, no Pillow. ``main.py`` uses this module for offline replay; the
optional live collector and the PNG plotter live in separate files.

Event representation
--------------------
CARLA ``sensor.camera.dvs`` emits a packed stream of 13-byte records

    x : uint16   y : uint16   t : int64   pol : bool

where ``t`` is a microsecond timestamp and ``pol`` is the brightness-change
polarity. For the replay dataset each event is stored as ``[x, y, pol, dt8]``;
``dt8`` is the timestamp quantised into 0..255 inside one sensor tick so that
a time surface can be rebuilt without shipping 64-bit timestamps.
"""

import collections
import json
import math
import random
import struct

DVS_STRUCT = struct.Struct("<HHq?")  # 13 bytes, no padding

DEFAULTS = {
    "cell": 8,                 # background grid cell size, px
    "baseline_frames": 12,     # first N static frames used as training baseline
    "resid_k": 2.0,            # residual = cur - (mu + k * sigma)
    "min_cell_events": 3,      # absolute floor for a cell to be "novel"
    "min_cluster_cells": 3,    # discard small islands of background activity
    "min_cluster_weight": 40.0,  # minimum residual event mass of a hypothesis
    "tau_ms": 30.0,            # time-surface decay constant
    "vel_window": 9,           # frames used for the least-squares velocity fit
}


# --------------------------------------------------------------------------
# raw DVS stream helpers (used by the live collector)
# --------------------------------------------------------------------------
def unpack_raw(raw):
    """Decode the packed CARLA DVS byte stream into (x, y, t, pol) tuples."""
    if len(raw) % DVS_STRUCT.size:
        raise ValueError("DVS raw_data length %d is not a multiple of %d"
                         % (len(raw), DVS_STRUCT.size))
    return list(DVS_STRUCT.iter_unpack(raw))


def quantise(events, stride=2):
    """Convert (x, y, t, pol) to the compact [x, y, pol, dt8] form.

    A *fixed* stride is applied to every frame. Keeping the sampling ratio
    constant is essential: per-cell activity must stay comparable across
    frames, otherwise the learned background statistics are biased.
    """
    stride = max(1, int(stride))
    if not events:
        return [], stride
    t0 = min(e[2] for e in events)
    t1 = max(e[2] for e in events)
    span = max(t1 - t0, 1)
    compact = [
        [int(e[0]), int(e[1]), 1 if e[3] else 0,
         int(round((e[2] - t0) * 255.0 / span))]
        for e in events
    ]
    return compact[::stride], stride


# --------------------------------------------------------------------------
# background activity filter (BAF)
# --------------------------------------------------------------------------
def frame_cells(frame, cfg):
    """Per-cell event counts for one frame."""
    cell = cfg["cell"]
    counts = collections.Counter()
    for x, y, _pol, _dt in frame["e"]:
        counts[(int(x) // cell, int(y) // cell)] += 1
    return counts


def build_baseline(static_frames, cfg):
    """Mean and standard deviation of per-cell activity over static frames."""
    n = max(1, len(static_frames))
    sums = collections.Counter()
    squares = collections.Counter()
    for f in static_frames:
        for key, val in frame_cells(f, cfg).items():
            sums[key] += val
            squares[key] += val * val
    mean, std = {}, {}
    for key in set(sums) | set(squares):
        mu = sums[key] / float(n)
        var = squares[key] / float(n) - mu * mu
        mean[key] = mu
        std[key] = math.sqrt(max(var, 0.0))
    return mean, std


def residual_cells(frame, mean, std, cfg):
    """Cells whose activity exceeds the learned static background."""
    current = frame_cells(frame, cfg)
    kept = {}
    for key, val in current.items():
        mu = mean.get(key, 0.0)
        sd = std.get(key, 0.0)
        threshold = max(cfg["min_cell_events"], mu + cfg["resid_k"] * sd)
        if val > threshold:
            kept[key] = val - mu
    return kept


def connected_components(cells):
    """8-neighbour connected components over a set of grid coordinates."""
    remaining = set(cells)
    out = []
    while remaining:
        seed = remaining.pop()
        stack, comp = [seed], [seed]
        while stack:
            cx, cy = stack.pop()
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    nb = (cx + dx, cy + dy)
                    if nb in remaining:
                        remaining.remove(nb)
                        stack.append(nb)
                        comp.append(nb)
        out.append(comp)
    return out


def detect(frame, mean, std, cfg):
    """Return the dominant moving-object hypothesis, or None."""
    kept = residual_cells(frame, mean, std, cfg)
    if not kept:
        return None
    comps = [c for c in connected_components(kept)
             if len(c) >= cfg["min_cluster_cells"]
             and sum(kept[k] for k in c) >= cfg["min_cluster_weight"]]
    if not comps:
        return None
    comp = max(comps, key=lambda c: sum(kept[k] for k in c))
    cells = set(comp)
    weight = sum(kept[k] for k in comp)
    cell = cfg["cell"]
    u = sum((k[0] + 0.5) * cell * kept[k] for k in comp) / weight
    v = sum((k[1] + 0.5) * cell * kept[k] for k in comp) / weight
    xs = [k[0] for k in comp]
    ys = [k[1] for k in comp]
    box = (min(xs) * cell, min(ys) * cell,
           (max(xs) + 1) * cell, (max(ys) + 1) * cell)
    events = [e for e in frame["e"]
              if (int(e[0]) // cell, int(e[1]) // cell) in cells]
    return {"u": u, "v": v, "box": box, "events": events,
            "cells": cells, "kept": kept}


# --------------------------------------------------------------------------
# time surface and velocity
# --------------------------------------------------------------------------
def time_surface(events, cfg, tick_ms):
    """Exponential recency map, classic event-camera surface of active events."""
    tau = cfg["tau_ms"]
    surf = {}
    for x, y, _pol, dt8 in events:
        t = dt8 / 255.0 * tick_ms
        key = (int(x), int(y))
        val = math.exp(-max(tick_ms - t, 0.0) / tau)
        if key not in surf or val > surf[key]:
            surf[key] = val
    return surf


def linear_slope(points):
    """Least-squares slope of (t, value) pairs."""
    n = len(points)
    if n < 2:
        return 0.0
    mt = sum(p[0] for p in points) / n
    mv = sum(p[1] for p in points) / n
    den = sum((p[0] - mt) ** 2 for p in points)
    if den < 1e-9:
        return 0.0
    return sum((p[0] - mt) * (p[1] - mv) for p in points) / den


# --------------------------------------------------------------------------
# pipeline
# --------------------------------------------------------------------------
def run_pipeline(meta, frames, cfg=None):
    """Replay the dataset and return (records, stats, detections)."""
    cfg = dict(DEFAULTS if cfg is None else cfg)
    static = [f for f in frames if f["phase"] == "static"]
    moving = [f for f in frames if f["phase"] == "moving"]
    n_base = min(cfg["baseline_frames"], len(static))
    baseline_frames = static[:n_base]
    holdout = static[n_base:]
    mean, std = build_baseline(baseline_frames, cfg)

    records, detections = [], {}
    for f in moving:
        det = detect(f, mean, std, cfg)
        detections[f["i"]] = det
        records.append({
            "i": f["i"], "t": f["t"],
            "gt_u": f.get("gt_u"), "gt_v": f.get("gt_v"),
            "u": round(det["u"], 2) if det else None,
            "v": round(det["v"], 2) if det else None,
            "n_events": len(f["e"]),
            "n_kept": len(det["events"]) if det else 0,
            "detected": bool(det),
        })

    false_alarms = sum(1 for f in holdout
                       if detect(f, mean, std, cfg) is not None)
    errs = [math.hypot(r["u"] - r["gt_u"], r["v"] - r["gt_v"])
            for r in records
            if r["detected"] and r["gt_u"] is not None]
    detected = [r for r in records if r["detected"]]

    window = cfg["vel_window"]
    est_pts = [(r["t"], r["u"]) for r in detected[-window:]] if detected else []
    gt_pts = [(r["t"], r["gt_u"]) for r in records
              if r["gt_u"] is not None][-window:]
    vel_est = linear_slope(est_pts)
    vel_gt = linear_slope(gt_pts)

    total_sampled = sum(len(f["e"]) for f in frames)
    total_raw = sum(f.get("n", len(f["e"])) for f in frames)
    total_kept = sum(r["n_kept"] for r in records)
    total_moving = sum(r["n_events"] for r in records)

    stats = {
        "width": meta.get("width"), "height": meta.get("height"),
        "frames": len(frames), "static_frames": len(static),
        "baseline_frames": n_base, "holdout_frames": len(holdout),
        "moving_frames": len(moving),
        "static_noise_per_frame": round(
            sum(len(f["e"]) for f in static) / max(1, len(static)), 1),
        "background_suppression": round(
            1.0 - total_kept / max(1, total_moving), 4),
        "detection_rate": round(len(detected) / max(1, len(records)), 4),
        "centroid_mae_px": round(sum(errs) / len(errs), 2) if errs else None,
        "centroid_max_px": round(max(errs), 2) if errs else None,
        "velocity_est_px_s": round(vel_est, 2),
        "velocity_gt_px_s": round(vel_gt, 2),
        "velocity_rel_error": round(abs(vel_est - vel_gt) / abs(vel_gt), 4)
        if abs(vel_gt) > 1e-6 else None,
        "false_alarms": false_alarms,
        "total_events_raw": total_raw,
        "total_events_sampled": total_sampled,
        "rgb_pixel_volume": meta.get("width", 0) * meta.get("height", 0) * len(frames),
        "event_sparsity_ratio": round(
            total_raw / max(1, meta.get("width", 0) * meta.get("height", 0) * len(frames)), 5),
        "cfg": cfg,
    }
    return records, stats, detections


# --------------------------------------------------------------------------
# I/O
# --------------------------------------------------------------------------
def load_dataset(path):
    with open(path, "r", encoding="utf-8") as fh:
        data = json.load(fh)
    return data["meta"], data["frames"]


def save_csv(records, path):
    cols = ["i", "t", "gt_u", "gt_v", "u", "v",
            "n_events", "n_kept", "detected"]
    with open(path, "w", encoding="utf-8", newline="") as fh:
        fh.write(",".join(cols) + "\n")
        for r in records:
            fh.write(",".join("" if r[c] is None else str(r[c]) for c in cols) + "\n")


def save_json(obj, path, compact=False):
    with open(path, "w", encoding="utf-8") as fh:
        if compact:
            json.dump(obj, fh, ensure_ascii=False, separators=(",", ":"))
        else:
            json.dump(obj, fh, ensure_ascii=False, indent=2)


# --------------------------------------------------------------------------
# deterministic synthetic self-test
# --------------------------------------------------------------------------
def synthetic_dataset(width=160, height=90, n_static=16, n_moving=40):
    """Static textured background + a blob crossing the field of view."""
    rng = random.Random(20261110)
    frames = []
    tick = 0.05

    def blob(cx, cy, n, spread):
        out = []
        for _ in range(n):
            x = max(0, min(width - 1, int(round(rng.gauss(cx, spread)))))
            y = max(0, min(height - 1, int(round(rng.gauss(cy, spread)))))
            out.append([x, y, rng.randint(0, 1), rng.randint(0, 255)])
        return out

    for i in range(n_static + n_moving):
        phase = "static" if i < n_static else "moving"
        e = []
        # two static textured regions: the "background activity"
        e += blob(30, 20, rng.randint(30, 40), 3.0)
        e += blob(125, 22, rng.randint(28, 38), 3.0)
        # a few isolated noise events
        e += [[rng.randrange(width), rng.randrange(height),
               rng.randint(0, 1), rng.randint(0, 255)] for _ in range(4)]
        gt_u = gt_v = None
        if phase == "moving":
            k = i - n_static
            cx = 30.0 + k * 2.4
            e += blob(cx, 62.0, 60, 4.0)
            gt_u, gt_v = cx, 62.0
        frames.append({
            "i": i, "t": round(i * tick, 3), "phase": phase,
            "gt_u": gt_u, "gt_v": gt_v,
            "n": len(e), "keep": 1, "e": e,
        })
    meta = {"width": width, "height": height, "sensor_tick": tick,
            "source": "synthetic", "fov_deg": 90.0}
    return meta, frames


def self_test():
    """Assert the BAF removes background activity and tracks the moving blob."""
    meta, frames = synthetic_dataset()
    cfg = dict(DEFAULTS)
    cfg["baseline_frames"] = 12
    records, stats, _ = run_pipeline(meta, frames, cfg)

    assert DVS_STRUCT.size == 13, "DVS record stride must be 13 bytes"
    raw = DVS_STRUCT.pack(12, 34, 123456789, True) + \
        DVS_STRUCT.pack(56, 78, 123456790, False)
    assert unpack_raw(raw) == [(12, 34, 123456789, True),
                               (56, 78, 123456790, False)]

    assert stats["false_alarms"] == 0, "background produced false detections"
    assert stats["detection_rate"] >= 0.90, "moving blob was not tracked"
    assert stats["centroid_mae_px"] is not None
    assert stats["centroid_mae_px"] < 8.0, "centroid error too large"
    assert stats["background_suppression"] > 0.50, "background not suppressed"
    assert stats["velocity_est_px_s"] > 0.0, "velocity sign/direction wrong"
    assert sum(1 for r in records if not r["detected"]) <= 4
    return stats
