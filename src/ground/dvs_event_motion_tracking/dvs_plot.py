# -*- coding: utf-8 -*-
"""Pillow-based running-effect figure for the DVS event-camera module.

Optional component: ``main.py --no-plot`` skips this file entirely, so the
whole offline pipeline stays runnable with the standard library alone.
"""

import os

from dvs_core import time_surface

BG = (14, 16, 20)
PANEL = (22, 25, 31)
FG = (225, 230, 236)
MUTED = (130, 140, 152)
BLUE = (80, 140, 255)
RED = (255, 90, 90)
GREEN = (80, 220, 140)
YELLOW = (255, 210, 80)
ORANGE = (255, 160, 70)


def _font(size):
    from PIL import ImageFont
    for path in ("C:/Windows/Fonts/consola.ttf",
                 "C:/Windows/Fonts/arial.ttf",
                 "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"):
        if os.path.exists(path):
            try:
                return ImageFont.truetype(path, size)
            except Exception:
                pass
    return ImageFont.load_default()


def _event_raster(events, width, height):
    from PIL import Image
    im = Image.new("RGB", (width, height), (0, 0, 0))
    px = im.load()
    for x, y, pol, _dt in events:
        if 0 <= x < width and 0 <= y < height:
            px[x, y] = BLUE if pol else RED
    return im


def _surface_raster(surface, width, height):
    from PIL import Image
    im = Image.new("RGB", (width, height), (0, 0, 0))
    px = im.load()
    for (x, y), val in surface.items():
        if 0 <= x < width and 0 <= y < height:
            v = max(0.0, min(1.0, val))
            px[x, y] = (int(20 + 90 * v), int(30 + 190 * v), int(60 + 195 * v))
    return im


def plot_analysis(meta, frames, records, stats, detections, out_path, cfg):
    """Render the five-panel summary figure and save it as PNG."""
    from PIL import Image, ImageDraw
    # Pillow < 9.1 没有 Image.Resampling，回退到旧版 Image.NEAREST 常量
    _Resampling = getattr(Image, "Resampling", Image)

    W, H = meta.get("width", 320), meta.get("height", 180)
    tick_ms = meta.get("sensor_tick", 0.05) * 1000.0
    f_big, f_mid, f_small = _font(22), _font(15), _font(12)
    canvas = Image.new("RGB", (1180, 1010), BG)
    d = ImageDraw.Draw(canvas)

    d.text((30, 18), "DVS event camera  |  background activity filter + motion tracking",
           fill=FG, font=f_big)

    panels = {
        "A": (30, 62, 540, 330),
        "B": (610, 62, 540, 330),
        "C": (30, 420, 540, 330),
        "D": (610, 420, 540, 330),
        "E": (30, 780, 1120, 190),
    }
    titles = {
        "A": "A  raw DVS events (one 50 ms tick)",
        "B": "B  after background suppression + detection",
        "C": "C  time surface of the tracked events",
        "D": "D  events per frame: raw vs. retained",
        "E": "E  centroid u trajectory vs. simulator ground truth",
    }
    for key, (x0, y0, w, h) in panels.items():
        d.rounded_rectangle([x0, y0, x0 + w, y0 + h], radius=8, fill=PANEL)
        d.text((x0 + 12, y0 + 8), titles[key], fill=FG, font=f_mid)

    # pick a representative moving frame with a detection
    chosen = None
    for r in records:
        if r["detected"] and r["i"] >= stats["static_frames"] + 5:
            chosen = r
            break
    if chosen is None:
        chosen = next((r for r in records if r["detected"]), records[0])
    frame = next(f for f in frames if f["i"] == chosen["i"])
    det = detections.get(chosen["i"])

    def paste(key, im, inset=34):
        x0, y0, w, h = panels[key]
        tw, th = w - 2 * inset, h - inset - 22
        im = im.resize((tw, th), _Resampling.NEAREST)
        canvas.paste(im, (x0 + inset, y0 + inset))

    paste("A", _event_raster(frame["e"], W, H))

    if det:
        raster = _event_raster(det["events"], W, H)
        rd = ImageDraw.Draw(raster)
        x1, y1, x2, y2 = det["box"]
        rd.rectangle([x1, y1, x2, y2], outline=GREEN, width=1)
        u, v = det["u"], det["v"]
        rd.line([(u - 5, v), (u + 5, v)], fill=GREEN, width=1)
        rd.line([(u, v - 5), (u, v + 5)], fill=GREEN, width=1)
        if chosen["gt_u"] is not None:
            gu, gv = chosen["gt_u"], chosen["gt_v"]
            rd.line([(gu - 4, gv - 4), (gu + 4, gv + 4)], fill=YELLOW, width=1)
            rd.line([(gu - 4, gv + 4), (gu + 4, gv - 4)], fill=YELLOW, width=1)
        paste("B", raster)
        paste("C", _surface_raster(time_surface(det["events"], cfg, tick_ms), W, H))

    # panel D: per-frame event bars
    x0, y0, w, h = panels["D"]
    gx0, gy0, gw, gh = x0 + 45, y0 + 40, w - 70, h - 75
    d.rectangle([gx0, gy0, gx0 + gw, gy0 + gh], outline=(60, 66, 76))
    all_n = [f.get("n", len(f["e"])) for f in frames]
    top = max(all_n) or 1
    bw = max(2, gw // max(1, len(frames)))
    for k, f in enumerate(frames):
        x = gx0 + k * bw
        bh = int(all_n[k] / top * gh)
        d.rectangle([x, gy0 + gh - bh, x + bw - 1, gy0 + gh],
                    fill=(90, 100, 118) if f["phase"] == "static" else (70, 120, 200))
    for r in records:
        k = r["i"]
        x = gx0 + k * bw
        keep = frames[k].get("keep", 1)
        bh = int(r["n_kept"] * keep / top * gh)
        d.rectangle([x, gy0 + gh - bh, x + max(1, bw - 2), gy0 + gh], fill=ORANGE)
    d.text((gx0, gy0 + gh + 6), "frame index", fill=MUTED, font=f_small)
    d.text((x0 + 12, y0 + h - 20),
           "gray = static  blue = moving raw  orange = retained after BAF",
           fill=MUTED, font=f_small)

    # panel E: centroid trajectory
    x0, y0, w, h = panels["E"]
    gx0, gy0, gw, gh = x0 + 55, y0 + 38, w - 360, h - 62
    d.rectangle([gx0, gy0, gx0 + gw, gy0 + gh], outline=(60, 66, 76))

    def px(k):
        return gx0 + k / max(1, len(frames) - 1) * gw

    def py(u):
        return gy0 + gh - (u / W) * gh

    gt_pts = [(px(r["i"]), py(r["gt_u"])) for r in records
              if r["gt_u"] is not None]
    est_pts = [(px(r["i"]), py(r["u"])) for r in records if r["detected"]]
    if len(est_pts) > 1:
        d.line(est_pts, fill=GREEN, width=2)
    for p in est_pts:
        d.ellipse([p[0] - 2, p[1] - 2, p[0] + 2, p[1] + 2], fill=GREEN)
    # yellow crosses mark simulator ground truth; they stay visible even
    # when the estimate (green) almost coincides with it
    for gx, gy in gt_pts:
        d.line([(gx - 3, gy - 3), (gx + 3, gy + 3)], fill=YELLOW, width=1)
        d.line([(gx - 3, gy + 3), (gx + 3, gy - 3)], fill=YELLOW, width=1)
    d.text((gx0, gy0 + gh + 6), "frame index", fill=MUTED, font=f_small)
    d.text((x0 + 12, gy0 - 4), "u / px", fill=MUTED, font=f_small)

    tx = gx0 + gw + 22
    summary = [
        "yellow = ground truth",
        "green  = BAF estimate",
        "",
        "detection rate   %.1f%%" % (stats["detection_rate"] * 100),
        "centroid MAE     %s px" % stats["centroid_mae_px"],
        "background cut   %.1f%%" % (stats["background_suppression"] * 100),
        "false alarms     %d" % stats["false_alarms"],
        "velocity est/gt  %.1f / %.1f px/s" %
        (stats["velocity_est_px_s"], stats["velocity_gt_px_s"]),
        "event sparsity   %.3f%%" % (stats["event_sparsity_ratio"] * 100),
    ]
    for i, line in enumerate(summary):
        d.text((tx, y0 + 40 + i * 17), line,
               fill=(YELLOW if i == 0 else GREEN if i == 1 else FG), font=f_small)

    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    canvas.save(out_path)
    return out_path
