"""Generate analysis plots from collected IMU data.

Uses Pillow (optional dependency) to produce a four-panel PNG:
  1. Raw vertical acceleration (gravity included)
  2. Gravity-removed high-pass residual
  3. Sliding-window energy with detection threshold
  4. GPS track with anomaly markers

The detection core itself has zero third-party dependencies; Pillow is only
needed for this visualisation, so ``main.py --no-plot`` works everywhere.
"""

from __future__ import annotations

import json
import math
import os
from typing import List, Optional, Tuple

from PIL import Image, ImageDraw, ImageFont

from imu_anomaly import (
    DEFAULTS,
    IMUSample,
    AnomalyEvent,
    gravity_align,
    high_pass_residual,
    detect_events,
    _mean,
    _std,
)


# colour palette (light theme, print-friendly)
C_BG = (255, 255, 255)
C_BORDER = (170, 175, 185)
C_TEXT = (40, 45, 55)
C_MUTED = (110, 118, 130)
C_GRID = (226, 230, 236)
C_SIGNAL = (36, 92, 170)
C_SIGNAL2 = (150, 75, 30)
C_THRESH = (205, 60, 60)
C_ZERO = (190, 195, 202)

LABEL_COLORS = {
    "pothole": (215, 50, 50),
    "speed_bump": (35, 150, 70),
    "rough_patch": (215, 135, 25),
}


def _try_font(size: int = 12, bold: bool = False):
    names = (["consolab.ttf", "arialbd.ttf", "DejaVuSans-Bold.ttf"]
             if bold else
             ["consola.ttf", "arial.ttf", "DejaVuSans.ttf"])
    for name in names:
        try:
            return ImageFont.truetype(name, size)
        except Exception:
            continue
    return ImageFont.load_default()


def _map_values(vals: List[float], x0: int, y0: int, w: int, h: int,
                vmin: float, vmax: float,
                n_total: int) -> List[Tuple[int, int]]:
    """Map values to *absolute* canvas coordinates, using the full timeline."""
    rng = (vmax - vmin) if vmax != vmin else 1.0
    pts = []
    for i, v in enumerate(vals):
        x = x0 + int(i * (w - 1) / max(1, n_total - 1))
        y = y0 + h - 1 - int((v - vmin) / rng * (h - 1))
        y = max(y0, min(y0 + h - 1, y))
        pts.append((x, y))
    return pts


def _panel(draw, x0, y0, w, h, title, vals, vmin, vmax,
           n_total, events, t_end, font, font_small,
           line_color=C_SIGNAL, threshold=None, zero_line=False):
    """Draw one bordered signal panel with title, grid and event markers."""
    draw.rectangle([x0, y0, x0 + w, y0 + h], fill=C_BG, outline=C_BORDER, width=1)

    # horizontal grid lines
    for frac in (0.25, 0.5, 0.75):
        gy = y0 + int(h * frac)
        draw.line([(x0 + 1, gy), (x0 + w - 1, gy)], fill=C_GRID)

    # zero line
    if zero_line and vmin < 0 < vmax:
        zy = y0 + h - 1 - int((0 - vmin) / (vmax - vmin) * (h - 1))
        draw.line([(x0 + 1, zy), (x0 + w - 1, zy)], fill=C_ZERO)

    # threshold line
    if threshold is not None and vmin <= threshold <= vmax:
        ty = y0 + h - 1 - int((threshold - vmin) / (vmax - vmin) * (h - 1))
        for tx in range(x0 + 1, x0 + w - 1, 8):
            draw.line([(tx, ty), (min(tx + 4, x0 + w - 1), ty)], fill=C_THRESH)

    # signal
    pts = _map_values(vals, x0, y0, w, h, vmin, vmax, n_total)
    if len(pts) > 1:
        draw.line(pts, fill=line_color, width=1)

    # event markers (mapped by time, not by array index)
    if events:
        for ev in events:
            if not (0 <= ev.t_peak <= t_end):
                continue
            px = x0 + int(ev.t_peak / t_end * (w - 1))
            c = LABEL_COLORS.get(ev.label, (80, 80, 80))
            draw.line([(px, y0 + 1), (px, y0 + h - 1)], fill=c, width=1)
            draw.ellipse([px - 3, y0 + 8, px + 3, y0 + 14], fill=c,
                         outline=(255, 255, 255))

    # title inside panel, top-left, with white backing for legibility
    tw = int(draw.textlength(title, font=font)) if hasattr(draw, "textlength") \
        else len(title) * 7
    draw.rectangle([x0 + 2, y0 + 2, x0 + tw + 10, y0 + 18], fill=C_BG)
    draw.text((x0 + 6, y0 + 3), title, fill=C_TEXT, font=font)

    # y-axis min/max annotations
    draw.text((x0 - 46, y0 + 1), f"{vmax:7.1f}", fill=C_MUTED, font=font_small)
    draw.text((x0 - 46, y0 + h - 12), f"{vmin:7.1f}", fill=C_MUTED, font=font_small)


def generate_plot(data_path: str, output_path: str,
                  events: Optional[List[AnomalyEvent]] = None) -> str:
    """Generate the multi-panel analysis image."""
    with open(data_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    samples = [IMUSample(
        t=d["t"], ax=d["ax"], ay=d["ay"], az=d["az"],
        gx=d.get("gx", 0), gy=d.get("gy", 0), gz=d.get("gz", 0),
    ) for d in data["samples"]]
    gps = data.get("gps", [])

    cfg = dict(DEFAULTS)
    cfg["sample_rate"] = data.get("scene", {}).get("sample_rate_hz", 100)

    gx, gy, gz = gravity_align(samples, cfg["baseline_frames"])
    residual = high_pass_residual(samples, gx, gy, gz, cfg["hp_window"])

    if events is None:
        events = detect_events(residual, samples, cfg, gps)

    n = len(samples)
    t_end = samples[-1].t if samples else 1.0
    raw_az = [s.az for s in samples]

    # sliding-window energy (aligned to window start times)
    win = cfg["detect_window"]
    stride = cfg["detect_stride"]
    energies = []
    for i in range(0, n - win + 1, stride):
        energies.append(sum(x * x for x in residual[i:i + win]))
    sorted_e = sorted(energies)
    cut = int(len(sorted_e) * 0.9)
    base_e = _mean(sorted_e[:cut]) if cut else _mean(sorted_e)
    std_e = _std(sorted_e[:cut]) if cut > 1 else _std(sorted_e)
    threshold = base_e + cfg["sigma_k"] * std_e
    # expand energy back to full-length by nearest-neighbour for plotting
    energy_full = []
    for k in range(n):
        ei = min(len(energies) - 1, k // stride)
        energy_full.append(energies[ei] if energies else 0.0)

    # ── canvas ──
    W, H = 960, 760
    x0, w = 70, W - 70 - 24
    panel_h = 128
    gap = 34
    y_positions = [40, 40 + panel_h + gap,
                   40 + 2 * (panel_h + gap), 40 + 3 * (panel_h + gap)]

    img = Image.new("RGB", (W, H), C_BG)
    draw = ImageDraw.Draw(img)
    font = _try_font(12, bold=True)
    font_small = _try_font(10)

    # header
    draw.text((x0, 12),
              "IMU Road Anomaly Detection  |  vertical acceleration & event map",
              fill=C_TEXT, font=_try_font(14, bold=True))

    # Panel 1: raw az
    rmin, rmax = min(raw_az), max(raw_az)
    pad = (rmax - rmin) * 0.12 + 0.1
    _panel(draw, x0, y_positions[0], w, panel_h,
           "1   Raw Z acceleration (m/s^2, gravity included)",
           raw_az, rmin - pad, rmax + pad, n, events, t_end,
           font, font_small, line_color=C_SIGNAL)

    # Panel 2: residual
    amin, amax = min(residual), max(residual)
    lim = max(abs(amin), abs(amax)) * 1.12 + 0.1
    _panel(draw, x0, y_positions[1], w, panel_h,
           "2   Gravity-removed high-pass residual (m/s^2)",
           residual, -lim, lim, n, events, t_end,
           font, font_small, line_color=C_SIGNAL, zero_line=True)

    # Panel 3: energy
    emax = max(energy_full) * 1.12 + 1e-6
    _panel(draw, x0, y_positions[2], w, panel_h,
           "3   Sliding-window energy (dashed = 3-sigma threshold)",
           energy_full, 0.0, emax, n, events, t_end,
           font, font_small, line_color=C_SIGNAL2, threshold=threshold)

    # Panel 4: GPS track
    y4 = y_positions[3]
    draw.rectangle([x0, y4, x0 + w, y4 + panel_h], fill=C_BG,
                   outline=C_BORDER, width=1)
    draw.text((x0 + 6, y4 + 3),
              "4   GPS track with detected anomalies (coloured markers)",
              fill=C_TEXT, font=font)
    if gps:
        # Local tangent-plane projection in metres with an *equal* aspect
        # ratio.  Normalising lat/lon independently would turn a straight
        # 16 m drive into a misleading diagonal line.
        lats = [g["lat"] for g in gps]
        lons = [g["lon"] for g in gps]
        lat0, lon0 = lats[0], lons[0]
        m_lat = 110540.0
        m_lon = 111320.0 * math.cos(math.radians(lat0))
        xs = [(lo - lon0) * m_lon for lo in lons]
        ys = [(la - lat0) * m_lat for la in lats]
        span_x = (max(xs) - min(xs)) or 1e-6
        span_y = (max(ys) - min(ys)) or 1e-6
        scale = min((w - 30) / span_x, (panel_h - 30) / span_y)
        cx = x0 + w / 2 - (min(xs) + max(xs)) / 2 * scale
        cy = y4 + panel_h / 2 + (min(ys) + max(ys)) / 2 * scale

        def to_xy_m(x_m, y_m):
            return (int(cx + x_m * scale), int(cy - y_m * scale))

        pts = [to_xy_m(x, y) for x, y in zip(xs, ys)]
        if len(pts) > 1:
            draw.line(pts, fill=C_SIGNAL, width=2)
        draw.ellipse([pts[0][0] - 3, pts[0][1] - 3,
                      pts[0][0] + 3, pts[0][1] + 3], fill=C_SIGNAL)
        for ev in events:
            if ev.lat is not None and ev.lon is not None:
                ex = (ev.lon - lon0) * m_lon
                ey = (ev.lat - lat0) * m_lat
                px, py = to_xy_m(ex, ey)
                c = LABEL_COLORS.get(ev.label, (80, 80, 80))
                draw.ellipse([px - 5, py - 5, px + 5, py + 5], fill=c,
                             outline=(255, 255, 255), width=1)
        draw.text((x0 + w - 235, y4 + panel_h - 16),
                  f"equal-aspect metres  |  span {span_x:.0f} m x "
                  f"{span_y:.0f} m", fill=C_MUTED, font=font_small)
    else:
        draw.text((x0 + w // 2 - 40, y4 + panel_h // 2),
                  "No GPS data", fill=C_MUTED, font=font)

    # time axis
    ay = y4 + panel_h + 8
    draw.line([(x0, ay), (x0 + w, ay)], fill=C_BORDER)
    for k in range(0, int(t_end) + 1, 5):
        tx = x0 + int(k / t_end * (w - 1))
        draw.line([(tx, ay), (tx, ay + 4)], fill=C_BORDER)
        draw.text((tx - 8, ay + 6), f"{k}s", fill=C_MUTED, font=font_small)

    # legend + summary
    ly = H - 42
    lx = x0
    for label, color in LABEL_COLORS.items():
        cnt = sum(1 for e in events if e.label == label)
        draw.ellipse([lx, ly + 2, lx + 9, ly + 11], fill=color,
                     outline=(255, 255, 255))
        draw.text((lx + 14, ly), f"{label} ({cnt})", fill=C_TEXT,
                  font=font_small)
        lx += 150
    draw.text((x0 + w - 250, ly),
              f"{n} samples @ {cfg['sample_rate']} Hz  |  {t_end:.0f} s drive",
              fill=C_MUTED, font=font_small)

    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    img.save(output_path)
    short = os.path.basename(output_path)
    print(f"  Plot saved to {short} "
          f"({os.path.getsize(output_path)} bytes)")
    return output_path


if __name__ == "__main__":
    import sys
    data_path = sys.argv[1] if len(sys.argv) > 1 else "data/imu_drive.json"
    output_path = sys.argv[2] if len(sys.argv) > 2 else "imu_analysis.png"
    generate_plot(data_path, output_path)
