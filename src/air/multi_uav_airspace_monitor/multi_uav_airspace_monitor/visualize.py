"""Pillow-based diagnostic figure for the airspace monitor."""

from __future__ import annotations

from pathlib import Path
import textwrap

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from .monitor import evaluate_scenario


COLORS = ["#2E86AB", "#E67E22", "#27AE60", "#8E44AD", "#C0392B"]


def _font(size: int) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    for name in ("arial.ttf", "DejaVuSans.ttf"):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            continue
    return ImageFont.load_default()


def render_scenario(scenario: dict, output_path: str | Path) -> dict:
    """Render a top-down airspace picture and return the summary."""
    summary = evaluate_scenario(scenario)
    width, height = 1060, 720
    left, right, top, bottom = 70, width - 330, 70, height - 70
    image = Image.new("RGB", (width, height), "#F7F9FB")
    draw = ImageDraw.Draw(image, "RGBA")

    tracks = scenario["uavs"]
    all_points = np.vstack([np.asarray(track["positions"])[:, :2] for track in tracks])
    for zone in scenario.get("zones", []):
        all_points = np.vstack([all_points, np.asarray(zone["polygon"], dtype=float)])
    minimum = all_points.min(axis=0) - 30.0
    maximum = all_points.max(axis=0) + 30.0
    span = np.maximum(maximum - minimum, 1.0)
    scale = min((right - left) / span[0], (bottom - top) / span[1])

    def project(point: np.ndarray) -> tuple[float, float]:
        x = left + (float(point[0]) - minimum[0]) * scale
        y = bottom - (float(point[1]) - minimum[1]) * scale
        return x, y

    # Grid and axes.
    for value in range(int(np.floor(minimum[0] / 50) * 50), int(maximum[0]) + 1, 50):
        x, _ = project(np.array([value, 0.0]))
        draw.line((x, top, x, bottom), fill="#E2E8F0", width=1)
        draw.text((x - 10, bottom + 10), f"{value}", fill="#64748B", font=_font(11))
    for value in range(int(np.floor(minimum[1] / 50) * 50), int(maximum[1]) + 1, 50):
        _, y = project(np.array([0.0, value]))
        draw.line((left, y, right, y), fill="#E2E8F0", width=1)
        draw.text((left - 45, y - 7), f"{value}", fill="#64748B", font=_font(11))

    # Restricted zones.
    for zone in scenario.get("zones", []):
        polygon = [project(point) for point in np.asarray(zone["polygon"], dtype=float)]
        draw.polygon(polygon, fill=(220, 53, 69, 55), outline="#C0392B")
        tx, ty = polygon[0]
        draw.text((tx + 6, ty + 6), zone["name"], fill="#922B21", font=_font(11))

    # Tracks.
    for index, track in enumerate(tracks):
        points = [project(point) for point in np.asarray(track["positions"])]
        color = COLORS[index % len(COLORS)]
        draw.line(points, fill=color, width=3)
        sx, sy = points[0]
        ex, ey = points[-1]
        draw.ellipse((sx - 5, sy - 5, sx + 5, sy + 5), fill=color)
        draw.rectangle((ex - 5, ey - 5, ex + 5, ey + 5), fill=color)
        draw.text((ex + 7, ey - 7), track["id"], fill=color, font=_font(12))

    # Conflicts.
    for conflict in summary["conflicts"]:
        cx, cy = project(np.asarray(conflict["position"], dtype=float))
        radius = max(9.0, conflict["separation_m"] * scale + 9.0)
        draw.ellipse(
            (cx - radius, cy - radius, cx + radius, cy + radius),
            outline="#C0392B",
            width=3,
        )
        draw.line((cx - 12, cy, cx + 12, cy), fill="#C0392B", width=2)
        draw.line((cx, cy - 12, cx, cy + 12), fill="#C0392B", width=2)

    # Side panel.
    panel_x = right + 25
    draw.text((panel_x, 55), "Airspace Monitor", fill="#1E293B", font=_font(22))
    draw.text(
        (panel_x, 90),
        f"Status: {summary['status'].upper()}",
        fill="#C0392B" if summary["status"] == "critical" else "#1E8449",
        font=_font(16),
    )
    draw.text(
        (panel_x, 118),
        f"UAVs: {summary['uav_count']}    Zones: {summary['zone_count']}",
        fill="#334155",
        font=_font(13),
    )
    min_sep = summary["minimum_separation_m"]
    draw.text(
        (panel_x, 142),
        f"Minimum separation: {min_sep:.1f} m" if min_sep is not None else "No overlap",
        fill="#334155",
        font=_font(13),
    )

    y = 185
    draw.text((panel_x, y), "Alerts", fill="#1E293B", font=_font(16))
    y += 28
    for alert in summary["alerts"][:4]:
        color = "#C0392B" if alert["severity"] == "critical" else "#B7791F"
        lines = textwrap.wrap(
            f"{alert['severity']}: {alert['message']}", width=38
        ) or [alert["message"]]
        for line in lines:
            draw.text((panel_x, y), line, fill=color, font=_font(10))
            y += 16
        y += 10
    if not summary["alerts"]:
        draw.text((panel_x, y), "No geofence violation", fill="#64748B", font=_font(12))
        y += 34

    y += 6
    draw.text((panel_x, y), "Conflicts", fill="#1E293B", font=_font(16))
    y += 28
    for conflict in summary["conflicts"][:3]:
        draw.text(
            (panel_x, y),
            f"{conflict['uav_a']} vs {conflict['uav_b']}",
            fill="#C0392B",
            font=_font(12),
        )
        y += 18
        draw.text(
            (panel_x, y),
            f"{conflict['separation_m']:.1f} m at t={conflict['time_s']:.1f}s",
            fill="#475569",
            font=_font(11),
        )
        y += 32
    if not summary["conflicts"]:
        draw.text((panel_x, y), "No conflict", fill="#64748B", font=_font(12))

    draw.text(
        (left, 30),
        "Top-down ENU view | circle = pairwise conflict | red rectangle = no-fly zone",
        fill="#475569",
        font=_font(13),
    )

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    image.save(output_path)
    return summary
