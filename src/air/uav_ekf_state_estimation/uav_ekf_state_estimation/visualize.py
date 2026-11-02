"""Pillow diagnostic figure for the EKF demo."""

from __future__ import annotations

from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from .simulation import SimulationResult


def _font(size: int) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    for name in ("arial.ttf", "DejaVuSans.ttf"):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            continue
    return ImageFont.load_default()


def render_result(result: SimulationResult, output_path: str | Path) -> None:
    width, height = 1080, 760
    margin_left, margin_right = 70, 40
    top, bottom_margin = 70, 60
    image = Image.new("RGB", (width, height), "#F8FAFC")
    draw = ImageDraw.Draw(image, "RGBA")

    panel_width = (width - margin_left - margin_right - 30) // 2
    panel_height = (height - top - bottom_margin - 30) // 2
    panels = [
        (margin_left, top),
        (margin_left + panel_width + 30, top),
        (margin_left, top + panel_height + 30),
        (margin_left + panel_width + 30, top + panel_height + 30),
    ]

    # Panel 1: top-down trajectory.
    x0, y0 = panels[0]
    draw.rectangle((x0, y0, x0 + panel_width, y0 + panel_height), outline="#CBD5E1")
    points = np.vstack([result.true_position[:, :2], result.estimated_position[:, :2]])
    minimum = points.min(axis=0) - 15
    maximum = points.max(axis=0) + 15
    span = np.maximum(maximum - minimum, 1.0)
    scale = min((panel_width - 30) / span[0], (panel_height - 40) / span[1])

    def project(point):
        px = x0 + 15 + (point[0] - minimum[0]) * scale
        py = y0 + panel_height - 20 - (point[1] - minimum[1]) * scale
        return px, py

    true_line = [project(point) for point in result.true_position[:, :2]]
    ekf_line = [project(point) for point in result.estimated_position[:, :2]]
    draw.line(true_line, fill="#111827", width=3)
    draw.line(ekf_line, fill="#0EA5E9", width=2)
    for point in result.gps_position[::5, :2]:
        px, py = project(point)
        draw.ellipse((px - 2, py - 2, px + 2, py + 2), fill="#F59E0B")
    draw.text((x0 + 12, y0 + 10), "Top-down trajectory (ENU)", fill="#1E293B", font=_font(14))
    draw.text((x0 + 12, y0 + 30), "black=true  blue=EKF  orange=GPS", fill="#64748B", font=_font(10))

    # Panel 2: position error.
    x0, y0 = panels[1]
    draw.rectangle((x0, y0, x0 + panel_width, y0 + panel_height), outline="#CBD5E1")
    ekf_error = np.linalg.norm(result.estimated_position - result.true_position, axis=1)
    dead_error = np.linalg.norm(
        result.dead_reckoning_position - result.true_position, axis=1
    )
    maximum_error = max(float(np.max(ekf_error)), float(np.max(dead_error)), 1.0)
    for error, color, label in [
        (ekf_error, "#0EA5E9", "EKF"),
        (dead_error, "#EF4444", "dead reckoning"),
    ]:
        line = []
        for index, value in enumerate(error):
            px = x0 + 12 + index / max(1, len(error) - 1) * (panel_width - 24)
            py = y0 + panel_height - 25 - value / maximum_error * (panel_height - 85)
            line.append((px, py))
        draw.line(line, fill=color, width=2)
        draw.text(
            (x0 + 12, y0 + 34 + (0 if label == "EKF" else 18)),
            label,
            fill=color,
            font=_font(11),
        )
    draw.text((x0 + 12, y0 + 10), "3-D position error", fill="#1E293B", font=_font(14))
    draw.text(
        (x0 + panel_width - 120, y0 + 12),
        f"max {maximum_error:.2f} m",
        fill="#64748B",
        font=_font(10),
    )

    # Panel 3: altitude.
    x0, y0 = panels[2]
    draw.rectangle((x0, y0, x0 + panel_width, y0 + panel_height), outline="#CBD5E1")
    altitude_min = min(float(np.min(result.true_altitude)), float(np.min(result.estimated_altitude))) - 2
    altitude_max = max(float(np.max(result.true_altitude)), float(np.max(result.estimated_altitude))) + 2
    for values, color, label in [
        (result.true_altitude, "#111827", "true"),
        (result.estimated_altitude, "#0EA5E9", "EKF"),
    ]:
        line = []
        for index, value in enumerate(values):
            px = x0 + 12 + index / max(1, len(values) - 1) * (panel_width - 24)
            py = y0 + panel_height - 25 - (value - altitude_min) / max(1e-6, altitude_max - altitude_min) * (panel_height - 85)
            line.append((px, py))
        draw.line(line, fill=color, width=2)
    draw.text((x0 + 12, y0 + 10), "Altitude (NED z, m)", fill="#1E293B", font=_font(14))
    draw.text((x0 + 12, y0 + 32), "black=true  blue=EKF", fill="#64748B", font=_font(10))

    # Panel 4: metrics.
    x0, y0 = panels[3]
    draw.rectangle((x0, y0, x0 + panel_width, y0 + panel_height), outline="#CBD5E1")
    draw.text((x0 + 12, y0 + 10), "Metrics after 5 s", fill="#1E293B", font=_font(14))
    y = y0 + 45
    for label, value in [
        ("EKF RMSE", f"{result.metrics['ekf_rmse_m']:.3f} m"),
        ("Dead reckoning RMSE", f"{result.metrics['dead_reckoning_rmse_m']:.3f} m"),
        ("Raw GPS RMSE", f"{result.metrics['gps_rmse_m']:.3f} m"),
        ("Final EKF error", f"{result.metrics['ekf_final_error_m']:.3f} m"),
        ("Complementary yaw RMSE", f"{result.metrics['complementary_yaw_rmse_deg']:.2f} deg"),
    ]:
        draw.text((x0 + 16, y), label, fill="#475569", font=_font(12))
        draw.text((x0 + panel_width - 150, y), value, fill="#0F172A", font=_font(12))
        y += 34
    draw.text(
        (x0 + 16, y + 10),
        "Synthetic IMU + GPS + barometer\nwind gust between 24 and 34 s",
        fill="#64748B",
        font=_font(10),
    )

    draw.text(
        (margin_left, 28),
        "uav_ekf_state_estimation | IMU/GPS/barometer fusion",
        fill="#1E293B",
        font=_font(20),
    )
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    image.save(output_path)
