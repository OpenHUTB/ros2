"""Pillow figure for the survey plan."""

from __future__ import annotations

from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from .planner import SurveyPlan


def _font(size: int) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    for name in ("arial.ttf", "DejaVuSans.ttf"):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            continue
    return ImageFont.load_default()


def render_plan(polygon: np.ndarray, plan: SurveyPlan, output_path: str | Path) -> None:
    width, height = 1060, 720
    left, right, top, bottom = 70, width - 330, 70, height - 60
    image = Image.new("RGB", (width, height), "#F8FAFC")
    draw = ImageDraw.Draw(image, "RGBA")

    points = np.vstack([polygon, plan.waypoints_enu])
    minimum = points.min(axis=0) - 20
    maximum = points.max(axis=0) + 20
    span = np.maximum(maximum - minimum, 1.0)
    scale = min((right - left) / span[0], (bottom - top) / span[1])

    def project(point):
        px = left + (float(point[0]) - minimum[0]) * scale
        py = bottom - (float(point[1]) - minimum[1]) * scale
        return px, py

    # Grid.
    for value in range(int(minimum[0] // 50 * 50), int(maximum[0]) + 1, 50):
        px, _ = project((value, 0))
        draw.line((px, top, px, bottom), fill="#E2E8F0", width=1)
        draw.text((px - 8, bottom + 8), str(value), fill="#64748B", font=_font(10))
    for value in range(int(minimum[1] // 50 * 50), int(maximum[1]) + 1, 50):
        _, py = project((0, value))
        draw.line((left, py, right, py), fill="#E2E8F0", width=1)
        draw.text((left - 38, py - 6), str(value), fill="#64748B", font=_font(10))

    polygon_pixels = [project(point) for point in polygon]
    draw.polygon(polygon_pixels, fill=(14, 165, 233, 30), outline="#0284C7")
    for start, end in plan.scan_segments:
        draw.line((project(start), project(end)), fill="#F59E0B", width=2)
    for east, north in plan.waypoints_enu:
        px, py = project((east, north))
        draw.ellipse((px - 3, py - 3, px + 3, py + 3), fill="#DC2626")

    panel_x = right + 24
    draw.text((panel_x, 55), "Survey Mission", fill="#1E293B", font=_font(22))
    lines = [
        ("Altitude", f"{plan.altitude_m:.1f} m"),
        ("GSD", f"{plan.gsd_m * 100:.2f} cm/px"),
        ("Footprint", f"{plan.footprint_width_m:.1f} x {plan.footprint_height_m:.1f} m"),
        ("Line spacing", f"{plan.line_spacing_m:.1f} m"),
        ("Shot spacing", f"{plan.shot_spacing_m:.1f} m"),
        ("Lines", str(plan.line_count)),
        ("Images", str(plan.image_count)),
        ("Waypoints", str(len(plan.waypoints_enu))),
        ("Distance", f"{plan.flight_distance_m / 1000:.2f} km"),
        ("Time", f"{plan.flight_time_s / 60:.1f} min"),
        ("Coverage", f"{plan.coverage_ratio:.1%}"),
    ]
    y = 100
    for label, value in lines:
        draw.text((panel_x, y), label, fill="#475569", font=_font(12))
        draw.text((panel_x + 150, y), value, fill="#0F172A", font=_font(12))
        y += 28

    draw.text((panel_x, y + 12), "Warnings", fill="#1E293B", font=_font(15))
    warning_y = y + 38
    for warning in plan.warnings[:4]:
        words = warning.split()
        line = ""
        for word in words:
            if len(line) + len(word) + 1 > 38:
                draw.text((panel_x, warning_y), line, fill="#B45309", font=_font(10))
                warning_y += 15
                line = word
            else:
                line = f"{line} {word}".strip()
        if line:
            draw.text((panel_x, warning_y), line, fill="#B45309", font=_font(10))
            warning_y += 18
    if not plan.warnings:
        draw.text((panel_x, warning_y), "None", fill="#64748B", font=_font(11))

    draw.text(
        (left, 30),
        "Survey plan in local ENU | blue polygon = site | orange = scan lines | red = waypoints",
        fill="#475569",
        font=_font(13),
    )
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    image.save(output_path)
