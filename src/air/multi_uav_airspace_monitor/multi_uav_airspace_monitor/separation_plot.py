"""Distance-vs-time chart for the exported separation CSV (Pillow only).

The module already depends on Pillow for the top-down airspace picture, so the
plot stays dependency-free as well: same CSV in, same PNG out on any machine.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Iterable, Mapping

from PIL import Image, ImageDraw, ImageFont

# One colour per UAV pair, cycled when a scenario has more pairs than colours.
_COLOURS = (
    (214, 69, 65),
    (43, 110, 190),
    (46, 160, 110),
    (168, 90, 190),
    (222, 143, 32),
    (60, 160, 190),
    (120, 120, 120),
    (180, 120, 60),
)

_MARGIN_LEFT = 74
_MARGIN_RIGHT = 210
_MARGIN_TOP = 52
_MARGIN_BOTTOM = 58


def _font(size: int = 13):
    try:
        return ImageFont.load_default(size=size)
    except TypeError:  # Pillow < 10.1 has no size argument
        return ImageFont.load_default()


def _nice_step(value: float) -> float:
    """Round a tick spacing up to 1/2/5 x 10^n."""
    if value <= 0:
        return 1.0
    magnitude = 10 ** int(f"{value:e}".split("e")[1])
    for factor in (1, 2, 5, 10):
        if factor * magnitude >= value:
            return factor * magnitude
    return 10 * magnitude


def plot_separation_series(
    rows: Iterable[Mapping[str, Any]], path: Path | str, size: tuple[int, int] = (960, 520)
) -> Path:
    """Draw one line per UAV pair (time on x, separation on y) and save a PNG."""
    series: dict[tuple[str, str], list[tuple[float, float]]] = {}
    for row in rows:
        pair = (str(row["uav_a"]), str(row["uav_b"]))
        series.setdefault(pair, []).append((float(row["time_s"]), float(row["separation_m"])))
    for points in series.values():
        points.sort()

    width, height = size
    plot_width = width - _MARGIN_LEFT - _MARGIN_RIGHT
    plot_height = height - _MARGIN_TOP - _MARGIN_BOTTOM
    image = Image.new("RGB", (width, height), (255, 255, 255))
    draw = ImageDraw.Draw(image)
    font = _font(13)
    small = _font(11)

    max_time = max((point[0] for points in series.values() for point in points), default=1.0)
    max_distance = max((point[1] for points in series.values() for point in points), default=1.0)
    max_time = max(max_time, 1e-6)
    max_distance = max(max_distance * 1.05, 1e-6)

    def to_px(time_s: float, distance_m: float) -> tuple[float, float]:
        x = _MARGIN_LEFT + plot_width * (time_s / max_time)
        y = _MARGIN_TOP + plot_height * (1.0 - distance_m / max_distance)
        return x, y

    # axes and grid
    left, right = _MARGIN_LEFT, width - _MARGIN_RIGHT
    top, bottom = _MARGIN_TOP, height - _MARGIN_BOTTOM
    time_step = _nice_step(max_time / 6.0)
    distance_step = _nice_step(max_distance / 5.0)
    tick = 0.0
    while tick <= max_time + 1e-9:
        x, _ = to_px(tick, 0.0)
        draw.line([(x, top), (x, bottom)], fill=(235, 235, 235))
        draw.text((x - 8, bottom + 6), f"{tick:g}", font=small, fill=(90, 90, 90))
        tick += time_step
    tick = 0.0
    while tick <= max_distance + 1e-9:
        _, y = to_px(0.0, tick)
        draw.line([(left, y), (right, y)], fill=(235, 235, 235))
        draw.text((8, y - 7), f"{tick:g}", font=small, fill=(90, 90, 90))
        tick += distance_step
    draw.rectangle([left, top, right, bottom], outline=(150, 150, 150))

    # warning / critical guides if they fit in the visible range
    for level, colour in ((30.0, (222, 143, 32)), (10.0, (214, 69, 65))):
        if level <= max_distance:
            _, y = to_px(0.0, level)
            draw.line([(left, y), (right, y)], fill=colour, width=1)
            draw.text((right - 92, y - 13), f"{level:g} m", font=small, fill=colour)

    for index, pair in enumerate(sorted(series)):
        colour = _COLOURS[index % len(_COLOURS)]
        points = [to_px(time_s, distance_m) for time_s, distance_m in series[pair]]
        if len(points) > 1:
            draw.line(points, fill=colour, width=2)
        elif points:
            x, y = points[0]
            draw.ellipse([x - 2, y - 2, x + 2, y + 2], fill=colour)

    draw.text((_MARGIN_LEFT, 16), "Pairwise separation vs time (from --csv export)",
              font=font, fill=(30, 30, 30))
    draw.text((left, bottom + 34), "time (s)", font=small, fill=(60, 60, 60))
    draw.text((8, 34), "m", font=small, fill=(60, 60, 60))

    legend_x = right + 14
    legend_y = top + 8
    draw.text((legend_x, legend_y - 18), "legend", font=small, fill=(60, 60, 60))
    for index, pair in enumerate(sorted(series)):
        colour = _COLOURS[index % len(_COLOURS)]
        y = legend_y + index * 18
        draw.line([(legend_x, y + 6), (legend_x + 16, y + 6)], fill=colour, width=2)
        draw.text((legend_x + 22, y), f"{pair[0]} / {pair[1]}", font=small, fill=(60, 60, 60))
    if not series:
        draw.text((left + 12, top + 12), "no UAV pair in this scenario", font=small,
                  fill=(120, 120, 120))

    target = Path(path)
    if target.parent and not target.parent.exists():
        target.parent.mkdir(parents=True, exist_ok=True)
    image.save(target, optimize=True)
    return target
