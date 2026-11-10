"""Offline deterministic demo for the yaw command guard."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

from .core import YawGuard, YawGuardConfig, shortest_angle, wrap_angle
from .scenario import target_yaw_at


def run_demo(
    duration_s: float = 8.0,
    rate_hz: float = 20.0,
    plant_yaw_rate_deg_s: float = 120.0,
) -> tuple[list[dict[str, float | str]], dict[str, object]]:
    guard = YawGuard(
        YawGuardConfig(
            control_rate_hz=rate_hz, max_yaw_rate_deg_s=90.0
        )
    )
    dt = 1.0 / rate_hz
    plant_rate = math.radians(plant_yaw_rate_deg_s)
    current = 0.0
    rows: list[dict[str, float | str]] = []
    limited_samples = 0
    max_abs_rate = 0.0
    steps = int(round(duration_s * rate_hz)) + 1
    for index in range(steps):
        now_s = index * dt
        target = target_yaw_at(now_s)
        guard.update_current(current, now_s)
        guard.update_target(target, now_s)
        result = guard.step(now_s)
        assert result is not None
        limited_samples += int(result.limited)
        max_abs_rate = max(max_abs_rate, abs(result.yaw_rate_rad_s))
        rows.append(
            {
                "time_s": now_s,
                "mode": result.mode,
                "current_yaw_deg": math.degrees(result.current_yaw_rad),
                "target_yaw_deg": math.degrees(result.target_yaw_rad),
                "error_deg": math.degrees(result.error_rad),
                "rate_deg_s": math.degrees(result.yaw_rate_rad_s),
                "limited": int(result.limited),
            }
        )
        current = wrap_angle(
            current
            + max(-plant_rate, min(plant_rate, result.yaw_rate_rad_s)) * dt
        )
    summary = {
        "duration_s": duration_s,
        "rate_hz": rate_hz,
        "samples": len(rows),
        "limited_samples": limited_samples,
        "max_abs_rate_deg_s": round(math.degrees(max_abs_rate), 3),
        "final_error_deg": round(
            math.degrees(shortest_angle(target_yaw_at(duration_s), current)), 3
        ),
    }
    return rows, summary


def _write_outputs(
    rows: list[dict[str, float | str]],
    summary: dict[str, object],
    output_dir: Path,
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / "runtime_demo.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    (output_dir / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        _write_pillow_plot(rows, output_dir / "runtime_demo.png")
        return
    time_values = [float(row["time_s"]) for row in rows]
    current_values = [float(row["current_yaw_deg"]) for row in rows]
    target_values = [float(row["target_yaw_deg"]) for row in rows]
    rate_values = [float(row["rate_deg_s"]) for row in rows]
    figure, (ax_yaw, ax_rate) = plt.subplots(
        2, 1, figsize=(10, 7), sharex=True
    )
    ax_yaw.plot(time_values, target_values, label="target yaw", linewidth=1.5)
    ax_yaw.plot(time_values, current_values, label="current yaw", linewidth=1.8)
    ax_yaw.set_ylabel("yaw (deg)")
    ax_yaw.grid(True, alpha=0.3)
    ax_yaw.legend(loc="best")
    ax_rate.plot(time_values, rate_values, color="tab:orange", label="rate command")
    ax_rate.axhline(90.0, color="tab:red", linestyle="--", label="+limit")
    ax_rate.axhline(-90.0, color="tab:red", linestyle="--", label="-limit")
    ax_rate.set_xlabel("time (s)")
    ax_rate.set_ylabel("yaw rate (deg/s)")
    ax_rate.grid(True, alpha=0.3)
    ax_rate.legend(loc="best")
    figure.suptitle("UAV yaw shortest-path and rate-limit demo")
    figure.tight_layout()
    figure.savefig(output_dir / "runtime_demo.png", dpi=150)
    plt.close(figure)


def _write_pillow_plot(
    rows: list[dict[str, float | str]], output_path: Path
) -> None:
    try:
        from PIL import Image, ImageDraw
    except ImportError:
        return

    width, height, margin = 1100, 700, 75
    image = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(image)
    plot_height = 230
    upper = (margin, 55, width - margin, 55 + plot_height)
    lower = (margin, 375, width - margin, 375 + plot_height)

    time_values = [float(row["time_s"]) for row in rows]
    current_values = [float(row["current_yaw_deg"]) for row in rows]
    target_values = [float(row["target_yaw_deg"]) for row in rows]
    rate_values = [float(row["rate_deg_s"]) for row in rows]
    time_max = max(time_values) or 1.0

    def point(
        box: tuple[int, int, int, int],
        time_s: float,
        value: float,
        value_min: float,
        value_max: float,
    ) -> tuple[float, float]:
        left, top, right, bottom = box
        x = left + (right - left) * time_s / time_max
        y = bottom - (bottom - top) * (value - value_min) / (
            value_max - value_min
        )
        return x, y

    def draw_panel(
        box: tuple[int, int, int, int],
        title: str,
        y_min: float,
        y_max: float,
    ) -> None:
        left, top, right, bottom = box
        draw.rectangle(box, outline="#666666", width=1)
        for fraction in (0.0, 0.5, 1.0):
            y = bottom - (bottom - top) * fraction
            value = y_min + (y_max - y_min) * fraction
            draw.line((left, y, right, y), fill="#dddddd")
            draw.text((8, y - 6), f"{value:.0f}", fill="#444444")
        draw.text((left, top - 24), title, fill="#222222")

    draw_panel(upper, "Target and current yaw (deg)", -180.0, 180.0)
    draw_panel(lower, "Yaw-rate command (deg/s)", -100.0, 100.0)
    for values, color in ((target_values, "#888888"), (current_values, "#1565c0")):
        points = [
            point(upper, time_s, value, -180.0, 180.0)
            for time_s, value in zip(time_values, values)
        ]
        draw.line(points, fill=color, width=2)
    rate_points = [
        point(lower, time_s, value, -100.0, 100.0)
        for time_s, value in zip(time_values, rate_values)
    ]
    draw.line(rate_points, fill="#ef6c00", width=2)
    for limit in (-90.0, 90.0):
        y = point(lower, 0.0, limit, -100.0, 100.0)[1]
        draw.line((lower[0], y, lower[2], y), fill="#c62828", width=1)
    draw.text((margin, 20), "UAV yaw shortest-path and rate-limit demo", fill="#111111")
    draw.text((margin, 665), "time (s)", fill="#333333")
    draw.text((width - 265, 330), "grey: target   blue: current", fill="#333333")
    draw.text((width - 265, 650), "orange: rate   red: limits", fill="#333333")
    image.save(output_path)


def main(args=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--rate-hz", type=float, default=20.0)
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/demo"))
    parsed = parser.parse_args(args)
    rows, summary = run_demo(parsed.duration, parsed.rate_hz)
    _write_outputs(rows, summary, parsed.output_dir)
    print(json.dumps(summary, ensure_ascii=False))


if __name__ == "__main__":
    main()
