"""Offline deterministic demo for the gradeability checker."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

from .core import GradeabilityChecker, GradeabilityConfig
from .scenario import slope_at, speed_at


def run_demo(
    duration_s: float = 10.0,
    rate_hz: float = 20.0,
) -> tuple[list[dict[str, float | str | bool]], dict[str, object]]:
    config = GradeabilityConfig(control_rate_hz=rate_hz)
    checker = GradeabilityChecker(config)
    dt = 1.0 / rate_hz
    rows: list[dict[str, float | str | bool]] = []
    climbable = 0
    traction_limited = 0
    power_limited = 0
    min_margin = float("inf")
    steps = int(round(duration_s * rate_hz)) + 1
    for index in range(steps):
        now_s = index * dt
        slope = slope_at(now_s)
        speed = speed_at(now_s)
        checker.update_slope(slope, now_s)
        checker.update_speed(speed, now_s)
        result = checker.step(now_s)
        assert result is not None
        climbable += int(result.mode == "CLIMBABLE")
        traction_limited += int(result.mode == "TRACTION_LIMIT")
        power_limited += int(result.mode == "POWER_LIMIT")
        min_margin = min(min_margin, result.margin_n)
        rows.append(
            {
                "time_s": now_s,
                "mode": result.mode,
                "slope_deg": round(result.slope_deg, 3),
                "speed_mps": round(result.speed_mps, 3),
                "required_n": round(result.required_n, 3),
                "drive_limit_n": round(result.drive_limit_n, 3),
                "traction_limit_n": round(result.traction_limit_n, 3),
                "available_n": round(result.available_n, 3),
                "margin_n": round(result.margin_n, 3),
                "slip_risk": result.slip_risk,
                "power_deficit": result.power_deficit,
            }
        )
    summary = {
        "duration_s": duration_s,
        "rate_hz": rate_hz,
        "samples": len(rows),
        "climbable_samples": climbable,
        "traction_limit_samples": traction_limited,
        "power_limit_samples": power_limited,
        "min_margin_n": round(min_margin, 3),
        "max_climbable_slope_deg": checker.max_climbable_slope_deg(
            speed_mps=6.0
        ),
        "final_mode": rows[-1]["mode"],
    }
    return rows, summary


def _write_outputs(
    rows: list[dict[str, float | str | bool]],
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
    slope_values = [float(row["slope_deg"]) for row in rows]
    speed_values = [float(row["speed_mps"]) for row in rows]
    required_values = [float(row["required_n"]) for row in rows]
    available_values = [float(row["available_n"]) for row in rows]
    margin_values = [float(row["margin_n"]) for row in rows]

    figure, (ax_slope, ax_force) = plt.subplots(
        2, 1, figsize=(10, 7), sharex=True
    )
    ax_slope.step(time_values, slope_values, where="post",
                  label="slope (deg)", linewidth=1.8, color="tab:blue")
    ax_slope.set_ylabel("slope (deg)")
    ax_slope.grid(True, alpha=0.3)
    ax_slope.legend(loc="upper left")
    ax_speed = ax_slope.twinx()
    ax_speed.plot(time_values, speed_values, label="speed (m/s)",
                  linewidth=1.2, color="tab:green", linestyle="--")
    ax_speed.set_ylabel("speed (m/s)")
    ax_speed.legend(loc="upper right")

    ax_force.plot(time_values, required_values, label="required force",
                  linewidth=1.8, color="tab:red")
    ax_force.plot(time_values, available_values, label="available force",
                  linewidth=1.5, color="tab:blue")
    ax_force.plot(time_values, margin_values, label="margin",
                  linewidth=1.5, color="tab:orange")
    ax_force.axhline(0.0, color="tab:gray", linestyle=":")
    ax_force.set_xlabel("time (s)")
    ax_force.set_ylabel("force (N)")
    ax_force.grid(True, alpha=0.3)
    ax_force.legend(loc="best")
    figure.suptitle("Vehicle gradeability and traction-margin demo")
    figure.tight_layout()
    figure.savefig(output_dir / "runtime_demo.png", dpi=150)
    plt.close(figure)


def _write_pillow_plot(
    rows: list[dict[str, float | str | bool]], output_path: Path
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
    slope_values = [float(row["slope_deg"]) for row in rows]
    speed_values = [float(row["speed_mps"]) for row in rows]
    required_values = [float(row["required_n"]) for row in rows]
    available_values = [float(row["available_n"]) for row in rows]
    margin_values = [float(row["margin_n"]) for row in rows]
    time_max = max(time_values) or 1.0

    def point(box, time_s, value, value_min, value_max):
        left, top, right, bottom = box
        x = left + (right - left) * time_s / time_max
        y = bottom - (bottom - top) * (value - value_min) / (
            value_max - value_min
        )
        return x, y

    def draw_panel(box, title, y_min, y_max):
        left, top, right, bottom = box
        draw.rectangle(box, outline="#666666", width=1)
        for fraction in (0.0, 0.5, 1.0):
            y = bottom - (bottom - top) * fraction
            value = y_min + (y_max - y_min) * fraction
            draw.line((left, y, right, y), fill="#dddddd")
            draw.text((8, y - 6), f"{value:.0f}", fill="#444444")
        draw.text((left, top - 24), title, fill="#222222")

    draw_panel(upper, "Slope (deg) and speed (m/s)", 0.0, 25.0)
    draw_panel(lower, "Force (N): required / available / margin", -2000.0, 7000.0)
    for values, color in ((slope_values, "#1565c0"), (speed_values, "#2e7d32")):
        points = [
            point(upper, time_s, value, 0.0, 25.0)
            for time_s, value in zip(time_values, values)
        ]
        draw.line(points, fill=color, width=2)
    for values, color in (
        (required_values, "#c62828"),
        (available_values, "#1565c0"),
        (margin_values, "#ef6c00"),
    ):
        points = [
            point(lower, time_s, value, -2000.0, 7000.0)
            for time_s, value in zip(time_values, values)
        ]
        draw.line(points, fill=color, width=2)
    draw.text((margin, 20), "Vehicle gradeability and traction-margin demo",
              fill="#111111")
    draw.text((margin, 665), "time (s)", fill="#333333")
    draw.text((width - 300, 330), "red: required  blue: available",
              fill="#333333")
    draw.text((width - 300, 650), "orange: margin", fill="#333333")
    image.save(output_path)


def main(args=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--rate-hz", type=float, default=20.0)
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/demo"))
    parsed = parser.parse_args(args)
    rows, summary = run_demo(parsed.duration, parsed.rate_hz)
    _write_outputs(rows, summary, parsed.output_dir)
    print(json.dumps(summary, ensure_ascii=False))


if __name__ == "__main__":
    main()
