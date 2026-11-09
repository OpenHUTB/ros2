"""Synthetic two-axis gimbal plant and closed-loop metrics."""

from __future__ import annotations

from dataclasses import dataclass
import csv
import math
import os

from gimbal_controller import GimbalController, angular_error


@dataclass(frozen=True)
class GimbalState:
    pan: float = 0.0
    tilt: float = 0.0


def target_angles(elapsed):
    """Smooth target motion used by both the CLI and ROS demo."""
    return (
        24.0 * math.sin(2.0 * math.pi * elapsed / 8.0),
        12.0 * math.sin(2.0 * math.pi * elapsed / 10.0 + 0.5),
    )


class GimbalPlant:
    """Simple rate-command plant; the command is degrees per second."""

    def __init__(self):
        self.state = GimbalState()

    def step(self, command, dt):
        pan = self.state.pan + float(command.pan_rate) * float(dt)
        tilt = self.state.tilt + float(command.tilt_rate) * float(dt)
        self.state = GimbalState(
            pan=max(-85.0, min(85.0, pan)),
            tilt=max(-45.0, min(45.0, tilt)),
        )
        return self.state


class GimbalSimulation:
    """Closed-loop simulation with per-step diagnostics."""

    def __init__(self, duration=12.0, rate_hz=50.0, controller=None):
        self.duration = float(duration)
        self.rate_hz = float(rate_hz)
        if not math.isfinite(self.duration) or self.duration <= 0.0:
            raise ValueError("duration must be finite and positive")
        if not math.isfinite(self.rate_hz) or self.rate_hz <= 0.0:
            raise ValueError("rate_hz must be finite and positive")
        self.dt = 1.0 / self.rate_hz
        self.controller = controller or GimbalController()
        self.plant = GimbalPlant()
        self.elapsed = 0.0
        self.rows = []

    def step(self):
        t = self.elapsed
        target_pan, target_tilt = target_angles(t)
        current = self.plant.state
        command = self.controller.command(
            target_pan, target_tilt, current.pan, current.tilt, self.dt
        )
        state = self.plant.step(command, self.dt)
        error = angular_error(target_pan, target_tilt, state.pan, state.tilt)
        row = {
            "t": t,
            "target_pan": target_pan,
            "target_tilt": target_tilt,
            "pan": state.pan,
            "tilt": state.tilt,
            "pan_rate": command.pan_rate,
            "tilt_rate": command.tilt_rate,
            "error_deg": error,
            "saturated": int(command.pan_saturated or command.tilt_saturated),
        }
        self.rows.append(row)
        self.elapsed += self.dt
        return row

    def run(self):
        while self.elapsed + self.dt * 0.5 < self.duration:
            self.step()
        return self.metrics()

    def metrics(self):
        if not self.rows:
            return {
                "samples": 0,
                "rmse_deg": 0.0,
                "max_error_deg": 0.0,
                "saturation_ratio": 0.0,
            }
        errors = [row["error_deg"] for row in self.rows]
        return {
            "samples": len(self.rows),
            "rmse_deg": math.sqrt(sum(error * error for error in errors) / len(errors)),
            "max_error_deg": max(errors),
            "saturation_ratio": sum(row["saturated"] for row in self.rows) / float(len(self.rows)),
        }

    def write_csv(self, path):
        if not path or not self.rows:
            return
        path = os.path.expanduser(path)
        parent = os.path.dirname(path)
        if parent:
            os.makedirs(parent, exist_ok=True)
        with open(path, "w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(self.rows[0]))
            writer.writeheader()
            writer.writerows(self.rows)

    def plot(self, path):
        if not path or not self.rows:
            return
        from PIL import Image, ImageDraw, ImageFont

        def make_font(size):
            try:
                return ImageFont.truetype("DejaVuSans.ttf", size)
            except OSError:
                try:
                    return ImageFont.load_default(size=size)
                except TypeError:
                    return ImageFont.load_default()

        font = make_font(17)
        small_font = make_font(14)
        title_font = make_font(21)
        width, height = 1200, 760
        image = Image.new("RGB", (width, height), "white")
        draw = ImageDraw.Draw(image)
        left, right = 105, 50
        top = 60
        panel_w = width - left - right
        panel_h = 330
        error_top = 500
        error_h = 190

        def x_of(t):
            return left + panel_w * t / max(self.duration, 1e-9)

        def y_of(value, panel_top, panel_height, y_min, y_max):
            span = max(y_max - y_min, 1e-9)
            return panel_top + panel_height * (1.0 - (value - y_min) / span)

        def draw_panel(panel_top, panel_height, y_min, y_max, title, show_x=False):
            draw.rectangle(
                (left, panel_top, left + panel_w, panel_top + panel_height),
                outline=(150, 150, 150),
                width=1,
            )
            draw.text((left, panel_top - 32), title, fill="black", font=title_font)
            for tick in range(5):
                ratio = tick / 4.0
                y = panel_top + panel_height * ratio
                value = y_max - (y_max - y_min) * ratio
                draw.line((left, y, left + panel_w, y), fill=(232, 232, 232))
                draw.text(
                    (12, y - 9),
                    "%.1f" % value,
                    fill=(70, 70, 70),
                    font=small_font,
                )
            for tick in range(7):
                value = self.duration * tick / 6.0
                x = x_of(value)
                draw.line(
                    (x, panel_top, x, panel_top + panel_height),
                    fill=(240, 240, 240),
                )
                if show_x:
                    draw.text(
                        (x - 10, panel_top + panel_height + 8),
                        "%.0f" % value,
                        fill=(70, 70, 70),
                        font=small_font,
                    )

        def draw_series(
            key,
            panel_top,
            panel_height,
            y_min,
            y_max,
            color,
            width_px=2,
            dashed=False,
        ):
            points = [
                (
                    x_of(row["t"]),
                    y_of(row[key], panel_top, panel_height, y_min, y_max),
                )
                for row in self.rows
            ]
            if dashed:
                for index, (start, end) in enumerate(zip(points, points[1:])):
                    if (index // 4) % 2 == 0:
                        draw.line((start, end), fill=color, width=width_px)
            elif len(points) > 1:
                draw.line(points, fill=color, width=width_px)

        colors = {
            "target_pan": (66, 133, 244),
            "pan": (13, 71, 161),
            "target_tilt": (251, 140, 0),
            "tilt": (216, 27, 96),
        }
        all_angles = []
        for key in colors:
            all_angles.extend(row[key] for row in self.rows)
        angle_low = min(all_angles) - 2.0
        angle_high = max(all_angles) + 2.0
        draw_panel(
            top,
            panel_h,
            angle_low,
            angle_high,
            "Pan and tilt tracking (deg)",
        )
        draw_series("target_pan", top, panel_h, angle_low, angle_high,
                    colors["target_pan"], dashed=True)
        draw_series("pan", top, panel_h, angle_low, angle_high, colors["pan"])
        draw_series("target_tilt", top, panel_h, angle_low, angle_high,
                    colors["target_tilt"], dashed=True)
        draw_series("tilt", top, panel_h, angle_low, angle_high, colors["tilt"])

        legend_x = left + 18
        legend_y = top + 14
        legends = (
            ("target_pan", "Pan target", True),
            ("pan", "Pan response", False),
            ("target_tilt", "Tilt target", True),
            ("tilt", "Tilt response", False),
        )
        draw.rectangle(
            (legend_x - 6, legend_y - 6, legend_x + 205, legend_y + 104),
            fill="white",
            outline=(205, 205, 205),
        )
        for index, (key, label, dashed) in enumerate(legends):
            item_y = legend_y + index * 24
            if dashed:
                for offset in range(0, 40, 12):
                    draw.line(
                        (legend_x + offset, item_y + 8,
                         legend_x + min(offset + 7, 40), item_y + 8),
                        fill=colors[key],
                        width=3,
                    )
            else:
                draw.line(
                    (legend_x, item_y + 8, legend_x + 40, item_y + 8),
                    fill=colors[key],
                    width=3,
                )
            draw.text(
                (legend_x + 48, item_y),
                label,
                fill=(35, 35, 35),
                font=small_font,
            )

        errors = [row["error_deg"] for row in self.rows]
        error_high = max(max(errors) + 1.0, 1.0)
        draw_panel(
            error_top,
            error_h,
            0.0,
            error_high,
            "Two-axis angular error (deg)",
            show_x=True,
        )
        draw_series(
            "error_deg",
            error_top,
            error_h,
            0.0,
            error_high,
            (198, 40, 40),
            width_px=3,
        )
        draw.text(
            (left + panel_w / 2 - 28, height - 30),
            "time (s)",
            fill="black",
            font=font,
        )
        path = os.path.expanduser(path)
        parent = os.path.dirname(path)
        if parent:
            os.makedirs(parent, exist_ok=True)
        image.save(path)


def run_simulation(duration=12.0, rate_hz=50.0, csv_path="", plot_path=""):
    simulation = GimbalSimulation(duration=duration, rate_hz=rate_hz)
    metrics = simulation.run()
    simulation.write_csv(csv_path)
    simulation.plot(plot_path)
    return simulation, metrics


def main(argv=None):
    import argparse

    parser = argparse.ArgumentParser(description="UAV gimbal stabilization simulation")
    parser.add_argument("--duration", type=float, default=12.0)
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--csv", default="")
    parser.add_argument("--plot", default="")
    args = parser.parse_args(argv)
    _, metrics = run_simulation(args.duration, args.rate, args.csv, args.plot)
    print("samples=%d rmse_deg=%.3f max_error_deg=%.3f saturation_ratio=%.3f" % (
        metrics["samples"], metrics["rmse_deg"], metrics["max_error_deg"],
        metrics["saturation_ratio"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
