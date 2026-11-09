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
        self.dt = 1.0 / max(self.rate_hz, 1.0)
        self.controller = controller or GimbalController()
        self.plant = GimbalPlant()
        self.elapsed = 0.0
        self.rows = []

    def step(self):
        target_pan, target_tilt = target_angles(self.elapsed)
        current = self.plant.state
        command = self.controller.command(
            target_pan, target_tilt, current.pan, current.tilt, self.dt
        )
        state = self.plant.step(command, self.dt)
        self.elapsed += self.dt
        error = angular_error(target_pan, target_tilt, state.pan, state.tilt)
        row = {
            "t": self.elapsed,
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
        if not path:
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
        if not path:
            return
        from PIL import Image, ImageDraw

        width, height = 900, 600
        image = Image.new("RGB", (width, height), "white")
        draw = ImageDraw.Draw(image)
        margin_left, margin_top, panel_h = 75, 35, 235
        panel_w = width - margin_left - 30

        def x_of(t):
            return margin_left + panel_w * t / max(self.duration, 1e-9)

        def draw_panel(top, values, color, y_min, y_max, title):
            draw.rectangle((margin_left, top, margin_left + panel_w, top + panel_h),
                           outline=(180, 180, 180))
            draw.text((margin_left, top - 20), title, fill="black")
            span = max(y_max - y_min, 1e-9)
            points = [
                (x_of(row["t"]), top + panel_h * (1.0 - (value - y_min) / span))
                for row, value in zip(self.rows, values)
            ]
            if len(points) > 1:
                draw.line(points, fill=color, width=2)

        pan_values = [row["target_pan"] for row in self.rows] + [
            row["pan"] for row in self.rows
        ]
        pan_min = min(pan_values) - 2.0
        pan_max = max(pan_values) + 2.0
        first_top = margin_top + 25
        second_top = first_top + panel_h + 70
        draw_panel(first_top, [row["target_pan"] for row in self.rows],
                   (30, 90, 220), pan_min, pan_max, "pan target (blue)")
        draw_panel(first_top, [row["pan"] for row in self.rows],
                   (220, 90, 30), pan_min, pan_max, "")
        errors = [row["error_deg"] for row in self.rows]
        draw_panel(second_top, errors, (200, 40, 40),
                   0.0, max(max(errors) + 1.0, 1.0), "2-axis error (deg)")
        draw.text((margin_left, height - 28),
                  "time: 0 s to %.1f s" % self.duration, fill="black")
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
