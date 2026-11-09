"""Synthetic altitude sensor stream and fusion comparison."""

from __future__ import annotations

import csv
import math
import os

import numpy as np

from altitude_filter import AltitudeKalman, rmse


def truth_at(t):
    """Smooth takeoff-like ascent to 8 m with a slow final hover."""
    altitude = 8.0 * (1.0 - math.exp(-max(float(t), 0.0) / 2.0))
    velocity = 4.0 * math.exp(-max(float(t), 0.0) / 2.0)
    acceleration = -2.0 * math.exp(-max(float(t), 0.0) / 2.0)
    return altitude, velocity, acceleration


class AltitudeSimulation:
    def __init__(
        self,
        duration=12.0,
        rate_hz=50.0,
        seed=3,
        barometer_bias=0.45,
        accel_bias=0.08,
    ):
        self.duration = float(duration)
        self.rate_hz = float(rate_hz)
        self.dt = 1.0 / max(self.rate_hz, 1.0)
        self.rng = np.random.default_rng(seed)
        self.barometer_bias = float(barometer_bias)
        self.accel_bias = float(accel_bias)
        self.elapsed = 0.0
        self.rows = []
        self.filter = None
        self.integrated_altitude = 0.0
        self.integrated_velocity = 0.0

    def step(self):
        truth, true_velocity, true_acceleration = truth_at(self.elapsed)
        barometer = truth + self.barometer_bias + self.rng.normal(0.0, 0.35)
        acceleration = true_acceleration + self.accel_bias + self.rng.normal(0.0, 0.10)
        if self.filter is None:
            self.filter = AltitudeKalman(initial_altitude=barometer)
            self.integrated_altitude = barometer
        self.integrated_velocity += acceleration * self.dt
        self.integrated_altitude += self.integrated_velocity * self.dt
        self.filter.predict(acceleration, self.dt)
        fused, filtered_velocity = self.filter.update(barometer)
        self.elapsed += self.dt
        row = {
            "t": self.elapsed,
            "truth": truth,
            "barometer": barometer,
            "accel_integrated": self.integrated_altitude,
            "fused": fused,
            "fused_velocity": filtered_velocity,
            "baro_error": barometer - truth,
            "fused_error": fused - truth,
            "integrated_error": self.integrated_altitude - truth,
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
                "baro_rmse": 0.0,
                "integrated_rmse": 0.0,
                "fused_rmse": 0.0,
                "final_fused_error": 0.0,
            }
        truth = [row["truth"] for row in self.rows]
        return {
            "samples": len(self.rows),
            "baro_rmse": rmse([row["barometer"] for row in self.rows], truth),
            "integrated_rmse": rmse(
                [row["accel_integrated"] for row in self.rows], truth
            ),
            "fused_rmse": rmse([row["fused"] for row in self.rows], truth),
            "final_fused_error": self.rows[-1]["fused_error"],
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

        width, height = 900, 620
        image = Image.new("RGB", (width, height), "white")
        draw = ImageDraw.Draw(image)
        left, top, panel_h = 75, 35, 235
        panel_w = width - left - 30

        def x_of(t):
            return left + panel_w * t / max(self.duration, 1e-9)

        def panel(panel_top, values, color, y_min, y_max, label):
            draw.rectangle((left, panel_top, left + panel_w, panel_top + panel_h),
                           outline=(180, 180, 180))
            draw.text((left, panel_top - 20), label, fill="black")
            span = max(y_max - y_min, 1e-9)
            points = [
                (x_of(row["t"]), panel_top + panel_h * (1.0 - (value - y_min) / span))
                for row, value in zip(self.rows, values)
            ]
            if len(points) > 1:
                draw.line(points, fill=color, width=2)

        all_values = [row["truth"] for row in self.rows] + [
            row["barometer"] for row in self.rows
        ]
        low, high = min(all_values) - 1.0, max(all_values) + 1.0
        panel(top + 25, [row["truth"] for row in self.rows],
              (30, 90, 220), low, high, "true altitude (blue)")
        panel(top + 25, [row["fused"] for row in self.rows],
              (220, 90, 30), low, high, "")
        errors = [row["fused_error"] for row in self.rows]
        panel(top + panel_h + 95, errors, (200, 40, 40),
              min(errors) - 0.2, max(errors) + 0.2, "fused error (m)")
        draw.text((left, height - 28),
                  "time: 0 s to %.1f s" % self.duration, fill="black")
        path = os.path.expanduser(path)
        parent = os.path.dirname(path)
        if parent:
            os.makedirs(parent, exist_ok=True)
        image.save(path)


def run_simulation(duration=12.0, rate_hz=50.0, csv_path="", plot_path="", seed=3):
    simulation = AltitudeSimulation(duration=duration, rate_hz=rate_hz, seed=seed)
    metrics = simulation.run()
    simulation.write_csv(csv_path)
    simulation.plot(plot_path)
    return simulation, metrics


def main(argv=None):
    import argparse

    parser = argparse.ArgumentParser(description="UAV IMU/barometer altitude fusion")
    parser.add_argument("--duration", type=float, default=12.0)
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--seed", type=int, default=3)
    parser.add_argument("--csv", default="")
    parser.add_argument("--plot", default="")
    args = parser.parse_args(argv)
    _, metrics = run_simulation(
        args.duration, args.rate, args.csv, args.plot, args.seed
    )
    print(
        "samples=%d baro_rmse=%.3f integrated_rmse=%.3f fused_rmse=%.3f "
        "final_fused_error=%.3f"
        % (
            metrics["samples"],
            metrics["baro_rmse"],
            metrics["integrated_rmse"],
            metrics["fused_rmse"],
            metrics["final_fused_error"],
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
