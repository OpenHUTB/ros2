"""Synthetic altitude sensor stream and fusion comparison."""

from __future__ import annotations

import csv
import os

import numpy as np

from altitude_filter import AltitudeKalman, rmse


def truth_at(t):
    """Smooth takeoff from rest to an 8 m hover over 8 s."""
    t = max(float(t), 0.0)
    duration = 8.0
    target_altitude = 8.0
    if t >= duration:
        return target_altitude, 0.0, 0.0
    u = t / duration
    altitude = target_altitude * (10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5)
    velocity = target_altitude * (
        30.0 * u**2 - 60.0 * u**3 + 30.0 * u**4
    ) / duration
    acceleration = target_altitude * (
        60.0 * u - 180.0 * u**2 + 120.0 * u**3
    ) / (duration ** 2)
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
        t = self.elapsed
        truth, true_velocity, true_acceleration = truth_at(t)
        barometer = truth + self.barometer_bias + self.rng.normal(0.0, 0.35)
        acceleration = true_acceleration + self.accel_bias + self.rng.normal(0.0, 0.10)
        if self.filter is None:
            self.filter = AltitudeKalman(initial_altitude=barometer)
            # The baseline comparison knows the true initial state. This keeps
            # the integrated result from inheriting an unrelated initial offset.
            self.integrated_altitude = truth
            self.integrated_velocity = true_velocity
        self.integrated_velocity += acceleration * self.dt
        self.integrated_altitude += self.integrated_velocity * self.dt
        self.filter.predict(acceleration, self.dt)
        fused, filtered_velocity = self.filter.update(barometer)
        row = {
            "t": t,
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
        width, height = 1200, 780
        image = Image.new("RGB", (width, height), "white")
        draw = ImageDraw.Draw(image)
        left, right = 105, 50
        top = 65
        panel_w = width - left - right
        altitude_h = 375
        error_top = 520
        error_h = 175

        colors = {
            "truth": (31, 78, 216),
            "barometer": (224, 138, 28),
            "accel_integrated": (128, 70, 180),
            "fused": (214, 48, 48),
        }
        labels = {
            "truth": "True altitude",
            "barometer": "Barometer",
            "accel_integrated": "Acceleration integration",
            "fused": "Kalman fusion",
        }

        def x_of(t):
            return left + panel_w * t / max(self.duration, 1e-9)

        def y_of(value, panel_top, panel_height, y_min, y_max):
            span = max(y_max - y_min, 1e-9)
            return panel_top + panel_height * (1.0 - (value - y_min) / span)

        def draw_panel(panel_top, panel_height, y_min, y_max, title):
            draw.rectangle(
                (left, panel_top, left + panel_w, panel_top + panel_height),
                outline=(150, 150, 150),
                width=1,
            )
            draw.text((left, panel_top - 34), title, fill="black", font=title_font)
            for tick in range(5):
                ratio = tick / 4.0
                y = panel_top + panel_height * ratio
                value = y_max - (y_max - y_min) * ratio
                draw.line(
                    (left, y, left + panel_w, y), fill=(232, 232, 232), width=1
                )
                draw.text(
                    (12, y - 9), "%.1f" % value, fill=(70, 70, 70), font=small_font
                )
            for tick in range(7):
                value = self.duration * tick / 6.0
                x = x_of(value)
                draw.line(
                    (x, panel_top, x, panel_top + panel_height),
                    fill=(240, 240, 240),
                    width=1,
                )
                draw.text(
                    (x - 10, panel_top + panel_height + 8),
                    "%.0f" % value,
                    fill=(70, 70, 70),
                    font=small_font,
                )

        def draw_series(key, panel_top, panel_height, y_min, y_max, width_px=3):
            points = [
                (
                    x_of(row["t"]),
                    y_of(row[key], panel_top, panel_height, y_min, y_max),
                )
                for row in self.rows
            ]
            if len(points) > 1:
                draw.line(points, fill=colors[key], width=width_px)

        all_values = [row["truth"] for row in self.rows] + [
            row["barometer"] for row in self.rows
        ] + [row["accel_integrated"] for row in self.rows] + [
            row["fused"] for row in self.rows
        ]
        altitude_low = min(all_values) - 0.5
        altitude_high = max(all_values) + 0.5
        draw_panel(
            top,
            altitude_h,
            altitude_low,
            altitude_high,
            "Altitude comparison (m)",
        )
        for key in ("truth", "barometer", "accel_integrated", "fused"):
            draw_series(key, top, altitude_h, altitude_low, altitude_high)

        legend_x = left + panel_w - 330
        legend_y = top + 14
        for index, key in enumerate(
            ("truth", "barometer", "accel_integrated", "fused")
        ):
            item_y = legend_y + index * 25
            draw.line(
                (legend_x, item_y + 8, legend_x + 38, item_y + 8),
                fill=colors[key],
                width=4,
            )
            draw.text(
                (legend_x + 48, item_y),
                labels[key],
                fill=(35, 35, 35),
                font=small_font,
            )

        error_keys = ("baro_error", "integrated_error", "fused_error")
        error_values = [
            row[key] for row in self.rows for key in error_keys
        ]
        error_low = min(error_values) - 0.25
        error_high = max(error_values) + 0.25
        draw_panel(
            error_top,
            error_h,
            error_low,
            error_high,
            "Height error (m)",
        )
        for key in error_keys:
            source = {
                "baro_error": "barometer",
                "integrated_error": "accel_integrated",
                "fused_error": "fused",
            }[key]
            points = [
                (
                    x_of(row["t"]),
                    y_of(row[key], error_top, error_h, error_low, error_high),
                )
                for row in self.rows
            ]
            if len(points) > 1:
                draw.line(points, fill=colors[source], width=2)
        draw.text(
            (left + panel_w / 2 - 28, height - 32),
            "time (s)",
            fill="black",
            font=font,
        )
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
