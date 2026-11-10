"""One-dimensional IMU/barometer altitude fusion."""

from __future__ import annotations

import math


class AltitudeKalman:
    """Kalman filter with state [altitude, vertical velocity]."""

    def __init__(
        self,
        initial_altitude=0.0,
        initial_velocity=0.0,
        process_altitude=0.015,
        process_velocity=0.08,
        barometer_sigma=0.35,
    ):
        self.altitude = float(initial_altitude)
        self.velocity = float(initial_velocity)
        self.p00 = 1.0
        self.p01 = 0.0
        self.p10 = 0.0
        self.p11 = 1.0
        self.q_altitude = float(process_altitude)
        self.q_velocity = float(process_velocity)
        self.r_barometer = float(barometer_sigma) ** 2

    @property
    def state(self):
        return self.altitude, self.velocity

    def predict(self, acceleration, dt):
        dt = max(float(dt), 1e-4)
        acceleration = float(acceleration)

        self.altitude += self.velocity * dt + 0.5 * acceleration * dt * dt
        self.velocity += acceleration * dt

        p00 = self.p00 + 2.0 * dt * self.p01 + dt * dt * self.p11
        p01 = self.p01 + dt * self.p11
        p11 = self.p11
        self.p00 = p00 + self.q_altitude
        self.p01 = p01
        self.p10 = p01
        self.p11 = p11 + self.q_velocity
        return self.altitude, self.velocity

    def update(self, barometer_altitude):
        innovation = float(barometer_altitude) - self.altitude
        s = self.p00 + self.r_barometer
        if s <= 1e-12:
            return self.altitude, self.velocity
        k0 = self.p00 / s
        k1 = self.p10 / s
        self.altitude += k0 * innovation
        self.velocity += k1 * innovation

        p00 = (1.0 - k0) * self.p00
        p01 = (1.0 - k0) * self.p01
        p11 = self.p11 - k1 * self.p01
        self.p00 = max(p00, 1e-9)
        self.p01 = p01
        self.p10 = p01
        self.p11 = max(p11, 1e-9)
        return self.altitude, self.velocity


def rmse(values, truth):
    if not values:
        return float("nan")
    return math.sqrt(
        sum((float(value) - float(reference)) ** 2
            for value, reference in zip(values, truth)) / len(values)
    )
