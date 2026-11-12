"""Deterministic flight segments and the measured wind triangle."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class WindConfig:
    """True wind, airspeed and measurement noise of one survey."""

    wind_m_s: tuple = (3.0, -2.0)
    airspeed_m_s: float = 12.0
    gps_noise_m_s: float = 0.2
    airspeed_noise_m_s: float = 0.15
    airspeed_variation_m_s: float = 0.35
    heading_noise_deg: float = 1.0
    samples_per_leg: int = 20
    seed: int = 0

    def validate(self) -> None:
        if self.airspeed_m_s <= 0.0:
            raise ValueError("airspeed must be positive")
        if self.gps_noise_m_s < 0.0 or self.airspeed_noise_m_s < 0.0:
            raise ValueError("noise must be non-negative")
        if self.samples_per_leg < 1:
            raise ValueError("samples_per_leg must be positive")

    @property
    def wind(self) -> np.ndarray:
        return np.asarray(self.wind_m_s, dtype=float)


MANEUVERS = ("hover", "straight", "reciprocal", "circle")


def headings_for(maneuver: str, legs: int = 6) -> np.ndarray:
    """Heading unit vectors of a manoeuvre."""
    if maneuver == "hover":
        angles = np.zeros(legs)
    elif maneuver == "straight":
        angles = np.zeros(legs)
    elif maneuver == "reciprocal":
        angles = np.array([0.0, np.pi] * (legs // 2))
    elif maneuver == "circle":
        angles = np.linspace(0.0, 2.0 * np.pi, legs, endpoint=False)
    else:
        raise ValueError(f"unknown maneuver {maneuver!r}")
    return np.stack([np.cos(angles), np.sin(angles)], axis=1)


def simulate_maneuver(config: WindConfig, maneuver: str) -> dict:
    """Ground velocity, heading and airspeed measurements of a manoeuvre."""
    config.validate()
    rng = np.random.default_rng(config.seed + MANEUVERS.index(maneuver) * 17)
    headings = headings_for(maneuver)
    headings = np.repeat(headings, config.samples_per_leg, axis=0)
    true_heading = np.arctan2(headings[:, 1], headings[:, 0])
    noise = np.deg2rad(config.heading_noise_deg)
    measured_heading = true_heading + rng.normal(0.0, noise, true_heading.size)
    measured_headings = np.stack(
        [np.cos(measured_heading), np.sin(measured_heading)], axis=1
    )
    true_ground = config.airspeed_m_s * headings + config.wind[None, :]
    ground = true_ground + rng.normal(0.0, config.gps_noise_m_s, true_ground.shape)
    leg_variation = rng.uniform(
        -config.airspeed_variation_m_s, config.airspeed_variation_m_s, true_ground.shape[0]
    )
    true_airspeed = config.airspeed_m_s + leg_variation
    true_ground = true_airspeed[:, None] * headings + config.wind[None, :]
    ground = true_ground + rng.normal(0.0, config.gps_noise_m_s, true_ground.shape)
    airspeed = true_airspeed + rng.normal(
        0.0, config.airspeed_noise_m_s, true_ground.shape[0]
    )
    return {
        "headings": measured_headings,
        "true_headings": headings,
        "ground_velocity": ground,
        "true_ground_velocity": true_ground,
        "airspeed": airspeed,
        "wind": config.wind,
        "maneuver": maneuver,
    }
