"""Synthetic IMU/GPS/barometer run used by the offline demo and tests."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from .attitude import ComplementaryAttitude, GRAVITY_NED, euler_to_rotation
from .ekf import NavigationEKF


@dataclass
class SimulationResult:
    times: np.ndarray
    true_position: np.ndarray
    estimated_position: np.ndarray
    dead_reckoning_position: np.ndarray
    gps_position: np.ndarray
    true_altitude: np.ndarray
    estimated_altitude: np.ndarray
    true_yaw: np.ndarray
    estimated_yaw: np.ndarray
    metrics: dict

    def as_dict(self) -> dict:
        return {
            "duration_s": round(float(self.times[-1]), 3),
            "samples": int(len(self.times)),
            **self.metrics,
        }


def _body_rates(euler: np.ndarray, dt: float) -> np.ndarray:
    derivative = np.gradient(euler, dt, axis=0)
    roll_dot, pitch_dot, yaw_dot = derivative[:, 0], derivative[:, 1], derivative[:, 2]
    roll, pitch = euler[:, 0], euler[:, 1]
    p = roll_dot - yaw_dot * np.sin(pitch)
    q = pitch_dot * np.cos(roll) + yaw_dot * np.sin(roll) * np.cos(pitch)
    r = -pitch_dot * np.sin(roll) + yaw_dot * np.cos(roll) * np.cos(pitch)
    return np.column_stack([p, q, r])


def run_simulation(
    duration_s: float = 60.0,
    dt: float = 0.01,
    seed: int = 7,
    gps_sigma_m: float = 1.8,
    baro_sigma_m: float = 0.35,
    debug: bool = False,
) -> SimulationResult:
    """Run a deterministic synthetic flight with wind gust and sensor noise."""
    rng = np.random.default_rng(seed)
    times = np.arange(0.0, duration_s + dt, dt)
    count = len(times)

    base_accel = np.column_stack(
        [
            -1.2 * np.sin(0.20 * times),
            -0.20 * np.sin(0.10 * times),
            -0.27 * np.sin(0.30 * times),
        ]
    )
    gust_envelope = np.clip((times - 24.0) / 4.0, 0.0, 1.0) * np.clip(
        (34.0 - times) / 4.0, 0.0, 1.0
    )
    gust = np.column_stack(
        [
            1.4 * gust_envelope * np.sin(2.0 * np.pi * 0.45 * times),
            -0.8 * gust_envelope * np.sin(2.0 * np.pi * 0.32 * times),
            np.zeros(count),
        ]
    )
    true_accel = base_accel + gust

    true_velocity = np.zeros((count, 3))
    true_position = np.zeros((count, 3))
    true_velocity[0] = np.array([3.0, 1.0, 0.0])
    true_position[0] = np.array([0.0, 0.0, -25.0])
    for index in range(1, count):
        true_velocity[index] = true_velocity[index - 1] + true_accel[index - 1] * dt
        true_position[index] = (
            true_position[index - 1]
            + 0.5 * (true_velocity[index - 1] + true_velocity[index]) * dt
        )

    euler = np.column_stack(
        [
            0.045 * np.sin(0.30 * times),
            0.035 * np.sin(0.24 * times),
            0.25 + 0.18 * np.sin(0.12 * times),
        ]
    )
    rotations = np.array([euler_to_rotation(*angles) for angles in euler])
    specific_force = np.einsum(
        "nij,nj->ni", rotations.transpose(0, 2, 1), true_accel - GRAVITY_NED
    )
    gyro = _body_rates(euler, dt)

    accel_bias = np.array([0.025, -0.018, 0.012])
    gyro_bias = np.array([0.0008, -0.0005, 0.0004])
    accel_measurement = specific_force + accel_bias + rng.normal(0.0, 0.08, (count, 3))
    gyro_measurement = gyro + gyro_bias + rng.normal(0.0, 0.004, (count, 3))
    mag_world = np.array([1.0, 0.0, 0.0])
    mag_measurement = np.einsum(
        "nij,j->ni", rotations.transpose(0, 2, 1), mag_world
    ) + rng.normal(0.0, 0.01, (count, 3))

    gps_position = np.full((count, 3), np.nan)
    baro_altitude = np.full(count, np.nan)
    for index in range(count):
        if index % 20 == 0:
            gps_position[index] = true_position[index] + rng.normal(
                0.0, [gps_sigma_m, gps_sigma_m, 2.5]
            )
        if index % 10 == 0:
            baro_altitude[index] = true_position[index, 2] + rng.normal(0.0, baro_sigma_m)

    first_gps = gps_position[0]
    first_baro = baro_altitude[0]
    initial_position = np.array([first_gps[0], first_gps[1], first_baro])
    ekf = NavigationEKF(initial_position)
    attitude = ComplementaryAttitude()
    dead_position = initial_position.copy()
    dead_velocity = np.zeros(3)

    sample_stride = 10
    estimated_samples = []
    dead_samples = []
    estimated_yaw_samples = []
    true_yaw_samples = []
    sample_indices = []
    for index in range(count):
        roll, pitch, yaw = attitude.update(
            gyro_measurement[index], accel_measurement[index], mag_measurement[index], dt
        )
        # Attitude is treated as an input from the flight controller / VIO.
        # A complementary filter is still evaluated below, but accelerometer
        # based tilt is biased by the vehicle's own acceleration.
        # This synthetic run already produces FRD specific force, so it feeds the
        # EKF directly; the ROS adapter converts REP-103 (FLU) input with
        # ``frames.step_imu`` before calling the EKF.
        rotation_for_navigation = rotations[index]
        ekf.predict(accel_measurement[index], rotation_for_navigation, dt)
        if not np.isnan(gps_position[index, 0]):
            if debug and index % 200 == 0:
                print(
                    "gps",
                    index,
                    "p_trace",
                    float(np.trace(ekf.P)),
                    "innovation",
                    gps_position[index, 0:2] - ekf.position[0:2],
                )
            ekf.update_position(gps_position[index, 0:2], gps_sigma_m)
        if not np.isnan(baro_altitude[index]):
            ekf.update_altitude(baro_altitude[index], baro_sigma_m)

        dead_accel = rotation_for_navigation @ accel_measurement[index] + GRAVITY_NED
        dead_position += dead_velocity * dt + 0.5 * dead_accel * dt * dt
        dead_velocity += dead_accel * dt

        if index % sample_stride == 0:
            sample_indices.append(index)
            estimated_samples.append(ekf.position)
            dead_samples.append(dead_position.copy())
            estimated_yaw_samples.append(yaw)
            true_yaw_samples.append(float(euler[index, 2]))

    sample_indices = np.asarray(sample_indices)
    estimated_position = np.asarray(estimated_samples)
    dead_position_samples = np.asarray(dead_samples)
    sample_times = times[sample_indices]
    true_samples = true_position[sample_indices]

    valid = sample_times >= 5.0
    ekf_error = np.linalg.norm(estimated_position[valid] - true_samples[valid], axis=1)
    dead_error = np.linalg.norm(
        dead_position_samples[valid] - true_samples[valid], axis=1
    )

    gps_reference = np.full_like(true_samples, np.nan)
    last_gps = np.array([first_gps[0], first_gps[1], first_baro])
    for sample_index, time_index in enumerate(sample_indices):
        if not np.isnan(gps_position[time_index, 0]):
            last_gps = np.array(
                [gps_position[time_index, 0], gps_position[time_index, 1], true_position[time_index, 2]]
            )
        gps_reference[sample_index] = last_gps
    gps_error = np.linalg.norm(gps_reference[valid] - true_samples[valid], axis=1)
    yaw_error = np.angle(
        np.exp(1j * (np.asarray(true_yaw_samples) - np.asarray(estimated_yaw_samples)))
    )

    metrics = {
        "ekf_rmse_m": round(float(np.sqrt(np.mean(ekf_error**2))), 4),
        "dead_reckoning_rmse_m": round(float(np.sqrt(np.mean(dead_error**2))), 4),
        "gps_rmse_m": round(float(np.sqrt(np.mean(gps_error**2))), 4),
        "ekf_final_error_m": round(float(ekf_error[-1]), 4),
        "complementary_yaw_rmse_deg": round(
            float(np.degrees(np.sqrt(np.mean(yaw_error**2)))), 4
        ),
        "gps_sigma_m": float(gps_sigma_m),
        "baro_sigma_m": float(baro_sigma_m),
    }

    return SimulationResult(
        times=sample_times,
        true_position=true_samples,
        estimated_position=estimated_position,
        dead_reckoning_position=dead_position_samples,
        gps_position=gps_reference,
        true_altitude=true_samples[:, 2],
        estimated_altitude=estimated_position[:, 2],
        true_yaw=np.asarray(true_yaw_samples),
        estimated_yaw=np.asarray(estimated_yaw_samples),
        metrics=metrics,
    )
