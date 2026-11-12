"""Physics-based IMU data generator for road anomaly detection.

When a live CARLA instance is not available, this module generates realistic
IMU accelerometer data by simulating vehicle suspension dynamics excited by
known road surface profiles.

The model uses a quarter-car suspension (sprung mass + unsprung mass + tire
spring + damper) excited by road displacement profiles for three surface
types:

- **Smooth asphalt**: low-amplitude high-frequency texture noise
- **Speed bump**: raised sinusoidal hump (~10 cm height, ~60 cm width)
- **Pothole**: sudden downward step-recovery (~8 cm depth, ~40 cm width)
- **Rough patch**: sustained medium-amplitude random unevenness (~3 cm RMS)

The vertical acceleration at the vehicle body (sprung mass) is sampled at
100 Hz, matching the CARLA IMU sensor configuration.

This is **not** a substitute for real-world data — it is a physics-grounded
simulator that produces signals with the correct spectral and amplitude
characteristics for algorithm development and testing.
"""

from __future__ import annotations

import json
import math
import os
import random
from typing import List, Tuple


# ── quarter-car model parameters (compact passenger car) ─────────────────

QCM = {
    "m_s": 350.0,     # sprung mass (kg) — body quarter
    "m_u": 45.0,      # unsprung mass (kg)
    "k_s": 16000.0,   # suspension spring (N/m) — softer
    "c_s": 2500.0,    # suspension damper (N·s/m) — well-damped
    "k_t": 200000.0,  # tire vertical stiffness (N/m)
    "g": 9.81,        # gravity (m/s^2)
}


def _road_profile(t: float, speed: float,
                  anomalies: List[Tuple[float, str, dict]]) -> float:
    """Road vertical displacement at time *t* given vehicle *speed*.

    *anomalies* is a list of ``(position_m, type, params)``.
    """
    x = speed * t  # distance traveled
    z = 0.0

    # baseline smooth texture: small high-freq noise
    z += 0.0015 * math.sin(2 * math.pi * 15.0 * t) * \
         (1.0 + 0.3 * math.sin(2 * math.pi * 0.7 * t))
    z += 0.0008 * math.sin(2 * math.pi * 27.0 * t + 1.3)

    for pos, atype, p in anomalies:
        dx = x - pos
        if atype == "speed_bump":
            # raised sinusoidal hump
            height = p.get("height", 0.10)   # 10 cm
            width = p.get("width", 0.60)     # 60 cm
            if abs(dx) < width / 2:
                z += height * 0.5 * (1 - math.cos(2 * math.pi * dx / width))
        elif atype == "pothole":
            # downward step-recovery
            depth = p.get("depth", 0.08)     # 8 cm
            width = p.get("width", 0.40)     # 40 cm
            if abs(dx) < width / 2:
                # sharp dip
                arg = math.pi * (dx / width + 0.5)
                z -= depth * 0.5 * (1 + math.cos(arg)) * \
                     (1.0 - 0.3 * abs(dx) / (width / 2))
        elif atype == "rough_patch":
            # sustained random unevenness
            amp = p.get("amp", 0.003)
            length = p.get("length", 5.0)
            if abs(dx) < length / 2:
                seed_val = int(dx * 100) + p.get("seed", 42)
                rng = random.Random(seed_val)
                z += amp * (rng.random() - 0.5) * 2
        elif atype == "curb":
            # sharp step up
            height = p.get("height", 0.06)
            if 0 < dx < 0.3:
                z += height * (dx / 0.3)
            elif dx >= 0.3:
                z += height

    return z


def _simulate_qcm(road_z: List[float], dt: float) -> List[float]:
    """Simulate quarter-car model; return body vertical acceleration.

    State: [z_s, v_s, z_u, v_u] (sprung pos/vel, unsprung pos/vel)
    """
    m_s = QCM["m_s"]
    m_u = QCM["m_u"]
    k_s = QCM["k_s"]
    c_s = QCM["c_s"]
    k_t = QCM["k_t"]
    g = QCM["g"]

    n = len(road_z)
    dt2 = dt

    # equilibrium positions
    d_t0 = (m_s + m_u) * g / k_t

    # start at equilibrium (z_u relative to road=0, z_s relative to z_u)
    z_s = d_t0 + m_s * g / k_s  # body sits above wheel
    v_s = 0.0
    z_u = 0.0   # tire at road level
    v_u = 0.0

    az_body = []

    # equilibrium spring compression
    d_s0 = m_s * g / k_s
    d_t0_eq = (m_s + m_u) * g / k_t

    for i in range(n):
        r = road_z[i]  # road profile at this point

        # tire force (spring) — compression from equilibrium
        tire_compression = d_t0_eq + (z_u - r)
        F_t = k_t * tire_compression

        # suspension force (spring + damper) — compression from equilibrium
        sus_compression = d_s0 + (z_s - z_u)
        sus_velocity = (v_s - v_u)
        F_s = k_s * sus_compression + c_s * sus_velocity

        # accelerations (net force = gravity - spring/damper forces)
        a_s = (m_s * g - F_s) / m_s  # body acceleration (upward positive)
        a_u = (m_u * g + F_s - F_t) / m_u

        # integrate (semi-implicit Euler)
        v_s += a_s * dt2
        v_u += a_u * dt2
        z_s += v_s * dt2
        z_u += v_u * dt2

        # record body acceleration (upward positive)
        # At equilibrium: a_s = 0 (no net acceleration)
        # IMU reads specific force = g - a_s_response (at rest reads +g)
        # CARLA IMU convention: az includes gravity, +z down → at rest az ≈ +9.81
        az_body.append(a_s)  # body vertical acceleration (will add gravity in samples)

    return az_body


def generate_dataset(output_path: str, duration: float = 30.0,
                     speed: float = 5.0, seed: int = 42) -> dict:
    """Generate a synthetic IMU dataset with known road anomalies.

    Parameters
    ----------
    output_path : str
        Path to write the JSON dataset.
    duration : float
        Recording duration in seconds.
    speed : float
        Vehicle speed in m/s (~29 km/h urban).
    seed : int
        Random seed for reproducibility.

    Returns the generated data dict.
    """
    random.seed(seed)
    fs = 100  # Hz
    dt = 1.0 / fs
    n = int(duration * fs)

    # define anomaly positions (in meters from start)
    # first 10m is smooth for baseline calibration
    anomalies = [
        (20.0, "speed_bump", {"height": 0.06, "width": 0.80}),   # wider → longer signal
        (45.0, "pothole", {"depth": 0.05, "width": 0.35}),
        (70.0, "rough_patch", {"amp": 0.003, "length": 6.0, "seed": 100}),
        (95.0, "pothole", {"depth": 0.04, "width": 0.30}),
        (120.0, "speed_bump", {"height": 0.05, "width": 0.70}),
        (160.0, "rough_patch", {"amp": 0.004, "length": 8.0, "seed": 200}),
        (200.0, "pothole", {"depth": 0.06, "width": 0.40}),
    ]

    # compute road profile
    road_z = [_road_profile(i * dt, speed, anomalies) for i in range(n)]

    # simulate quarter-car dynamics
    az_body = _simulate_qcm(road_z, dt)

    # add lateral and longitudinal components (small vibrations)
    ax_body = [0.02 * math.sin(2 * math.pi * 3.0 * i * dt) +
               0.01 * random.gauss(0, 1) for i in range(n)]
    ay_body = [0.05 * math.sin(2 * math.pi * 1.5 * i * dt + 0.5) +
               0.01 * random.gauss(0, 1) for i in range(n)]

    # add sensor noise
    noise_std = 0.03  # m/s^2
    az_noisy = [v + random.gauss(0, noise_std) for v in az_body]
    ax_noisy = [v + random.gauss(0, noise_std) for v in ax_body]
    ay_noisy = [v + random.gauss(0, noise_std) for v in ay_body]

    # gyro (small values)
    gx = [0.001 * math.sin(2 * math.pi * 2.0 * i * dt) for i in range(n)]
    gy = [0.002 * random.gauss(0, 1) for i in range(n)]
    gz = [0.01 * math.sin(2 * math.pi * 0.5 * i * dt) for i in range(n)]

    # GPS: simulate circular path around Town10HD center
    gps_samples = []
    for i in range(0, n, 10):  # 10 Hz
        t = i * dt
        # simulate GNSS coordinates near CARLA Town10HD
        lat_base = 0.0  # CARLA uses 0,0 for town center in lat/lon
        lon_base = 0.0
        radius = 50.0  # meters
        angle = speed * t / radius
        lat = lat_base + radius * math.cos(angle) * 1e-5  # rough conversion
        lon = lon_base + radius * math.sin(angle) * 1e-5
        gps_samples.append({
            "t": round(t, 4),
            "lat": round(lat, 8),
            "lon": round(lon, 8),
            "alt": 0.0,
        })

    # build samples
    samples = []
    for i in range(n):
        t = i * dt
        samples.append({
            "t": round(t, 4),
            "ax": round(ax_noisy[i], 6),
            "ay": round(ay_noisy[i], 6),
            "az": round(9.81 + az_noisy[i], 6),  # add gravity back (IMU sees g)
            "gx": round(gx[i], 6),
            "gy": round(gy[i], 6),
            "gz": round(gz[i], 6),
        })

    # ground truth
    gt_events = []
    for pos, atype, p in anomalies:
        t_event = pos / speed
        gt_events.append({
            "t": round(t_event, 2),
            "position_m": pos,
            "type": atype,
            "params": p,
        })

    result = {
        "scene": {
            "map": "Town10HD (physics-based simulation)",
            "duration_s": duration,
            "sample_rate_hz": fs,
            "vehicle": "compact_sedan (quarter-car model)",
            "speed_ms": speed,
            "mode": "constant_velocity",
            "model": "quarter_car_suspension",
            "note": "IMU signals generated from quarter-car suspension dynamics "
                    "excited by road surface profiles. Not live CARLA data.",
        },
        "samples": samples,
        "gps": gps_samples,
        "ground_truth": gt_events,
        "n_imu": len(samples),
        "n_gps": len(gps_samples),
        "n_anomalies": len(anomalies),
    }

    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(result, f, ensure_ascii=False)

    print(f"  Generated {len(samples)} IMU samples ({duration:.1f}s @ {fs} Hz)")
    print(f"  Speed: {speed:.1f} m/s, {len(anomalies)} road anomalies")
    print(f"  Ground truth events at t={[e['t'] for e in gt_events]}s")
    print(f"  Saved to {output_path}")
    return result


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(
        description="Generate physics-based IMU dataset")
    parser.add_argument("--output", default="data/imu_drive.json")
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--speed", type=float, default=8.0)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()
    generate_dataset(args.output, args.duration, args.speed, args.seed)
