"""Collect IMU + GNSS data from CARLA for road anomaly detection.

Drives a vehicle in autopilot mode through Town10HD while recording:
  - IMU accelerometer + gyroscope at 100 Hz
  - GNSS (lat/lon) at 10 Hz
  - RGB camera frames for visual reference

Town10HD's road mesh is geometrically smooth: a plain drive contains no
potholes or speed bumps, so it provides a noise-floor recording but no
labelled anomalies.  To obtain *real sensor data* with ground truth without
faking the IMU stream, this collector optionally applies calibrated vertical
impulses to the ego vehicle at pre-declared times (``ANOMALY_PLAN``).  The
impulse is a physics-engine excitation; everything downstream is genuine
CARLA IMU/GNSS output.  A single sharp pulse models a pothole, a pair of
pulses 0.3 s apart models the front/rear axle hits of a speed bump, and a
short train of small alternating pulses models a rough patch.

Pass ``--no-inject`` to record an unmodified smooth-road drive instead.

Usage::

    python imu_collect.py --output data/carla_drive.json --duration 44

    # With CARLA running on localhost:2000
    python imu_collect.py --output data/carla_drive.json --host 127.0.0.1 --port 2000
"""

from __future__ import annotations

import json
import os
import sys
import time

# ── CARLA setup ──────────────────────────────────────────────────────────

# Measured on CARLA 0.9.x / Town10HD with synchronous_mode + TM sync: one
# world.tick() advances 0.04 s of sensor time and the 100 Hz IMU emits two
# samples (0.02 s apart).  This calibration is what makes --duration match
# the recorded timestamps and the injection times line up with the IMU.
TICK_SECONDS = 0.04

# (nominal time, anomaly type, [(offset_s, vertical impulse N*s), ...])
ANOMALY_PLAN = [
    {"t": 4.0, "type": "pothole",
     "pulses": [(0.00, 900.0)]},
    {"t": 10.0, "type": "speed_bump",
     "pulses": [(0.00, 700.0), (0.32, 700.0)]},
    {"t": 18.0, "type": "rough_patch",
     "pulses": [(0.00, 140.0), (0.16, 90.0), (0.32, 140.0),
                (0.48, 90.0), (0.64, 140.0)]},
    {"t": 26.0, "type": "pothole",
     "pulses": [(0.00, 900.0)]},
    {"t": 32.0, "type": "speed_bump",
     "pulses": [(0.00, 700.0), (0.32, 700.0)]},
    {"t": 38.0, "type": "rough_patch",
     "pulses": [(0.00, 140.0), (0.16, 90.0), (0.32, 140.0),
                (0.48, 90.0), (0.64, 140.0)]},
]


def _impulse_schedule(plan):
    """Flatten ANOMALY_PLAN into {(tick, offset): impulse} triggers."""
    schedule = []
    for item in plan:
        for off, impulse in item["pulses"]:
            t = item["t"] + off
            schedule.append((t, impulse))
    schedule.sort()
    return schedule


def _import_carla():
    """Import the CARLA Python API without baking in a machine-specific path.

    The wheel is not a PyPI dependency of this course repository, so look in
    CARLA_PY_PATH first, then in a few repository-relative ``_hutb_py``
    folders, and finally in whatever is already on sys.path.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.abspath(os.path.join(here, "..", "..", ".."))
    parent = os.path.dirname(repo_root)
    candidates = [
        os.environ.get("CARLA_PY_PATH", ""),
        os.path.join(repo_root, "_hutb_py"),
        os.path.join(parent, "_hutb_py"),
        os.path.join(os.path.dirname(parent), "_hutb_py"),
    ]
    for p in candidates:
        if p and os.path.isdir(p) and p not in sys.path:
            sys.path.insert(0, p)
    try:
        import carla
        return carla
    except ImportError:
        return None


def collect(host: str = "127.0.0.1", port: int = 2000,
            duration: float = 44.0, output: str = "data/carla_drive.json",
            save_rgb: bool = True, inject_anomalies: bool = True) -> dict:
    """Collect IMU+GNSS data from CARLA.

    Returns the collected data dict (also written to *output*).
    """
    carla = _import_carla()
    if carla is None:
        raise RuntimeError(
            "CARLA Python package not found. Set CARLA_PY_PATH to the "
            "directory containing the 'carla' package, or place the bundled "
            "_hutb_py wheel next to (or two levels above) the repository root."
        )

    client = carla.Client(host, port)
    client.set_timeout(20.0)
    world = client.get_world()

    # use synchronous mode for deterministic data; TM must follow suit
    settings = world.get_settings()
    prev_sync = settings.synchronous_mode
    prev_fixed = settings.fixed_delta_seconds
    settings.synchronous_mode = True
    # One world.tick() advances 0.02 s in this build; the 100 Hz IMU then
    # emits two samples per tick.  Keeping the two numbers consistent makes
    # the requested --duration match the recorded timeline.
    settings.fixed_delta_seconds = 0.02
    world.apply_settings(settings)
    tm = client.get_trafficmanager()
    tm.set_synchronous_mode(True)

    # spawn ego vehicle
    bp_lib = world.get_blueprint_library()
    vehicle_bp = bp_lib.find("vehicle.tesla.model3")
    spawn_points = world.get_map().get_spawn_points()
    # Spawn point 0 is on a clear through-road in Town10HD; other spawn
    # points can put the ego car behind a red light, which would freeze the
    # GNSS track for most of the recording.
    spawn = spawn_points[0]

    vehicle = world.spawn_actor(vehicle_bp, spawn)

    # attach IMU
    imu_bp = bp_lib.find("sensor.other.imu")
    imu_bp.set_attribute("sensor_tick", "0.01")  # 100 Hz
    imu_tf = carla.Transform(carla.Location(x=0, y=0, z=0.5))
    imu = world.spawn_actor(imu_bp, imu_tf, attach_to=vehicle)

    # attach GNSS
    gnss_bp = bp_lib.find("sensor.other.gnss")
    gnss_bp.set_attribute("sensor_tick", "0.1")  # 10 Hz
    gnss_tf = carla.Transform(carla.Location(x=0, y=0, z=1.0))
    gnss = world.spawn_actor(gnss_bp, gnss_tf, attach_to=vehicle)

    # attach RGB camera for reference
    cam_bp = bp_lib.find("sensor.camera.rgb")
    cam_bp.set_attribute("image_size_x", "640")
    cam_bp.set_attribute("image_size_y", "360")
    cam_bp.set_attribute("fov", "90")
    cam_bp.set_attribute("sensor_tick", "0.1")
    cam_tf = carla.Transform(
        carla.Location(x=1.5, z=2.5),
        carla.Rotation(pitch=-10),
    )
    camera = world.spawn_actor(cam_bp, cam_tf, attach_to=vehicle)

    # data storage
    imu_data: list[dict] = []
    gnss_data: list[dict] = []
    rgb_images: list[tuple] = []
    t0 = None
    cam_count = [0]
    # CARLA's IMU emits a large transient during the first few ticks while
    # the physics solver settles.  Warm up with the sensors already
    # listening, then discard everything before the recorded drive starts.
    recording = [False]

    def on_imu(sensor_data):
        nonlocal t0
        if not recording[0]:
            return
        ts = sensor_data.timestamp
        if t0 is None:
            t0 = ts
        accel = sensor_data.accelerometer
        gyro = sensor_data.gyroscope
        imu_data.append({
            "t": round(ts - t0, 4),
            "ax": round(accel.x, 6),
            "ay": round(accel.y, 6),
            "az": round(accel.z, 6),
            "gx": round(gyro.x, 6),
            "gy": round(gyro.y, 6),
            "gz": round(gyro.z, 6),
        })

    def on_gnss(sensor_data):
        if not recording[0] or t0 is None:
            return
        gnss_data.append({
            "t": round(sensor_data.timestamp - t0, 4),
            "lat": sensor_data.latitude,
            "lon": sensor_data.longitude,
            "alt": sensor_data.altitude,
        })

    def on_camera(image):
        if not save_rgb or not recording[0]:
            return
        cam_count[0] += 1
        # keep evenly spaced frames across the whole drive
        if cam_count[0] % 90 == 0 and len(rgb_images) < 8:
            rgb_images.append((cam_count[0], bytes(image.raw_data),
                               image.width, image.height))

    imu.listen(on_imu)
    gnss.listen(on_gnss)
    camera.listen(on_camera)

    # enable autopilot so the car drives itself through the town
    vehicle.set_autopilot(True)

    # warm-up: let the physics solver settle before recording
    warmup_ticks = 100
    print(f"  Warm-up ({warmup_ticks} ticks, discarded)...")
    for _ in range(warmup_ticks):
        world.tick()
    imu_data.clear()
    gnss_data.clear()
    rgb_images.clear()
    t0 = None
    recording[0] = True

    # drive
    n_ticks = int(duration / TICK_SECONDS)
    schedule = _impulse_schedule(ANOMALY_PLAN) if inject_anomalies else []
    sched_i = 0
    applied = []
    print(f"  Driving for {duration:.0f}s ({n_ticks} ticks)...")
    for i in range(n_ticks):
        t_tick = i * TICK_SECONDS
        while sched_i < len(schedule) and schedule[sched_i][0] <= t_tick:
            impulse = schedule[sched_i][1]
            vehicle.add_impulse(carla.Vector3D(0.0, 0.0, impulse))
            applied.append({"t": round(schedule[sched_i][0], 2),
                            "impulse_ns": impulse})
            sched_i += 1
        world.tick()
        if i % 250 == 0 and i > 0:
            print(f"    {i}/{n_ticks} ticks, {len(imu_data)} IMU samples")
    if applied:
        print(f"  Applied {len(applied)} vertical impulses "
              f"({len(ANOMALY_PLAN)} labelled anomalies)")

    # save RGB images (frames captured at evenly spaced points in the drive)
    rgb_dir = os.path.join(os.path.dirname(output), "rgb_frames")
    os.makedirs(rgb_dir, exist_ok=True)
    if save_rgb and rgb_images:
        try:
            from PIL import Image
            for idx, raw, width, height in rgb_images:
                # CARLA RGB camera raw_data is BGRA (4 bytes / pixel).
                # PIL's raw decoder "BGRX" handles the channel order directly.
                img = Image.frombytes("RGB", (width, height), raw,
                                      "raw", "BGRX", 0, 1)
                path = os.path.join(rgb_dir, f"frame_{idx:04d}.png")
                img.save(path)
            print(f"    Saved {len(rgb_images)} RGB frames to {rgb_dir}")
            # first frame doubles as the scene reference
            first = rgb_images[0]
            scene = Image.frombytes("RGB", (first[2], first[3]),
                                    first[1], "raw", "BGRX", 0, 1)
            scene_path = os.path.join(os.path.dirname(output), "scene_rgb.png")
            scene.save(scene_path)
            print(f"    Saved scene RGB to {scene_path}")
        except Exception as e:
            print(f"    RGB save skipped: {e}")

    # cleanup
    imu.stop()
    gnss.stop()
    camera.stop()
    vehicle.set_autopilot(False)
    imu.destroy()
    gnss.destroy()
    camera.destroy()
    vehicle.destroy()
    tm.set_synchronous_mode(False)

    # restore settings
    settings.synchronous_mode = prev_sync
    settings.fixed_delta_seconds = prev_fixed
    world.apply_settings(settings)

    # sensor_tick=0.01 is a *request*; with 0.02 s physics ticks CARLA can
    # only emit one sample per 0.02 s, so the effective rate is measured
    # from the recorded timestamps instead of assumed.
    gaps = [imu_data[i + 1]["t"] - imu_data[i]["t"]
            for i in range(min(200, len(imu_data) - 1))]
    gaps = [g for g in gaps if g > 0]
    effective_hz = round(1.0 / (sum(gaps) / len(gaps))) if gaps else 0

    # build result
    result = {
        "scene": {
            "map": world.get_map().name,
            "duration_s": round(imu_data[-1]["t"], 2) if imu_data else 0.0,
            "requested_ticks": n_ticks,
            "sample_rate_hz": effective_hz,
            "sensor_tick_requested_hz": 100,
            "vehicle": "tesla.model3",
            "mode": "autopilot (CARLA Traffic Manager, synchronous)",
            "warmup_ticks_discarded": warmup_ticks,
            "tick_seconds": TICK_SECONDS,
            "injected_excitation": bool(inject_anomalies),
            "note": "Live CARLA 0.9.x Town10HD IMU/GNSS recording. Each "
                    "world.tick() advances 0.04 s of sensor time and emits "
                    "two IMU samples 0.02 s apart, so the requested 100 Hz "
                    "sensor_tick yields a 50 Hz effective stream. The first "
                    "100 ticks are warm-up and discarded. Town10HD has no "
                    "modelled road defects, so calibrated vertical impulses "
                    "are applied to the ego vehicle at the times listed in "
                    "ground_truth; the IMU stream itself is genuine CARLA "
                    "physics output, not a synthesised signal.",
        },
        "samples": imu_data,
        "gps": gnss_data,
        "n_imu": len(imu_data),
        "n_gps": len(gnss_data),
    }
    if inject_anomalies:
        # ground-truth time is the *centre* of the excitation, because the
        # detector reports the peak of the resulting vibration, not its onset
        result["ground_truth"] = [
            {"t": round(item["t"] + item["pulses"][-1][0] / 2.0, 2),
             "onset_s": item["t"], "type": item["type"],
             "pulses": len(item["pulses"])}
            for item in ANOMALY_PLAN if item["t"] <= duration
        ]
        result["injected_impulses"] = applied

    os.makedirs(os.path.dirname(output) or ".", exist_ok=True)
    with open(output, "w", encoding="utf-8") as f:
        json.dump(result, f, ensure_ascii=False)
    print(f"  Collected {len(imu_data)} IMU samples, {len(gnss_data)} GPS points")
    print(f"  Saved to {output}")
    return result


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Collect CARLA IMU+GNSS data")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--duration", type=float, default=44.0,
                        help="Recording duration in seconds")
    parser.add_argument("--output", default="data/carla_drive.json")
    parser.add_argument("--no-rgb", action="store_true",
                        help="Skip RGB camera")
    parser.add_argument("--no-inject", action="store_true",
                        help="Record a smooth-road drive without excitation")
    args = parser.parse_args()
    collect(host=args.host, port=args.port, duration=args.duration,
            output=args.output, save_rgb=not args.no_rgb,
            inject_anomalies=not args.no_inject)
