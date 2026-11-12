# -*- coding: utf-8 -*-
"""Live OpenHUTB / CARLA data collection for the DVS event-camera module.

Only needed for ``python main.py --collect``. Requires a running CARLA
server and an importable ``carla`` Python package (set CARLA_PYTHON_PATH or
PYTHONPATH if the package is not installed in the current interpreter).

Scene: a stationary ego vehicle with a forward-facing DVS camera in CARLA
Town10HD. A textured street with trees produces a stationary background
event activity. After a static measurement phase, a target vehicle crosses
the field of view at a constant lateral speed. The simulator's own bounding
box is projected into the image to provide pixel-level ground truth.
"""

import math
import os
import queue
import time

from dvs_core import quantise, save_json, unpack_raw

SCENE = {
    "event_stride": 2,        # fixed subsampling ratio for every frame
    "width": 320,
    "height": 180,
    "fov_deg": 90.0,
    "sensor_tick": 0.05,
    "threshold": 0.6,
    "warmup_frames": 25,
    "static_frames": 16,
    "moving_frames": 40,
    "target_depth_m": 18.0,
    "target_lateral_start_m": -7.0,
    "target_speed_mps": 6.0,
}


def import_carla():
    """Import carla, honouring an explicit CARLA_PYTHON_PATH override."""
    extra = os.environ.get("CARLA_PYTHON_PATH", "")
    if extra and extra not in __import__("sys").path:
        __import__("sys").path.insert(0, extra)
    try:
        import carla
        return carla
    except ImportError as exc:
        raise RuntimeError(
            "Cannot import carla. Set CARLA_PYTHON_PATH to the folder that "
            "contains the carla package (CARLA PythonAPI/carla/dist egg or "
            "the OpenHUTB _hutb_py directory). Original error: %s" % exc)


def make_sensor(bp_library, sensor_id, cfg):
    bp = bp_library.find(sensor_id)
    attrs = {
        "image_size_x": str(cfg["width"]),
        "image_size_y": str(cfg["height"]),
        "fov": str(cfg["fov_deg"]),
        "sensor_tick": str(cfg["sensor_tick"]),
        "enable_postprocess_effects": "false",
        "exposure_mode": "manual",
        "motion_blur_intensity": "0",
        "motion_blur_max_distortion": "0",
        "lens_flare_intensity": "0",
        "bloom_intensity": "0",
        "chromatic_aberration_intensity": "0",
    }
    if sensor_id == "sensor.camera.dvs":
        attrs.update({
            "positive_threshold": str(cfg["threshold"]),
            "negative_threshold": str(cfg["threshold"]),
            "sigma_positive_threshold": "0",
            "sigma_negative_threshold": "0",
            "use_log": "true",
            "log_eps": "0.001",
        })
    for key, val in attrs.items():
        if bp.has_attribute(key):
            bp.set_attribute(key, val)
    return bp


def collect(host="127.0.0.1", port=2000, out_json="dvs_events.json",
            out_rgb=None, cfg=None, verbose=True):
    """Run the scenario and write a replayable event dataset."""
    cfg = dict(SCENE if cfg is None else {**SCENE, **cfg})
    carla = import_carla()
    client = carla.Client(host, port)
    client.set_timeout(30.0)
    world = client.get_world()
    bp_lib = world.get_blueprint_library()
    if verbose:
        print("  simulator map :", world.get_map().name)

    settings = world.get_settings()
    old = (settings.synchronous_mode, settings.fixed_delta_seconds)
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = cfg["sensor_tick"]
    world.apply_settings(settings)

    actors = []

    def cleanup():
        for a in reversed(actors):
            try:
                a.destroy()
            except Exception:
                pass
        actors.clear()
        for _ in range(3):
            world.tick()

    try:
        # remove leftover vehicles/sensors from earlier runs
        for a in list(world.get_actors()):
            if a.type_id.startswith("vehicle.") or a.type_id.startswith("sensor."):
                try:
                    a.destroy()
                except Exception:
                    pass
        cleanup()
        time.sleep(0.8)

        ego = None
        spawn = None
        for cand in world.get_map().get_spawn_points()[:16]:
            ego = world.try_spawn_actor(
                bp_lib.filter("vehicle.tesla.model3")[0], cand)
            if ego is not None:
                spawn = cand
                break
        if ego is None:
            raise RuntimeError("No free spawn point in the current map")
        actors.append(ego)
        fwd = spawn.get_forward_vector()
        right = spawn.get_right_vector()

        tf = carla.Transform(carla.Location(x=1.6, z=1.7))
        dvs = world.spawn_actor(
            make_sensor(bp_lib, "sensor.camera.dvs", cfg), tf, attach_to=ego)
        # the RGB camera is only used for the scene screenshot, so it can
        # render at twice the DVS resolution without changing the event data
        rgb_bp = make_sensor(bp_lib, "sensor.camera.rgb", cfg)
        for key, val in (("image_size_x", str(cfg["width"] * 2)),
                         ("image_size_y", str(cfg["height"] * 2))):
            if rgb_bp.has_attribute(key):
                rgb_bp.set_attribute(key, val)
        rgb = world.spawn_actor(rgb_bp, tf, attach_to=ego)
        actors.extend([dvs, rgb])
        qd, qr = queue.Queue(), queue.Queue()
        dvs.listen(lambda d: qd.put(d))
        rgb.listen(lambda d: qr.put(d))

        def tick(n, keep_rgb=False):
            last_d = last_r = None
            for _ in range(n):
                world.tick()
                try:
                    last_d = qd.get(timeout=3.0)
                except queue.Empty:
                    pass
                try:
                    last_r = qr.get(timeout=3.0)
                except queue.Empty:
                    pass
            return last_d, (last_r if keep_rgb else None)

        def project(target):
            """Project the target's bounding-box centre into the camera."""
            cam_tf = dvs.get_transform()
            cam_loc = cam_tf.location
            f = cam_tf.rotation.get_forward_vector()
            r = cam_tf.rotation.get_right_vector()
            u = cam_tf.rotation.get_up_vector()
            box = target.bounding_box.location
            t_tf = target.get_transform()
            try:
                centre = t_tf.transform(box)
            except Exception:
                centre = carla.Location(
                    t_tf.location.x + box.x, t_tf.location.y + box.y,
                    t_tf.location.z + box.z)
            vec = centre - cam_loc
            px = vec.x * f.x + vec.y * f.y + vec.z * f.z
            py = vec.x * r.x + vec.y * r.y + vec.z * r.z
            pz = vec.x * u.x + vec.y * u.y + vec.z * u.z
            if px <= 0.1:
                return None, None
            focal = (cfg["width"] / 2.0) / math.tan(
                math.radians(cfg["fov_deg"]) / 2.0)
            return (round(cfg["width"] / 2.0 + focal * py / px, 2),
                    round(cfg["height"] / 2.0 - focal * pz / px, 2))

        # warm-up lets the auto-exposure / renderer settle; these frames are
        # deliberately discarded because the first DVS frames are dominated
        # by exposure adaptation rather than real brightness changes.
        tick(cfg["warmup_frames"])

        frames = []
        rgb_saved = False

        def record(d, phase, index, target=None):
            nonlocal rgb_saved
            events = unpack_raw(d.raw_data) if d is not None else []
            compact, stride = quantise(events, cfg["event_stride"])
            gt_u, gt_v = (None, None) if target is None else project(target)
            frames.append({
                "i": index,
                "t": round(index * cfg["sensor_tick"], 3),
                "phase": phase,
                "gt_u": gt_u, "gt_v": gt_v,
                "n": len(events), "keep": stride, "e": compact,
            })

        for k in range(cfg["static_frames"]):
            d, _r = tick(1)
            record(d, "static", len(frames))

        target = world.spawn_actor(
            bp_lib.filter("vehicle.audi.tt")[0],
            carla.Transform(
                spawn.location
                + fwd * cfg["target_depth_m"]
                + right * cfg["target_lateral_start_m"]
                + carla.Location(z=0.3),
                spawn.rotation))
        actors.append(target)
        target.enable_constant_velocity(carla.Vector3D(
            right.x * cfg["target_speed_mps"],
            right.y * cfg["target_speed_mps"], 0))
        for k in range(cfg["moving_frames"]):
            mid = (k == cfg["moving_frames"] // 2)
            d, r = tick(1, keep_rgb=mid)
            if mid and r is not None and out_rgb and not rgb_saved:
                r.save_to_disk(out_rgb)
                rgb_saved = True
            record(d, "moving", len(frames), target)

        meta = {
            "simulator": "OpenHUTB v2.10.0 / CARLA Town10HD",
            "sensor": "sensor.camera.dvs",
            "source": "live simulation",
            "width": cfg["width"], "height": cfg["height"],
            "fov_deg": cfg["fov_deg"],
            "sensor_tick": cfg["sensor_tick"],
            "dvs_threshold": cfg["threshold"],
            "static_frames": cfg["static_frames"],
            "moving_frames": cfg["moving_frames"],
            "target_depth_m": cfg["target_depth_m"],
            "target_lateral_start_m": cfg["target_lateral_start_m"],
            "target_speed_mps": cfg["target_speed_mps"],
            "event_stride": cfg["event_stride"],
            "rgb_screenshot_size": [cfg["width"] * 2, cfg["height"] * 2],
            "event_format": "[x, y, pol, dt8]; dt8 = timestamp quantized to 0..255 inside one tick",
            "note": "every frame uses the same fixed stride subsampling; field keep records it",
        }
        save_json({"meta": meta, "frames": frames}, out_json, compact=True)
        if verbose:
            print("  frames         :", len(frames),
                  "(static %d / moving %d)" %
                  (cfg["static_frames"], cfg["moving_frames"]))
            print("  raw events     :", sum(f["n"] for f in frames))
            print("  dataset        :", out_json)
            if rgb_saved:
                print("  RGB screenshot :", out_rgb)
        return meta, frames
    finally:
        cleanup()
        settings.synchronous_mode, settings.fixed_delta_seconds = old
        world.apply_settings(settings)
