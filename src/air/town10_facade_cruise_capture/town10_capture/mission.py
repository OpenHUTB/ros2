"""共享任务状态机、神经控制与可追溯采集记录。"""

import csv
import json
import math
import time
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

from .config import CruiseConfig, town10_route
from .learning import DenseController, control_features
from .route import scan_route
from .scene import MujocoBackend, ray_features


def native_hybrid_velocity(neural_velocity, error, velocity):
    """有界网络前馈叠加三轴PD，前馈随误差收敛到零。"""
    error = np.asarray(error, dtype=float)
    velocity = np.asarray(velocity, dtype=float)
    neural_velocity = np.asarray(neural_velocity, dtype=float)
    if any(
        value.shape != (3,) or not np.isfinite(value).all()
        for value in (error, velocity, neural_velocity)
    ):
        raise ValueError("原生控制输入须为三个有限数")
    weight = 0.2 * np.clip(np.abs(error) / 0.5, 0.0, 1.0)
    command = np.clip(
        weight * np.clip(neural_velocity, -1.0, 1.0) + 1.1 * error - 0.5 * velocity,
        -1.0,
        1.0,
    )
    command[np.abs(error) < 0.035] = 0.0
    return command


def waypoint_reached(error, velocity, backend):
    """原生飞控与确定性平移模型分别使用稳定到达阈值。"""
    position_limit, speed_limit = (0.3, 0.35) if backend == "airsim" else (0.15, 0.22)
    return (
        np.linalg.norm(error) < position_limit
        and np.linalg.norm(velocity) < speed_limit
    )


class CruiseMission:
    def __init__(self, config: CruiseConfig, on_sample=None, backend=None):
        config.validate()
        self.config, self.on_sample = config, on_sample
        self.output = Path(config.output)
        self.output.mkdir(parents=True, exist_ok=True)
        self.model = DenseController.load(
            config.model or Path(__file__).parent / "models/facade_controller.json"
        )
        if backend is not None:
            self.backend = backend
        elif config.backend == "mujoco":
            self.backend = MujocoBackend(config.style, config.distance)
        else:
            from .airsim_backend import AirSimBackend

            self.backend = AirSimBackend(config)
        self.dt = 1 / config.rate
        position, _, _ = self.backend.state()
        self.basis = np.eye(3)
        if config.backend == "mujoco":
            self.route = scan_route(distance=config.distance)
        else:
            # 局部x沿墙向右、y为相机朝向、z向上；默认不前冲靠墙。
            self.basis = np.column_stack(
                (self.backend.right, self.backend.forward, [0.0, 0.0, 1.0])
            )
            if config.route == "town10":
                absolute = town10_route(config.route_offset)
                self.route = absolute[:, :3]
                self.backend.yaw = float(absolute[0, 3])
                yaw = math.radians(self.backend.yaw)
                from .config import ned_to_enu

                self.basis = np.column_stack(
                    (
                        ned_to_enu([-math.sin(yaw), math.cos(yaw), 0]),
                        ned_to_enu([math.cos(yaw), math.sin(yaw), 0]),
                        [0.0, 0.0, 1.0],
                    )
                )
            else:
                relative = scan_route(distance=config.distance) - np.array(
                    [-3.0, -config.distance, 1.5]
                )
                self.route = position + relative @ self.basis.T
        self.start = time.monotonic()
        self.elapsed = 0.0
        self.waypoint = 0
        self.waypoint_since = 0.0
        self.rows = []
        self.captures = []
        self.frames = []
        self.last_frame_time = -math.inf
        self.finished = False
        self.reason = "running"
        self.metrics = {}

    def tick(self):
        if self.finished:
            return False
        try:
            self.elapsed = (
                time.monotonic() - self.start
                if self.config.backend == "airsim"
                else len(self.rows) * self.dt
            )
            if self.elapsed >= self.config.duration:
                return self.finish("duration_limit")
            if self.elapsed - self.waypoint_since > self.config.waypoint_timeout:
                return self.finish("waypoint_timeout")
            position, velocity, orientation = self.backend.state()
            if not np.isfinite(np.concatenate((position, velocity))).all():
                raise RuntimeError("位置或速度出现非有限数值")
            if self.config.backend == "airsim":
                self.backend.check_collision()
                ranges = self.backend.ranges()
                distance = self.backend.depth_statistics["front_depth_median_m"]
                if self.backend.depth_statistics["front_depth_min_m"] < 1.2:
                    return self.finish("obstacle_stop")
            else:
                ranges = self.backend.ranges()
                values = ray_features(ranges)
                valid = values[np.isfinite(values)]
                if not valid.size:
                    raise RuntimeError("墙面测距无有效回波")
                distance = float(np.median(valid))
                if distance < 0.6:
                    raise RuntimeError("与墙面距离小于0.6m")
            target = self.route[self.waypoint]
            error = (target - position) @ self.basis
            local_velocity = velocity @ self.basis
            features = control_features(
                error, ranges, local_velocity, self.config.distance
            )
            if self.config.backend == "airsim":
                # 原生任务按实际航点保持前后位置，真实九点深度仍参与神经网络输入。
                features[9:12] = error
            local_command = self.model.predict(features)
            if self.config.backend == "airsim":
                local_command = native_hybrid_velocity(
                    local_command, error, local_velocity
                )
            # NN误差接近零时显式停稳，避免小偏置积累。
            local_command[np.abs(error) < 0.035] = 0.0
            command = local_command @ self.basis.T
            norm = np.linalg.norm(command)
            command *= min(1.0, self.config.speed / max(norm, 1e-12))
            if self.config.backend == "airsim":
                command *= np.clip(
                    (self.backend.depth_statistics["front_depth_min_m"] - 1.2) / 1.8,
                    0.0,
                    1.0,
                )
            sample = {
                "time_s": self.elapsed,
                "position": position.copy(),
                "velocity": velocity.copy(),
                "orientation_xyzw": orientation.copy(),
                "command": command.copy(),
                "waypoint": self.waypoint,
                "distance_m": distance,
                "image": None,
            }
            self.rows.append(
                [
                    self.elapsed,
                    *position,
                    *velocity,
                    *command,
                    distance,
                    self.waypoint,
                    getattr(self.backend, "depth_statistics", {}).get(
                        "front_depth_min_m", distance
                    ),
                    float(np.linalg.norm(target - position)),
                ]
            )
            arrived = waypoint_reached(target - position, velocity, self.config.backend)
            periodic = (
                self.elapsed - self.last_frame_time
                >= 1 / self.config.capture_rate - 1e-8
            )
            if periodic or arrived:
                pixels = (
                    self.backend.image(force=arrived)
                    if self.config.backend == "airsim"
                    else self.backend.image()
                )
                sample["image"] = pixels
                self.last_frame_time = self.elapsed
                if self.config.gif and periodic:
                    front = Image.fromarray(pixels).resize((320, 240))
                    if self.config.backend == "mujoco":
                        overview = Image.fromarray(self.backend.image("overview"))
                        frame = Image.new("RGB", (640, 264), "#142522")
                        frame.paste(overview, (0, 0))
                        frame.paste(front, (320, 0))
                    else:
                        frame = Image.new("RGB", (320, 264), "#142522")
                        frame.paste(front, (0, 0))
                    ImageDraw.Draw(frame).text(
                        (8, 245),
                        f"{self.config.backend}  t={self.elapsed:5.1f}s  waypoint={self.waypoint + 1}/{len(self.route)}",
                        fill="white",
                    )
                    self.frames.append(frame)
                if arrived:
                    filename = f"capture_{self.waypoint:03d}.png"
                    Image.fromarray(pixels).save(self.output / filename)
                    self.captures.append(
                        {
                            "file": filename,
                            "waypoint": self.waypoint,
                            "time_s": self.elapsed,
                            "position_enu_m": position.tolist(),
                            "orientation_enu_flu_xyzw": orientation.tolist(),
                            "target_enu_m": target.tolist(),
                            "position_error_m": float(
                                np.linalg.norm(target - position)
                            ),
                            **getattr(self.backend, "last_image_metadata", {}),
                        }
                    )
            if self.on_sample:
                self.on_sample(sample)
            if arrived:
                self.waypoint += 1
                self.waypoint_since = self.elapsed
                if self.waypoint == len(self.route):
                    return self.finish("complete")
            self.backend.step(command, self.dt)
            return True
        except BaseException as exc:
            self.finish(
                "interrupted" if isinstance(exc, KeyboardInterrupt) else "error",
                str(exc),
            )
            raise

    def finish(self, reason="interrupted", error=None):
        if self.finished:
            return False
        self.finished = True
        self.reason = reason
        try:
            if self.config.backend == "mujoco":
                Image.fromarray(self.backend.image("overview")).save(
                    self.output / "overview.png"
                )
        finally:
            try:
                self.backend.close()
            finally:
                with (self.output / "trajectory.csv").open(
                    "w", newline="", encoding="utf-8"
                ) as stream:
                    writer = csv.writer(stream)
                    writer.writerow(
                        [
                            "time_s",
                            "x_m",
                            "y_m",
                            "z_m",
                            "vx_m_s",
                            "vy_m_s",
                            "vz_m_s",
                            "cmd_x_m_s",
                            "cmd_y_m_s",
                            "cmd_z_m_s",
                            "distance_m",
                            "waypoint",
                            "front_depth_min_m",
                            "route_error_m",
                        ]
                    )
                    writer.writerows(self.rows)
                records = np.asarray(self.rows)
                self.metrics = {
                    "backend": self.config.backend,
                    "scene": self.config.style
                    if self.config.backend == "mujoco"
                    else "server_scene",
                    "status": reason,
                    "controller": "numpy_mlp_15_32_3"
                    if self.config.backend == "mujoco"
                    else "bounded_mlp_feedforward_pd",
                    "waypoints_total": len(self.route),
                    "waypoints_completed": self.waypoint,
                    "images_captured": len(self.captures),
                    "duration_s": self.elapsed,
                    "wall_clock_s": time.monotonic() - self.start,
                    "distance_source": "mujoco_nine_ray"
                    if self.config.backend == "mujoco"
                    else "airsim_depth_planar_nine_samples",
                    "maximum_speed_m_s": float(
                        np.max(np.linalg.norm(records[:, 4:7], axis=1))
                    )
                    if len(records)
                    else None,
                    "error": error,
                    "arrival_position_tolerance_m": 0.3
                    if self.config.backend == "airsim"
                    else 0.15,
                    "arrival_speed_tolerance_m_s": 0.35
                    if self.config.backend == "airsim"
                    else 0.22,
                }
                if self.config.backend == "mujoco":
                    self.metrics["reference_distance_rmse_m"] = (
                        float(
                            np.sqrt(
                                np.mean((records[:, 10] - self.config.distance) ** 2)
                            )
                        )
                        if len(records)
                        else None
                    )
                else:
                    self.metrics.update(
                        {
                            "front_depth_median_mean_m": float(np.mean(records[:, 10]))
                            if len(records)
                            else None,
                            "front_depth_min_m": float(np.min(records[:, 12]))
                            if len(records)
                            else None,
                            "route_error_rmse_m": float(
                                np.sqrt(np.mean(records[:, 13] ** 2))
                            )
                            if len(records)
                            else None,
                            "route_error_max_m": float(np.max(records[:, 13]))
                            if len(records)
                            else None,
                            "capture_position_error_max_m": max(
                                (
                                    capture["position_error_m"]
                                    for capture in self.captures
                                ),
                                default=None,
                            ),
                        }
                    )
                (self.output / "metrics.json").write_text(
                    json.dumps(
                        self.metrics, indent=2, ensure_ascii=False, allow_nan=False
                    ),
                    encoding="utf-8",
                )
                (self.output / "manifest.json").write_text(
                    json.dumps(
                        {
                            "backend": self.config.backend,
                            "coordinate_frame": "ENU world, FLU body",
                            "scene_scope": "Town10 RGB crops on a planar proxy; not Town10 city geometry"
                            if self.config.backend == "mujoco"
                            else "native AirSim server geometry",
                            "route_enu_m": self.route.tolist(),
                            "captures": self.captures,
                        },
                        indent=2,
                        ensure_ascii=False,
                        allow_nan=False,
                    ),
                    encoding="utf-8",
                )
                if self.frames:
                    self.frames[0].save(
                        self.output / "cruise.gif",
                        save_all=True,
                        append_images=self.frames[1:],
                        duration=round(1000 / self.config.capture_rate),
                        loop=0,
                        optimize=True,
                    )
        return False


def run_cruise(config, on_sample=None, backend=None):
    mission = CruiseMission(config, on_sample, backend)
    try:
        next_tick = time.monotonic()
        while mission.tick():
            if config.backend == "airsim":
                next_tick += mission.dt
                now = time.monotonic()
                if next_tick < now - mission.dt:
                    next_tick = now
                time.sleep(max(0.0, next_tick - now))
    finally:
        if not mission.finished:
            mission.finish()
    return mission.metrics
