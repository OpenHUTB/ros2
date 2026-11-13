"""Kinematic-bicycle road-vehicle simulator with lidar, GNSS, IMU and camera.

The vehicle model is the single-track (bicycle) model with an understeer
gradient:

    dv/dt = a - k_d v^2 - k_r sign(v) - offroad_term
    dpsi/dt = v / L * tan(delta) / (1 + K v^2)
    dx/dt = v cos(psi),  dy/dt = v sin(psi)

Collisions, curbs, parked cars, cones, pedestrians and traffic lights are part
of the scene model, so the simulator is a physically meaningful 2-D vehicle
simulation that runs at several hundred Hz on a CPU-only VM.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from . import config as C
from .geometry import wrap_angle
from .scenes import Scene, get_scene


class RoadSim:
    """A car with a 2-D bicycle model inside a road-network scene."""

    def __init__(self, scene_name: str = "downtown", seed: int = C.DEFAULT_SEED,
                 noise: bool = True):
        self.scene: Scene = get_scene(scene_name)
        self.rng = np.random.default_rng(seed)
        self.noise = bool(noise)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = 0.0
        self.delta = 0.0

        # sensor noise
        self.lidar_sigma = 0.03 if self.noise else 0.0
        self.lidar_outlier_p = 0.04 if self.noise else 0.0
        self.lidar_dropout_p = 0.03 if self.noise else 0.0
        self.gnss_sigma = C.GNSS_SIGMA if self.noise else 0.0
        self.imu_sigma = C.IMU_SIGMA if self.noise else 0.0
        self.odom_scale = 1.0 + (0.02 * self.rng.normal() if self.noise else 0.0)
        self.odom_yaw_bias = (0.003 * self.rng.normal() if self.noise else 0.0)
        self.gnss_bias = (self.gnss_sigma * 2.0 * self.rng.normal(size=2)
                          if self.noise else np.zeros(2))

        self._angles = np.linspace(-math.pi, math.pi, C.LIDAR_BEAMS, endpoint=False)
        self._cam_angles = np.linspace(-C.CAM_FOV / 2, C.CAM_FOV / 2, C.CAM_WIDTH // 2)

        self.time = 0.0
        self.collisions = 0
        self.red_light_violations = 0
        self.pedestrians_hit = 0
        self.off_road_time = 0.0
        self.cmd = np.zeros(2)
        self._cooldown = 0.0
        self._contact = False
        self._ped_contact: set = set()
        self._passed_lights: Dict[int, float] = {}
        self._prev_light_dist: Dict[int, float] = {}
        self.reset()

    # ------------------------------------------------------------------ reset
    def reset(self, pose: Optional[Sequence[float]] = None) -> None:
        if pose is None:
            x, y, _z, yaw = self.scene.start
        else:
            vals = [float(v) for v in pose]
            x, y, yaw = vals[0], vals[1], vals[3]
        self.x, self.y, self.yaw = x, y, yaw
        self.v = 0.0
        self.delta = 0.0
        self.odom = np.array([x, y, 0.0, yaw], dtype=np.float64)
        self.time = 0.0
        self.collisions = 0
        self.red_light_violations = 0
        self.pedestrians_hit = 0
        self.off_road_time = 0.0
        self._cooldown = 0.0
        self._contact = False
        self._ped_contact: set = set()
        self._passed_lights.clear()
        self._prev_light_dist.clear()
        self.cmd[:] = 0.0

    # ------------------------------------------------------------------- state
    @property
    def pose(self) -> np.ndarray:
        return np.array([self.x, self.y, 0.0, self.yaw], dtype=np.float64)

    @property
    def speed(self) -> float:
        return float(self.v)

    @property
    def steering(self) -> float:
        return float(self.delta)

    # -------------------------------------------------------------------- step
    def step(self, cmd: Sequence[float], dt: float = 1.0 / C.CONTROL_HZ) -> None:
        """Advance ``dt`` seconds with ``cmd = (throttle [-1,1], steer [-1,1])``."""
        cmd = np.asarray(cmd, dtype=np.float64).ravel()
        if cmd.size < 2:
            cmd = np.array([cmd[0] if cmd.size else 0.0, 0.0])
        throttle = float(np.clip(cmd[0], -1.0, 1.0))
        steer = float(np.clip(cmd[1], -1.0, 1.0))
        self.cmd = np.array([throttle, steer])
        n_sub = max(1, int(round(dt / C.PHYS_DT)))
        for _ in range(n_sub):
            self._physics_step(throttle, steer, C.PHYS_DT)
        self.time += n_sub * C.PHYS_DT

    def _physics_step(self, throttle: float, steer: float, dt: float) -> None:
        # steering actuator: rate-limited response
        target_delta = steer * C.MAX_STEER
        step = C.STEER_RATE * dt
        self.delta += float(np.clip(target_delta - self.delta, -step, step))

        # longitudinal dynamics
        if throttle >= 0.0:
            accel = throttle * C.MAX_ACCEL
        elif self.v > 0.4:
            accel = throttle * C.MAX_BRAKE          # braking while moving forward
        else:
            accel = throttle * C.MAX_ACCEL          # reverse
        drag = C.DRAG * self.v * abs(self.v) + C.ROLLING * (1.0 if self.v > 0.0 else -1.0)
        offroad = 0.0
        if not self.scene.is_drivable(self.x, self.y):
            offroad = C.OFFROAD_DRAG
            self.off_road_time += dt
        v_new = self.v + (accel - drag) * dt - offroad * dt * (1.0 if self.v > 0 else -1.0)
        if throttle <= 0.0 and abs(v_new) < 0.25:
            v_new = 0.0
        self.v = float(np.clip(v_new, -C.MAX_REVERSE, C.MAX_SPEED))

        # bicycle model with understeer
        yaw_rate = (self.v / C.WHEELBASE) * math.tan(self.delta) / (
            1.0 + C.UNDERSTEER * self.v * self.v)
        self.yaw = wrap_angle(self.yaw + yaw_rate * dt)
        nx = self.x + self.v * math.cos(self.yaw) * dt
        ny = self.y + self.v * math.sin(self.yaw) * dt

        # ---- collision with parked cars / cones / barriers / pedestrians
        hit = self._check_collision(nx, ny)
        if hit is not None:
            kind, index = hit
            if kind == "pedestrian":
                if index not in self._ped_contact:
                    self.pedestrians_hit += 1
                    self._ped_contact.add(index)
            if not self._contact:
                self.collisions += 1
                self._contact = True
            self.v = -0.15 * self.v
            nx, ny = self.x, self.y
        if hit is None or hit[0] != "pedestrian":
            self._ped_contact.clear()
        if hit is None:
            self._contact = False
        self._cooldown = max(0.0, self._cooldown - dt)

        # ---- red-light running detection at approach stop lines
        self._update_lights(nx, ny, dt)

        self.x, self.y = nx, ny

    def _check_collision(self, x: float, y: float) -> Optional[Tuple[str, int]]:
        r = C.CAR_RADIUS
        for i, (cx, cy, hw, hh) in enumerate(self.scene.parked):
            dx = max(abs(x - cx) - hw, 0.0)
            dy = max(abs(y - cy) - hh, 0.0)
            if dx * dx + dy * dy <= r * r:
                return ("parked", i)
        for i, (cx, cy, hw, hh) in enumerate(self.scene.barriers):
            dx = max(abs(x - cx) - hw, 0.0)
            dy = max(abs(y - cy) - hh, 0.0)
            if dx * dx + dy * dy <= r * r:
                return ("barrier", i)
        for i, (cx, cy, cr) in enumerate(self.scene.cones):
            if (x - cx) ** 2 + (y - cy) ** 2 <= (r + cr) ** 2:
                return ("cone", i)
        for i, (px, py) in enumerate(self.scene.pedestrian_positions(self.time)):
            if (x - px) ** 2 + (y - py) ** 2 <= (r + 0.45) ** 2:
                return ("pedestrian", i)
        return None

    def _update_lights(self, nx: float, ny: float, dt: float) -> None:
        if not self.scene.lights:
            return
        for idx, light in enumerate(self.scene.lights):
            lx, ly = float(light["x"]), float(light["y"])
            axis = str(light.get("axis", "x"))
            direction = float(light.get("dir", 1.0))
            if axis == "x":
                if abs(ny - ly) > 3.5:
                    continue
                dist = (lx - nx) * direction - 5.0
            else:
                if abs(nx - lx) > 3.5:
                    continue
                dist = (ly - ny) * direction - 5.0
            prev = self._prev_light_dist.get(idx, dist)
            state = self.scene.light_state(light, self.time)
            if prev > 0.0 >= dist and state == "red":
                if self.time - self._passed_lights.get(idx, -99.0) > 4.0:
                    self.red_light_violations += 1
                    self._passed_lights[idx] = self.time
            self._prev_light_dist[idx] = dist

    # ----------------------------------------------------------------- sensors
    def lidar(self) -> np.ndarray:
        """360 deg lidar (72 beams); curbs, cars, cones, barriers, pedestrians."""
        pose = self.pose
        ranges = np.empty(C.LIDAR_BEAMS, dtype=np.float64)
        rects = self.scene.obstacle_rects()
        cones = list(self.scene.cones)
        peds = [(px, py, 0.45) for px, py in self.scene.pedestrian_positions(self.time)]
        step = 0.20
        for i, rel in enumerate(self._angles):
            ang = pose[3] + rel
            dx, dy = math.cos(ang), math.sin(ang)
            hit = C.LIDAR_RANGE_MAX
            dist = 0.2
            while dist < C.LIDAR_RANGE_MAX:
                px, py = pose[0] + dx * dist, pose[1] + dy * dist
                if not self.scene.is_drivable(px, py):
                    hit = dist
                    break
                if self._point_in_obstacles(px, py, rects, cones, peds):
                    hit = dist
                    break
                dist += step
            ranges[i] = min(C.LIDAR_RANGE_MAX, hit)
        if self.noise:
            ranges = ranges * (1.0 + 0.01 * self.rng.normal(size=ranges.shape))
            ranges += self.lidar_sigma * self.rng.normal(size=ranges.shape)
            outlier = self.rng.random(ranges.shape) < self.lidar_outlier_p
            ranges[outlier] = self.rng.uniform(2.0, C.LIDAR_RANGE_MAX, size=outlier.sum())
            dropout = self.rng.random(ranges.shape) < self.lidar_dropout_p
            ranges[dropout] = C.LIDAR_RANGE_MAX
        return np.clip(ranges, C.LIDAR_RANGE_MIN, C.LIDAR_RANGE_MAX)

    @staticmethod
    def _point_in_obstacles(px: float, py: float,
                            rects: Sequence[Sequence[float]],
                            cones: Sequence[Sequence[float]],
                            peds: Sequence[Sequence[float]]) -> bool:
        for cx, cy, hw, hh in rects:
            if abs(px - cx) <= hw and abs(py - cy) <= hh:
                return True
        for cx, cy, r in cones:
            if (px - cx) ** 2 + (py - cy) ** 2 <= r * r:
                return True
        for cx, cy, r in peds:
            if (px - cx) ** 2 + (py - cy) ** 2 <= r * r:
                return True
        return False

    def lidar_beam_angles(self) -> np.ndarray:
        return self._angles.copy()

    # Backwards-compatible aliases for the shared SLAM / planner code.
    def sonar(self) -> np.ndarray:
        return self.lidar()

    def sonar_beam_angles(self) -> np.ndarray:
        return self.lidar_beam_angles()

    def altitude(self) -> float:
        return 0.0

    def gnss(self) -> np.ndarray:
        pos = np.array([self.x, self.y], dtype=np.float64)
        if self.noise:
            pos = pos + self.gnss_bias + self.gnss_sigma * self.rng.normal(size=2)
        return pos

    def imu(self) -> Tuple[float, float]:
        yaw_rate = (self.v / C.WHEELBASE) * math.tan(self.delta) / (
            1.0 + C.UNDERSTEER * self.v * self.v)
        if self.noise:
            return wrap_angle(self.yaw + self.imu_sigma * self.rng.normal()), \
                float(yaw_rate + 0.004 * self.rng.normal())
        return float(self.yaw), float(yaw_rate)

    def integrate_odometry(self, dt: float) -> np.ndarray:
        yaw = self.yaw
        self.odom[0] += self.odom_scale * self.v * math.cos(yaw) * dt
        self.odom[1] += self.odom_scale * self.v * math.sin(yaw) * dt
        self.odom[3] = wrap_angle(0.6 * (self.odom[3] + self.odom_yaw_bias * dt) + 0.4 * yaw)
        if self.noise:
            gnss = self.gnss()
            self.odom[0:2] = 0.98 * self.odom[0:2] + 0.02 * gnss
        return self.odom.copy()

    def traffic_light(self) -> Optional[Dict[str, float]]:
        return self.scene.nearest_light(self.x, self.y, self.yaw, self.time)

    def camera(self) -> np.ndarray:
        """Forward RGB camera (48x64) rendered from the road scene geometry."""
        img = np.zeros((C.CAM_HEIGHT, C.CAM_WIDTH, 3), dtype=np.uint8)
        horizon = int(C.CAM_HEIGHT * 0.42)
        for row in range(horizon):
            f = row / max(1, horizon)
            img[row, :, :] = np.array([150 - 40 * f, 178 - 30 * f, 205 - 20 * f],
                                      dtype=np.uint8)
        for row in range(horizon, C.CAM_HEIGHT):
            f = (row - horizon) / max(1, C.CAM_HEIGHT - horizon)
            img[row, :, :] = np.array([60 + 30 * f, 62 + 30 * f, 66 + 30 * f],
                                      dtype=np.uint8)
        # lane markings: two dashed lines ahead
        for col in (C.CAM_WIDTH // 2 - 9, C.CAM_WIDTH // 2 + 9):
            for row in range(horizon, C.CAM_HEIGHT):
                if ((row + int(self.time * 8)) // 3) % 2 == 0:
                    img[row, col:col + 2, :] = np.array([225, 220, 195], dtype=np.uint8)
        # buildings / parked cars on the sides
        left, right = self.lidar()[: C.LIDAR_BEAMS // 2], self.lidar()[C.LIDAR_BEAMS // 2:]
        side_left = float(np.min(left[(np.abs(self._angles[: C.LIDAR_BEAMS // 2] + 1.2) < 0.8)])) if left.size else 40.0
        side_right = float(np.min(right[(np.abs(self._angles[C.LIDAR_BEAMS // 2:] - 1.2) < 0.8)])) if right.size else 40.0
        if side_left < 12:
            w = int((1.0 - side_left / 12.0) * 14)
            img[horizon:, :w, :] = np.array([70, 76, 82], dtype=np.uint8)
        if side_right < 12:
            w = int((1.0 - side_right / 12.0) * 14)
            img[horizon:, C.CAM_WIDTH - w:, :] = np.array([70, 76, 82], dtype=np.uint8)
        # traffic light: colour disc at lateral offset, size by distance
        light = self.traffic_light()
        if light is not None and light["distance"] < 30.0:
            lateral = (light["x"] - self.x) if light["axis"] == "x" else (light["y"] - self.y)
            col = int(C.CAM_WIDTH / 2 + np.clip(lateral * 2.0, -20, 20))
            row = int(horizon - 6 + np.clip(light["distance"] * 0.15, 0, 10))
            radius = int(max(2, 5 - light["distance"] / 10.0))
            colour = {"green": (60, 230, 90), "yellow": (245, 215, 60),
                      "red": (240, 70, 60)}[light["state"]]
            yy, xx = np.ogrid[-radius:radius + 1, -radius:radius + 1]
            mask = yy * yy + xx * xx <= radius * radius
            r0, r1 = max(0, row - radius), min(C.CAM_HEIGHT, row + radius + 1)
            c0, c1 = max(0, col - radius), min(C.CAM_WIDTH, col + radius + 1)
            sub = mask[(r0 - (row - radius)):(r1 - (row - radius)),
                       (c0 - (col - radius)):(c1 - (col - radius))]
            img[r0:r1, c0:c1][sub] = colour
        # pedestrians ahead as dark blobs
        for px, py in self.scene.pedestrian_positions(self.time):
            dx, dy = px - self.x, py - self.y
            forward = dx * math.cos(self.yaw) + dy * math.sin(self.yaw)
            lateral = -dx * math.sin(self.yaw) + dy * math.cos(self.yaw)
            if 1.0 < forward < 35.0 and abs(lateral) < forward * 0.8:
                col = int(C.CAM_WIDTH / 2 + lateral / forward * 24)
                row = int(horizon + 8 - 6.0 / max(1.0, forward) * 10)
                h = int(max(2, 10.0 / max(1.0, forward)))
                img[max(0, row - h):min(C.CAM_HEIGHT, row), max(0, col - 2):min(C.CAM_WIDTH, col + 2), :] = np.array([40, 42, 44], dtype=np.uint8)
        if self.noise:
            img = np.clip(img.astype(np.float64) + 3.0 * self.rng.normal(size=img.shape),
                          0, 255).astype(np.uint8)
        return img

    # ------------------------------------------------------------------ extras
    def goal(self) -> np.ndarray:
        return np.array(self.scene.goal, dtype=np.float64)

    def scene_map(self, resolution: float = C.PLAN_RESOLUTION) -> np.ndarray:
        return self.scene.coarse_map(resolution)
