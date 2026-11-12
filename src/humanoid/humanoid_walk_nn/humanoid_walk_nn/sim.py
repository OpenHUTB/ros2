"""Simplified bipedal humanoid simulator.

The model is a 2-D walking robot: a linear inverted-pendulum style lateral
balance state on top of a step-phase gait generator.  It keeps the aspects a
navigation / perception stack needs — walking speed & turning limits, lateral
balance failures ("falls"), footstep phase, lidar / IMU / joint encoders and a
forward depth camera — while staying small enough to run on a CPU-only VM.

    v_dot   = a - k_r sign(v),          |a| <= MAX_ACCEL / MAX_DECEL
    psi_dot = omega,                    |omega| <= MAX_TURN_RATE
    b_dot   = k_b * v * omega - k_s * b        (lateral CoM offset)
    phase_dot = |v| / STEP_LENGTH              (one step per 0.36 m)
    fall    when |b| > BALANCE_LIMIT for FALL_HOLD seconds
"""

from __future__ import annotations

import math
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

from . import config as C
from .geometry import wrap_angle
from .scenes import Scene, get_scene


class HumanoidSim:
    """A walking humanoid inside a 2-D floor plan."""

    def __init__(self, scene_name: str = "hall", seed: int = C.DEFAULT_SEED,
                 noise: bool = True):
        self.scene: Scene = get_scene(scene_name)
        self.rng = np.random.default_rng(seed)
        self.noise = bool(noise)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = 0.0
        self.yaw_rate = 0.0
        self.phase = 0.0
        self.balance = 0.0
        self._fall_timer = 0.0

        self.lidar_sigma = 0.02 if self.noise else 0.0
        self.lidar_outlier_p = 0.03 if self.noise else 0.0
        self.lidar_dropout_p = 0.02 if self.noise else 0.0
        self.gnss_sigma = C.GNSS_SIGMA if self.noise else 0.0
        self.imu_sigma = C.IMU_SIGMA if self.noise else 0.0
        self.odom_scale = 1.0 + (0.01 * self.rng.normal() if self.noise else 0.0)

        self._angles = np.linspace(-math.pi, math.pi, C.LIDAR_BEAMS, endpoint=False)

        self.time = 0.0
        self.collisions = 0
        self.falls = 0
        self.distance = 0.0
        self.steps = 0
        self._contact = False
        self._contact_cooldown = 0.0
        self._cart_contact: set = set()
        self.cmd = np.zeros(2)
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
        self.yaw_rate = 0.0
        self.phase = 0.0
        self.balance = 0.0
        self._fall_timer = 0.0
        self.time = 0.0
        self.collisions = 0
        self.falls = 0
        self.distance = 0.0
        self.steps = 0
        self._contact = False
        self._contact_cooldown = 0.0
        self._cart_contact.clear()
        self.odom = np.array([x, y, 0.0, yaw], dtype=np.float64)
        self.cmd[:] = 0.0

    # ------------------------------------------------------------------- state
    @property
    def pose(self) -> np.ndarray:
        return np.array([self.x, self.y, 0.0, self.yaw], dtype=np.float64)

    @property
    def speed(self) -> float:
        return float(self.v)

    @property
    def turn_rate(self) -> float:
        return float(self.yaw_rate)

    # -------------------------------------------------------------------- step
    def step(self, cmd: Sequence[float], dt: float = 1.0 / C.CONTROL_HZ) -> None:
        """Advance ``dt`` seconds with ``cmd = (walk_speed [m/s], turn_rate [rad/s])``."""
        cmd = np.asarray(cmd, dtype=np.float64).ravel()
        v_target = float(np.clip(cmd[0] if cmd.size else 0.0, -C.MAX_BACKWARD, C.MAX_SPEED))
        w_target = float(np.clip(cmd[1] if cmd.size > 1 else 0.0,
                                 -C.MAX_TURN_RATE, C.MAX_TURN_RATE))
        self.cmd = np.array([v_target, w_target])
        n_sub = max(1, int(round(dt / C.PHYS_DT)))
        for _ in range(n_sub):
            self._physics_step(v_target, w_target, C.PHYS_DT)
        self.time += n_sub * C.PHYS_DT

    def _physics_step(self, v_target: float, w_target: float, dt: float) -> None:
        # longitudinal acceleration / braking limits
        dv = v_target - self.v
        accel = C.MAX_ACCEL if dv > 0 else C.MAX_DECEL
        self.v += float(np.clip(dv, -accel * dt, accel * dt))
        if abs(self.v) < 0.01:
            self.v = 0.0

        # turn rate ramp
        dw = w_target - self.yaw_rate
        self.yaw_rate += float(np.clip(dw, -C.TURN_ACCEL * dt, C.TURN_ACCEL * dt))
        self.yaw = wrap_angle(self.yaw + self.yaw_rate * dt)

        # lateral balance (linear inverted pendulum approximation)
        gait_wobble = 0.004 * math.sin(2.0 * math.pi * self.phase) if self.v > 0.05 else 0.0
        self.balance += (C.BALANCE_GAIN * self.v * self.yaw_rate
                         - C.BALANCE_RESTORE * self.balance) * dt + gait_wobble * dt
        if self.noise:
            self.balance += 0.0015 * self.rng.normal() * math.sqrt(dt)
        if abs(self.balance) > C.BALANCE_LIMIT:
            self._fall_timer += dt
            if self._fall_timer > C.FALL_HOLD:
                self.falls += 1
                self._fall_timer = 0.0
                self.balance = 0.0
                self.v = 0.0
                self.yaw_rate = 0.0
        else:
            self._fall_timer = max(0.0, self._fall_timer - 0.5 * dt)

        # gait phase and position integration
        moved = abs(self.v) * dt
        self.phase += moved / C.STEP_LENGTH
        self.steps = int(self.phase)
        self.distance += moved

        nx = self.x + self.v * math.cos(self.yaw) * dt
        ny = self.y + self.v * math.sin(self.yaw) * dt

        hit = self._check_collision(nx, ny)
        if hit is not None:
            if not self._contact and self._contact_cooldown <= 0.0:
                self.collisions += 1
                self._contact_cooldown = 0.6
                self.balance += 0.02
            self._contact = True
            self.v = -0.10 * self.v
            nx, ny = self.x, self.y
        else:
            self._contact = False
        self._contact_cooldown = max(0.0, self._contact_cooldown - dt)
        self.x, self.y = nx, ny

    def _check_collision(self, x: float, y: float) -> Optional[Tuple[str, int]]:
        radius = C.ROBOT_RADIUS
        for i, (cx, cy, hw, hh) in enumerate(self.scene.walls + self.scene.obstacles):
            dx = max(abs(x - cx) - hw, 0.0)
            dy = max(abs(y - cy) - hh, 0.0)
            if dx * dx + dy * dy <= radius * radius:
                return ("rect", i)
        for i, (cx, cy, r) in enumerate(self.scene.pillars):
            if (x - cx) ** 2 + (y - cy) ** 2 <= (radius + r) ** 2:
                return ("pillar", i)
        for i, (cx, cy, hw, hh) in enumerate(self.scene.cart_positions(self.time)):
            dx = max(abs(x - cx) - hw, 0.0)
            dy = max(abs(y - cy) - hh, 0.0)
            if dx * dx + dy * dy <= radius * radius:
                return ("cart", i)
        return None

    # ----------------------------------------------------------------- sensors
    def _ray_hit(self, x: float, y: float, dx: float, dy: float,
                 max_range: float) -> float:
        step = 0.10
        dist = 0.15
        obstacles = self.scene.walls + self.scene.obstacles
        carts = self.scene.cart_positions(self.time)
        while dist < max_range:
            px, py = x + dx * dist, y + dy * dist
            if not self.scene.is_free(px, py, radius=0.05):
                return dist
            for cx, cy, hw, hh in obstacles:
                if abs(px - cx) <= hw and abs(py - cy) <= hh:
                    return dist
            for cx, cy, hw, hh in carts:
                if abs(px - cx) <= hw and abs(py - cy) <= hh:
                    return dist
            dist += step
        return max_range

    def lidar(self) -> np.ndarray:
        pose = self.pose
        ranges = np.empty(C.LIDAR_BEAMS, dtype=np.float64)
        for i, rel in enumerate(self._angles):
            ang = pose[3] + rel
            ranges[i] = self._ray_hit(pose[0], pose[1], math.cos(ang), math.sin(ang),
                                      C.LIDAR_RANGE_MAX)
        if self.noise:
            ranges = ranges * (1.0 + 0.01 * self.rng.normal(size=ranges.shape))
            ranges += self.lidar_sigma * self.rng.normal(size=ranges.shape)
            outlier = self.rng.random(ranges.shape) < self.lidar_outlier_p
            ranges[outlier] = self.rng.uniform(1.0, C.LIDAR_RANGE_MAX, size=outlier.sum())
            dropout = self.rng.random(ranges.shape) < self.lidar_dropout_p
            ranges[dropout] = C.LIDAR_RANGE_MAX
        return np.clip(ranges, C.LIDAR_RANGE_MIN, C.LIDAR_RANGE_MAX)

    def lidar_beam_angles(self) -> np.ndarray:
        return self._angles.copy()

    # shared-naming aliases for the SLAM / planner code
    def sonar(self) -> np.ndarray:
        return self.lidar()

    def sonar_beam_angles(self) -> np.ndarray:
        return self.lidar_beam_angles()

    def camera_depth(self) -> np.ndarray:
        """Forward depth camera (32x32): bright = near, dark = far/empty."""
        img = np.zeros((C.CAM_HEIGHT, C.CAM_WIDTH), dtype=np.float32)
        pose = self.pose
        half = C.CAM_FOV / 2.0
        for col in range(C.CAM_WIDTH):
            bearing = pose[3] + (-half + C.CAM_FOV * (col + 0.5) / C.CAM_WIDTH)
            dist = self._ray_hit(pose[0], pose[1], math.cos(bearing), math.sin(bearing),
                                 C.CAM_RANGE)
            proximity = float(np.clip(1.0 - dist / C.CAM_RANGE, 0.0, 1.0))
            height = int(round(C.CAM_HEIGHT * proximity))
            if height > 0:
                img[C.CAM_HEIGHT - height:, col] = 1.0
        # floor gradient in the bottom rows for a more natural image
        for row in range(C.CAM_HEIGHT):
            img[row, :] = np.maximum(img[row, :], 0.06 + 0.16 * row / C.CAM_HEIGHT)
        if self.noise:
            img = np.clip(img + 0.02 * self.rng.normal(size=img.shape), 0.0, 1.0)
        return img

    def camera(self) -> np.ndarray:
        depth = self.camera_depth()
        return np.stack([depth, depth, depth], axis=-1)

    def imu(self) -> Tuple[float, float, float]:
        yaw_rate = self.yaw_rate + (0.01 * self.rng.normal() if self.noise else 0.0)
        roll = 0.05 * self.balance + (self.imu_sigma * self.rng.normal() if self.noise else 0.0)
        pitch = -0.02 * self.v + (self.imu_sigma * self.rng.normal() if self.noise else 0.0)
        return float(roll), float(pitch), float(yaw_rate)

    def joints(self) -> np.ndarray:
        """Hip/knee/ankle angles (left, right) from the gait phase, with noise."""
        p = 2.0 * math.pi * self.phase
        hip = 0.45 * math.sin(p)
        knee = 0.60 * max(0.0, math.sin(p + 0.6))
        ankle = -0.25 * math.sin(p)
        joints = np.array([hip, knee, ankle, -hip, -knee, -ankle], dtype=np.float64)
        if self.noise:
            joints += 0.01 * self.rng.normal(size=joints.shape)
        return joints

    def gnss(self) -> np.ndarray:
        pos = np.array([self.x, self.y], dtype=np.float64)
        if self.noise:
            pos += self.gnss_sigma * self.rng.normal(size=2)
        return pos

    def integrate_odometry(self, dt: float) -> np.ndarray:
        self.odom[0] += self.odom_scale * self.v * math.cos(self.yaw) * dt
        self.odom[1] += self.odom_scale * self.v * math.sin(self.yaw) * dt
        self.odom[3] = wrap_angle(0.7 * (self.odom[3] + self.yaw_rate * dt) + 0.3 * self.yaw)
        if self.noise:
            gnss = self.gnss()
            self.odom[0:2] = 0.985 * self.odom[0:2] + 0.015 * gnss
        return self.odom.copy()

    # ------------------------------------------------------------------ extras
    def goal(self) -> np.ndarray:
        return np.array(self.scene.goal, dtype=np.float64)

    def scene_map(self, resolution: float = C.PLAN_RESOLUTION) -> np.ndarray:
        return self.scene.coarse_map(resolution)
