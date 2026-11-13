"""MuJoCo quadrotor simulation with lidar, barometer, GPS, IMU and camera.

The vehicle is a real 6-DOF rigid body: gravity, collective thrust along the
body z-axis, attitude tracking and aerodynamic drag are all integrated by
MuJoCo.  A velocity-tracking outer loop converts the guidance command
``(vx, vy, vz, yaw_rate)`` into a thrust vector plus body torque.
"""

from __future__ import annotations

import math
from typing import Optional, Sequence, Tuple

import numpy as np

try:  # pragma: no cover - environment dependent
    import mujoco
except Exception as exc:  # pragma: no cover
    mujoco = None
    _MUJOCO_ERROR = exc
else:
    _MUJOCO_ERROR = None

from . import config as C
from .geometry import rot2, wrap_angle
from .scenes import Scene, get_scene, mujoco_xml


def _rot_z(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _vee(m: np.ndarray) -> np.ndarray:
    return np.array([m[2, 1] - m[1, 2], m[0, 2] - m[2, 0], m[1, 0] - m[0, 1]]) * 0.5


class UAVSim:
    """Physical quadrotor simulation with an airspace scene."""

    def __init__(self, scene_name: str = "city", seed: int = C.DEFAULT_SEED,
                 noise: bool = True):
        if mujoco is None:
            raise RuntimeError(f"mujoco is not importable: {_MUJOCO_ERROR}")
        self.scene: Scene = get_scene(scene_name)
        self.rng = np.random.default_rng(seed)
        self.noise = bool(noise)
        self.model = mujoco.MjModel.from_xml_string(mujoco_xml(self.scene))
        self.data = mujoco.MjData(self.model)
        self.body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "uav")
        self._ray_group = np.zeros(6, dtype=np.uint8)
        self._ray_group[1] = 1          # scene geoms live in group 1 (hull = group 2)

        # vehicle / controller parameters
        self.mass = C.MASS
        self.thrust_max = C.THRUST_MAX
        self.kp_xy = 2.6
        self.ki_xy = 0.35
        self.kp_z = 3.2
        self.ki_z = 0.6
        self.k_att = 5.5
        self.k_rate = 0.28
        self.k_yaw = 0.16
        self.drag_lin = np.array([0.16, 0.16, 0.22])
        self.drag_quad = np.array([0.045, 0.045, 0.06])
        self._i_xy = np.zeros(2)
        self._i_z = 0.0

        # wind (slowly varying gust process)
        self.wind = np.array(C.WIND_BASE, dtype=np.float64)
        self.wind_gust = np.zeros(2)
        self.wind_tau = C.WIND_TAU

        # sensor noise
        self.lidar_sigma = 0.05 if self.noise else 0.0
        self.lidar_outlier_p = 0.04 if self.noise else 0.0
        self.lidar_dropout_p = 0.03 if self.noise else 0.0
        self.baro_sigma = C.BARO_SIGMA if self.noise else 0.0
        self.gps_sigma = C.GPS_SIGMA if self.noise else 0.0
        self.imu_sigma = 0.003 if self.noise else 0.0
        self.odom_scale = 1.0 + (0.02 * self.rng.normal() if self.noise else 0.0)
        self.odom_yaw_bias = (0.004 * self.rng.normal() if self.noise else 0.0)
        self.gps_bias = (self.gps_sigma * 2.0 * self.rng.normal(size=2)
                         if self.noise else np.zeros(2))

        self._angles = np.linspace(-math.pi, math.pi, C.LIDAR_BEAMS, endpoint=False)
        self._cam_angles = np.linspace(-C.CAM_FOV / 2, C.CAM_FOV / 2, C.CAM_WIDTH // 2)

        self.time = 0.0
        self.collisions = 0
        self.nfz_violations = 0
        self._collision_active = False
        self._collision_cooldown = 0.0
        self._inside_nfz = False
        self.cmd = np.zeros(4)
        self.reset()

    # ------------------------------------------------------------------ reset
    def reset(self, pose: Optional[Sequence[float]] = None) -> None:
        if pose is None:
            x, y, alt, yaw = self.scene.start
        else:
            x, y, alt, yaw = [float(v) for v in pose]
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[0:3] = [x, y, alt]
        self.data.qpos[3:7] = [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]
        self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        self.odom = np.array([x, y, alt, yaw], dtype=np.float64)
        self.time = 0.0
        self.collisions = 0
        self.nfz_violations = 0
        self._collision_active = False
        self._collision_cooldown = 0.0
        self._inside_nfz = False
        self._i_xy[:] = 0.0
        self._i_z = 0.0
        self.wind_gust[:] = 0.0
        self.cmd[:] = 0.0

    # ------------------------------------------------------------------- state
    def _pose(self) -> np.ndarray:
        p = self.data.xpos[self.body]
        r = self.data.xmat[self.body].reshape(3, 3)
        yaw = math.atan2(r[1, 0], r[0, 0])
        return np.array([p[0], p[1], p[2], yaw], dtype=np.float64)

    @property
    def pose(self) -> np.ndarray:
        """Ground-truth pose (x, y, altitude, yaw)."""
        return self._pose()

    def _euler(self, r: np.ndarray) -> Tuple[float, float]:
        pitch = math.asin(max(-1.0, min(1.0, -r[2, 0])))
        roll = math.atan2(r[2, 1], r[2, 2])
        return roll, pitch

    # ------------------------------------------------------------------ step
    def step(self, cmd: Sequence[float], dt: float = 1.0 / C.CONTROL_HZ) -> None:
        """Advance the simulation by ``dt`` seconds.

        ``cmd`` = (vx m/s, vy m/s, vz m/s (+up), yaw rate rad/s).
        """
        cmd = np.asarray(cmd, dtype=np.float64)
        self.cmd = cmd.copy()
        cmd = np.clip(cmd, [-C.MAX_VX, -C.MAX_VY, -C.MAX_VZ, -C.MAX_YAW_RATE],
                      [C.MAX_VX, C.MAX_VY, C.MAX_VZ, C.MAX_YAW_RATE])
        n_sub = max(1, int(round(dt / C.PHYS_DT)))
        for _ in range(n_sub):
            self._physics_step(cmd, C.PHYS_DT)
        self.time += n_sub * C.PHYS_DT

    def _physics_step(self, cmd: np.ndarray, dt: float) -> None:
        r = self.data.xmat[self.body].reshape(3, 3)
        v_world = np.asarray(self.data.qvel[0:3], dtype=np.float64)
        w_body = np.asarray(self.data.qvel[3:6], dtype=np.float64)
        yaw = math.atan2(r[1, 0], r[0, 0])
        roll, pitch = self._euler(r)

        # wind gust random walk
        if self.noise:
            k = dt / self.wind_tau
            self.wind_gust += -k * self.wind_gust + math.sqrt(2 * k) * C.WIND_GUST * self.rng.normal(size=2) * 0.6
        wind = np.array([self.wind[0] + self.wind_gust[0],
                         self.wind[1] + self.wind_gust[1], 0.0])

        # ---- outer velocity loop (world frame)
        e_xy = np.array([cmd[0] - v_world[0], cmd[1] - v_world[1]])
        self._i_xy = np.clip(self._i_xy + e_xy * dt, -3.0, 3.0)
        a_xy = self.kp_xy * e_xy + self.ki_xy * self._i_xy
        e_z = cmd[2] - v_world[2]
        self._i_z = float(np.clip(self._i_z + e_z * dt, -3.0, 3.0))
        a_z = self.kp_z * e_z + self.ki_z * self._i_z
        a_des = np.array([a_xy[0], a_xy[1], a_z + C.GRAVITY])
        f_des = self.mass * a_des
        thrust = float(np.clip(np.linalg.norm(f_des), 0.0, self.thrust_max))
        bz_des = f_des / max(1e-6, np.linalg.norm(f_des))

        # ---- desired attitude: body z towards thrust, yaw from yaw-rate command
        yaw_des = wrap_angle(yaw + float(cmd[3]) * dt)
        x_c = np.array([math.cos(yaw_des), math.sin(yaw_des), 0.0])
        y_c = np.cross(bz_des, x_c)
        if np.linalg.norm(y_c) < 1e-6:
            y_c = np.array([0.0, 1.0, 0.0])
        y_c /= np.linalg.norm(y_c)
        x_c = np.cross(y_c, bz_des)
        r_des = np.column_stack([x_c, y_c, bz_des])

        # ---- attitude PD in body frame
        e_r = _vee(r_des.T @ r - r.T @ r_des)
        torque = -self.k_att * e_r - self.k_rate * w_body
        torque[2] += self.k_yaw * (float(cmd[3]) - w_body[2])

        # ---- aerodynamic drag relative to the wind
        v_rel = v_world - wind
        f_drag = -(self.drag_lin * v_rel + self.drag_quad * np.abs(v_rel) * v_rel)

        force_w = r[:, 2] * thrust + f_drag
        torque_w = r @ torque
        self.data.xfrc_applied[self.body] = np.concatenate([force_w, torque_w])
        mujoco.mj_step(self.model, self.data)
        self.data.xfrc_applied[self.body] = 0.0

        # ---- events: collisions and no-fly-zone entry
        contact = bool(np.any(self.data.contact.dist[: self.data.ncon] < 0.0)) if self.data.ncon else False
        self._collision_cooldown = max(0.0, self._collision_cooldown - dt)
        if contact and not self._collision_active and self._collision_cooldown <= 0.0:
            self.collisions += 1
            self._collision_cooldown = 0.6
        self._collision_active = contact
        inside = self.scene.inside_nfz(float(self.data.xpos[self.body][0]),
                                       float(self.data.xpos[self.body][1]))
        if inside and not self._inside_nfz:
            self.nfz_violations += 1
        self._inside_nfz = inside

    def _ray(self, origin, direction):
        """mj_ray 跨版本兼容（3.2+ 需要 int32 的 geomid 输出数组，旧版接受 None）。"""
        geomid = np.zeros(1, dtype=np.int32)
        try:
            out = mujoco.mj_ray(self.model, self.data, origin, direction,
                                self._ray_group, 1, self.body, geomid)
        except TypeError:
            out = mujoco.mj_ray(self.model, self.data, origin, direction,
                                self._ray_group, 1, self.body, None)
        if isinstance(out, tuple):
            out = out[0]
        return float(out)

    # ----------------------------------------------------------------- sensors
    def lidar(self) -> np.ndarray:
        """360 deg horizontal lidar, 72 beams (metres)."""
        pose = self._pose()
        origin = np.array([pose[0], pose[1], pose[2]])
        beam_angles = pose[3] + self._angles
        ranges = np.empty(C.LIDAR_BEAMS, dtype=np.float64)
        for i, a in enumerate(beam_angles):
            direction = np.array([math.cos(a), math.sin(a), 0.0])
            dist = self._ray(origin, direction)
            if dist < 0.0:
                dist = C.LIDAR_RANGE_MAX
            ranges[i] = min(C.LIDAR_RANGE_MAX, dist)
        if self.noise:
            ranges = ranges * (1.0 + 0.008 * self.rng.normal(size=ranges.shape))
            ranges += self.lidar_sigma * self.rng.normal(size=ranges.shape)
            outlier = self.rng.random(ranges.shape) < self.lidar_outlier_p
            ranges[outlier] = self.rng.uniform(2.0, C.LIDAR_RANGE_MAX, size=outlier.sum())
            dropout = self.rng.random(ranges.shape) < self.lidar_dropout_p
            ranges[dropout] = C.LIDAR_RANGE_MAX
        return np.clip(ranges, C.LIDAR_RANGE_MIN, C.LIDAR_RANGE_MAX)

    def lidar_beam_angles(self) -> np.ndarray:
        return self._angles.copy()

    # Backwards-compatible alias (perception/planner code uses sonar naming).
    def sonar(self) -> np.ndarray:
        return self.lidar()

    def sonar_beam_angles(self) -> np.ndarray:
        return self.lidar_beam_angles()

    def velocity(self) -> np.ndarray:
        """World-frame linear velocity (m/s), used by HUD / metrics."""
        return np.asarray(self.data.qvel[0:3], dtype=np.float64).copy()

    def altitude(self) -> float:
        a = float(self.data.xpos[self.body][2])
        if self.noise:
            a += self.baro_sigma * self.rng.normal()
        return a

    def imu(self) -> Tuple[float, float]:
        _, w_body = (np.asarray(self.data.qvel[0:3], dtype=np.float64),
                     np.asarray(self.data.qvel[3:6], dtype=np.float64))
        yaw = self._pose()[3]
        if self.noise:
            return wrap_angle(yaw + self.imu_sigma * self.rng.normal()), \
                float(w_body[2] + 0.008 * self.rng.normal())
        return yaw, float(w_body[2])

    def gps(self) -> np.ndarray:
        p = self.data.xpos[self.body]
        pos = np.array([p[0], p[1]], dtype=np.float64)
        if self.noise:
            pos = pos + self.gps_bias + self.gps_sigma * self.rng.normal(size=2)
        return pos

    def integrate_odometry(self, dt: float) -> np.ndarray:
        """Fuse GPS position (slow correction) with dead-reckoning (drift)."""
        r = self.data.xmat[self.body].reshape(3, 3)
        v_world = np.asarray(self.data.qvel[0:3], dtype=np.float64)
        yaw = self._pose()[3]
        self.odom[0] += self.odom_scale * v_world[0] * dt
        self.odom[1] += self.odom_scale * v_world[1] * dt
        self.odom[2] += v_world[2] * dt
        if self.noise:
            gps = self.gps()
            self.odom[0:2] = 0.97 * self.odom[0:2] + 0.03 * gps
        self.odom[3] = wrap_angle(0.4 * yaw + 0.6 * (self.odom[3] + self.odom_yaw_bias * dt))
        return self.odom.copy()

    # ------------------------------------------------------------------ camera
    def camera(self) -> np.ndarray:
        """Forward-looking RGB camera image (48 x 64 x 3) built from lidar depth."""
        pose = self._pose()
        origin = np.array([pose[0], pose[1], pose[2]])
        ranges = np.empty(len(self._cam_angles))
        for i, a in enumerate(self._cam_angles + pose[3]):
            direction = np.array([math.cos(a), math.sin(a), 0.0])
            d = self._ray(origin, direction)
            ranges[i] = C.CAM_RANGE if d < 0 else min(C.CAM_RANGE, d)
        if self.noise:
            ranges = ranges + 0.08 * self.rng.normal(size=ranges.shape)
        img = np.zeros((C.CAM_HEIGHT, C.CAM_WIDTH, 3), dtype=np.uint8)
        horizon = int(C.CAM_HEIGHT * 0.52)
        # sky gradient
        for row in range(horizon):
            f = row / max(1, horizon)
            img[row, :, :] = np.array([150 - 60 * f, 190 - 50 * f, 230 - 40 * f], dtype=np.uint8)
        # ground gradient
        for row in range(horizon, C.CAM_HEIGHT):
            f = (row - horizon) / max(1, C.CAM_HEIGHT - horizon)
            img[row, :, :] = np.array([70 - 25 * f, 92 - 30 * f, 62 - 20 * f], dtype=np.uint8)
        cols = np.linspace(0, len(ranges) - 1, C.CAM_WIDTH)
        prof = np.interp(cols, np.arange(len(ranges)), ranges)
        for c in range(C.CAM_WIDTH):
            rng = float(prof[c])
            closeness = max(0.0, 1.0 - rng / C.CAM_RANGE)
            height = int(closeness * horizon * 0.9)
            if height > 1:
                top = max(0, horizon - height)
                shade = int(60 + 120 * (1.0 - closeness))
                img[top:horizon, c, :] = np.array([shade, shade + 6, shade + 10], dtype=np.uint8)
        if self.noise:
            img = np.clip(img.astype(np.float64) + 4.0 * self.rng.normal(size=img.shape),
                          0, 255).astype(np.uint8)
        return img

    # ------------------------------------------------------------------ extras
    def nfz_clearance(self) -> float:
        pose = self._pose()
        return self.scene.nfz_clearance(float(pose[0]), float(pose[1]))

    def goal(self) -> np.ndarray:
        return np.array(self.scene.goal, dtype=np.float64)

    def scene_map(self, resolution: float = C.PLAN_RESOLUTION) -> np.ndarray:
        return self.scene.planning_map(resolution)
