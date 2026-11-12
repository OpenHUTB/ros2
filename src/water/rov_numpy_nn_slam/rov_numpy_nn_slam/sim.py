"""MuJoCo-backed 6-DOF ROV simulation with sonar, DVL, IMU and camera."""

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

# Thruster layout: 4 horizontal in an X configuration, 2 vertical.
_THRUST_MAX_H = 15.0     # N per horizontal thruster
_THRUST_MAX_V = 16.0     # N per vertical thruster
_H_ARM_X, _H_ARM_Y = 0.18, 0.15
_V_ARM_X = 0.18


class ROVSim:
    """Physical ROV simulation.

    The rigid body is integrated by MuJoCo; weight, buoyancy, quadratic fluid
    drag and thruster wrenches are applied explicitly every physics step so the
    underwater dynamics stay transparent (see ``docs/simulation.md``).
    """

    def __init__(self, scene_name: str = "pool", seed: int = C.DEFAULT_SEED,
                 noise: bool = True):
        if mujoco is None:
            raise RuntimeError(f"mujoco is not importable: {_MUJOCO_ERROR}")
        self.scene: Scene = get_scene(scene_name)
        self.rng = np.random.default_rng(seed)
        self.noise = bool(noise)
        self.model = mujoco.MjModel.from_xml_string(mujoco_xml(self.scene))
        self.data = mujoco.MjData(self.model)
        self.body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "rov")
        self.ngeom = self.model.ngeom
        self.geom_group = np.asarray(self.model.geom_group, dtype=np.int32).copy()
        # group 1 -> environment geoms visible to the sonar rays
        self.env_group = (self.geom_group == 1).astype(np.uint8)
        self._ray_group = np.zeros(6, dtype=np.uint8)
        self._ray_group[0] = 0
        self._ray_group[1] = 1

        # dynamics parameters
        self.mass = C.MASS
        self.added_mass = np.array([0.45, 0.75, 0.95])       # per-axis added mass
        self.drag_quad = np.array([25.0, 45.0, 30.0])        # 0.5*rho*Cd*A (N/(m/s)^2)
        self.drag_lin = np.array([6.0, 9.0, 7.0])
        self.drag_quad_ang = np.array([0.18, 0.20, 0.30])
        self.drag_lin_ang = np.array([0.55, 0.60, 0.75])
        self.buoyancy_force = (C.WATER_DENSITY * C.VOLUME - self.mass) * C.GRAVITY
        # actuator saturation (4 x 15 N horizontal, 2 x 16 N vertical)
        self.force_max = np.array([4.0 * _THRUST_MAX_H / math.sqrt(2.0),
                                   4.0 * _THRUST_MAX_H / math.sqrt(2.0),
                                   2.0 * _THRUST_MAX_V])
        self.torque_max = np.array([2.0, 0.18 * 2.0 * _THRUST_MAX_V, 3.2])

        # sensor noise / bias
        self.sonar_sigma = 0.02 if self.noise else 0.0
        self.sonar_outlier_p = 0.04 if self.noise else 0.0
        self.sonar_dropout_p = 0.03 if self.noise else 0.0
        self.depth_sigma = 0.008 if self.noise else 0.0
        self.dvl_sigma = 0.02 if self.noise else 0.0
        self.imu_sigma = 0.004 if self.noise else 0.0
        self.odom_scale = 1.0 + (0.03 * self.rng.normal() if self.noise else 0.0)
        self.odom_yaw_bias = (0.008 * self.rng.normal() if self.noise else 0.0)

        self._angles = np.linspace(-C.SONAR_FOV / 2, C.SONAR_FOV / 2, C.SONAR_BEAMS)
        self._cam_angles = np.linspace(-C.CAM_FOV / 2, C.CAM_FOV / 2, C.CAM_WIDTH // 2)

        self.time = 0.0
        self.collisions = 0
        self._collision_active = False
        self.cmd = np.zeros(4)
        self.reset()

    # ------------------------------------------------------------------ reset
    def reset(self, pose: Optional[Sequence[float]] = None) -> None:
        if pose is None:
            x, y, depth, yaw = self.scene.start
        else:
            x, y, depth, yaw = [float(v) for v in pose]
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[0:3] = [x, y, -depth]
        self.data.qpos[3:7] = [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]
        self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        self.truth = self._pose()
        self.odom = np.array([x, y, depth, yaw], dtype=np.float64)
        self.time = 0.0
        self.collisions = 0
        self._collision_active = False
        self._collision_cooldown = 0.0
        self.cmd[:] = 0.0

    # ------------------------------------------------------------------- state
    def _pose(self) -> np.ndarray:
        p = self.data.xpos[self.body]
        r = self.data.xmat[self.body].reshape(3, 3)
        yaw = math.atan2(r[1, 0], r[0, 0])
        return np.array([p[0], p[1], -p[2], yaw], dtype=np.float64)

    @property
    def pose(self) -> np.ndarray:
        """Ground-truth pose (x, y, depth, yaw)."""
        return self._pose()

    def _body_velocity(self) -> Tuple[np.ndarray, np.ndarray]:
        """Return (linear velocity in body frame, angular velocity in body frame)."""
        r = self.data.xmat[self.body].reshape(3, 3)
        v_world = np.asarray(self.data.qvel[0:3], dtype=np.float64)
        w_body = np.asarray(self.data.qvel[3:6], dtype=np.float64)
        return r.T @ v_world, w_body

    # ---------------------------------------------------------------- wrench
    def _thruster_wrench(self, thrust_h: np.ndarray, thrust_v: np.ndarray) -> np.ndarray:
        """Map four horizontal + two vertical thruster forces to a body wrench."""
        s = 1.0 / math.sqrt(2.0)
        h1, h2, h3, h4 = thrust_h
        v1, v2 = thrust_v
        fx = (h1 + h2 - h3 - h4) * s
        fy = (h1 - h2 + h3 - h4) * s
        # z-moment of the four horizontal thrusters (documented in docs/simulation.md)
        tz = ((_H_ARM_Y * (h2 + h3)) - (_H_ARM_Y * (h1 + h4))) * s
        fz = v1 + v2
        ty = -_V_ARM_X * (v1 - v2)
        return np.array([fx, fy, fz, 0.0, ty, tz], dtype=np.float64)

    def _velocity_control(self, vx: float, vy: float, vz: float, wz: float) -> np.ndarray:
        """Simple P velocity controller + attitude stabilisation."""
        v_b, w_b = self._body_velocity()
        r = self.data.xmat[self.body].reshape(3, 3)
        euler = self._euler(r)
        # desired body velocity
        ex = vx - v_b[0]
        ey = vy - v_b[1]
        ez = vz - v_b[2]
        # drag feed-forward so the command tracks even against quadratic drag
        ff = np.array([
            np.sign(vx) * self.drag_quad[0] * vx * vx + self.drag_lin[0] * vx,
            np.sign(vy) * self.drag_quad[1] * vy * vy + self.drag_lin[1] * vy,
            np.sign(vz) * self.drag_quad[2] * vz * vz + self.drag_lin[2] * vz,
        ])
        kp = np.array([55.0, 45.0, 60.0])
        f = kp * np.array([ex, ey, ez]) + ff
        # attitude PD: hold roll/pitch at zero
        kr = np.array([9.0, 9.0])
        dr = np.array([2.2, 2.2])
        # yaw rate
        eyaw = wz - w_b[2]
        tz = 3.2 * eyaw
        t_roll = -(kr[0] * euler[0] + dr[0] * w_b[0])
        t_pitch = -(kr[1] * euler[1] + dr[1] * w_b[1])
        wrench = np.array([f[0], f[1], f[2], t_roll, t_pitch, tz])
        # actuator saturation
        wrench[0:3] = np.clip(wrench[0:3], -self.force_max, self.force_max)
        wrench[3:6] = np.clip(wrench[3:6], -self.torque_max, self.torque_max)
        return wrench

    def _euler(self, r: np.ndarray) -> Tuple[float, float, float]:
        pitch = math.asin(max(-1.0, min(1.0, -r[2, 0])))
        roll = math.atan2(r[2, 1], r[2, 2])
        return roll, pitch, 0.0

    # -------------------------------------------------------------------- step
    def step(self, cmd: Sequence[float], dt: float = 1.0 / C.CONTROL_HZ) -> None:
        """Advance the simulation by ``dt`` seconds.

        ``cmd`` = (surge m/s, sway m/s, heave m/s, yaw rate rad/s).
        """
        cmd = np.asarray(cmd, dtype=np.float64)
        self.cmd = cmd.copy()
        cmd = np.clip(cmd, [-C.MAX_SURGE, -C.MAX_SWAY, -C.MAX_HEAVE, -C.MAX_YAW_RATE],
                      [C.MAX_SURGE, C.MAX_SWAY, C.MAX_HEAVE, C.MAX_YAW_RATE])
        n_sub = max(1, int(round(dt / C.PHYS_DT)))
        for _ in range(n_sub):
            self._physics_step(cmd)
        self.time += n_sub * C.PHYS_DT

    def _physics_step(self, cmd: np.ndarray) -> None:
        r = self.data.xmat[self.body].reshape(3, 3)
        v_b, w_b = self._body_velocity()

        # ---- fluid drag (body frame, quadratic + linear)
        f_drag_b = -(self.drag_quad * np.abs(v_b) * v_b + self.drag_lin * v_b)
        t_drag_b = -(self.drag_quad_ang * np.abs(w_b) * w_b + self.drag_lin_ang * w_b)

        # ---- controller wrench (body frame)
        wrench_b = self._velocity_control(cmd[0], cmd[1], cmd[2], cmd[3])
        total_force_b = wrench_b[0:3] + f_drag_b + np.array([0.0, 0.0, self.buoyancy_force])
        total_torque_b = wrench_b[3:6] + t_drag_b

        # ---- to world frame
        force_w = r @ total_force_b
        torque_w = r @ total_torque_b
        # user forces are added on top of the model forces
        self.data.xfrc_applied[self.body] = np.concatenate([force_w, torque_w])
        mujoco.mj_step(self.model, self.data)
        self.data.xfrc_applied[self.body] = 0.0

        # collision bookkeeping (count one event per contact episode)
        contact = bool(np.any(self.data.contact.dist[: self.data.ncon] < 0.0)) if self.data.ncon else False
        self._collision_cooldown = max(0.0, getattr(self, "_collision_cooldown", 0.0) - C.PHYS_DT)
        if contact and not self._collision_active and self._collision_cooldown <= 0.0:
            self.collisions += 1
            self._collision_cooldown = 0.6
        self._collision_active = contact

    def _ray(self, origin, direction):
        """mj_ray 跨版本兼容（3.1.x 接受 geomid=None，3.2.x 需要 int32 输出数组）。"""
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
    def sonar(self) -> np.ndarray:
        """120 deg horizontal sonar fan, 64 beams (metres)."""
        pose = self._pose()
        origin = np.array([pose[0], pose[1], -pose[2]])
        beam_angles = pose[3] + self._angles
        ranges = np.empty(C.SONAR_BEAMS, dtype=np.float64)
        for i, a in enumerate(beam_angles):
            direction = np.array([math.cos(a), math.sin(a), 0.0])
            dist = self._ray(origin, direction)
            if dist < 0.0:
                dist = C.SONAR_RANGE_MAX
            ranges[i] = min(C.SONAR_RANGE_MAX, dist)
        if self.noise:
            ranges = ranges * (1.0 + 0.01 * self.rng.normal(size=ranges.shape))
            ranges += self.sonar_sigma * self.rng.normal(size=ranges.shape)
            outlier = self.rng.random(ranges.shape) < self.sonar_outlier_p
            ranges[outlier] = self.rng.uniform(0.5, C.SONAR_RANGE_MAX, size=outlier.sum())
            dropout = self.rng.random(ranges.shape) < self.sonar_dropout_p
            ranges[dropout] = C.SONAR_RANGE_MAX
        return np.clip(ranges, C.SONAR_RANGE_MIN, C.SONAR_RANGE_MAX)

    def sonar_beam_angles(self) -> np.ndarray:
        return self._angles.copy()

    def depth(self) -> float:
        d = float(-self.data.xpos[self.body][2])
        if self.noise:
            d += self.depth_sigma * self.rng.normal()
        return d

    def imu(self) -> Tuple[float, float]:
        """Return (yaw, yaw rate) with IMU noise."""
        _, w_b = self._body_velocity()
        yaw = self._pose()[3]
        if self.noise:
            yaw = wrap_angle(yaw + self.imu_sigma * self.rng.normal())
            return yaw, w_b[2] + 0.01 * self.rng.normal()
        return yaw, w_b[2]

    def dvl(self) -> np.ndarray:
        """Body-frame linear velocity with noise."""
        v_b, _ = self._body_velocity()
        if self.noise:
            v_b = v_b + self.dvl_sigma * self.rng.normal(size=3)
        return v_b

    def integrate_odometry(self, dt: float) -> np.ndarray:
        """Integrate drifting odometry from DVL + IMU (x, y, depth, yaw)."""
        v_b = self.dvl()
        yaw, _ = self.imu()
        v_w = rot2(yaw) @ v_b[:2]
        self.odom[0] += self.odom_scale * v_w[0] * dt
        self.odom[1] += self.odom_scale * v_w[1] * dt
        self.odom[2] += -v_b[2] * dt
        self.odom[3] = wrap_angle(self.odom[3] + self.odom_yaw_bias * dt)
        self.odom[3] = wrap_angle(0.35 * yaw + 0.65 * self.odom[3])
        return self.odom.copy()

    def camera(self) -> np.ndarray:
        """Simulated forward-looking RGB camera image (48 x 64 x 3, uint8).

        Built by ray-casting a horizontal depth profile in front of the ROV and
        shading it with the murky-water attenuation model.
        """
        pose = self._pose()
        origin = np.array([pose[0], pose[1], -pose[2]])
        ranges = np.empty(len(self._cam_angles))
        for i, a in enumerate(self._cam_angles + pose[3]):
            direction = np.array([math.cos(a), math.sin(a), 0.0])
            d = self._ray(origin, direction)
            ranges[i] = C.CAM_RANGE if d < 0 else min(C.CAM_RANGE, d)
        if self.noise:
            ranges = ranges + 0.05 * self.rng.normal(size=ranges.shape)
        # stretch the forward depth profile to the image width
        cols = np.linspace(0, len(ranges) - 1, C.CAM_WIDTH)
        prof = np.interp(cols, np.arange(len(ranges)), ranges)
        depth_norm = np.clip(prof / C.CAM_RANGE, 0.0, 1.0)[None, :]
        fog = np.clip(1.0 - depth_norm, 0.0, 1.0) ** 1.5
        base = np.array([12, 46, 64], dtype=np.float64)
        near = np.array([214, 196, 142], dtype=np.float64)
        col = base[None, None, :] + (near - base)[None, None, :] * fog[:, :, None]
        band = (0.55 + 0.80 * (1.0 - np.linspace(0, 1, C.CAM_HEIGHT)))[:, None, None]
        img = np.clip(col * band, 0, 255).astype(np.uint8)
        img = np.repeat(img, C.CAM_WIDTH, axis=1) if img.shape[1] == 1 else img
        if self.noise:
            img = np.clip(img.astype(np.float64) + 4.0 * self.rng.normal(size=img.shape),
                          0, 255).astype(np.uint8)
        return img
        if self.noise:
            img = np.clip(img.astype(np.float64) + 4.0 * self.rng.normal(size=img.shape),
                          0, 255).astype(np.uint8)
        return img

    # ------------------------------------------------------------------ utils
    def reset_free(self) -> None:
        self.reset()

    def goal(self) -> np.ndarray:
        return np.array(self.scene.goal, dtype=np.float64)

    def scene_map(self, resolution: float = C.PLAN_RESOLUTION) -> np.ndarray:
        return self.scene.coarse_map(resolution)
