"""MuJoCo 3.12.0 physics backend and simulated sensors."""

from __future__ import annotations

import math
from typing import Dict, Optional, Sequence, Tuple

import numpy as np
from PIL import Image, ImageDraw

try:
    import mujoco
except Exception as exc:  # pragma: no cover - handled by runtime bootstrap
    mujoco = None
    _MUJOCO_IMPORT_ERROR = exc
else:
    _MUJOCO_IMPORT_ERROR = None

from ..config import (
    BEV_EXTENT,
    BEV_SIZE,
    CHASSIS_Z,
    LIDAR_BEAMS,
    LIDAR_HEIGHT,
    LIDAR_RANGE_MAX,
    LIDAR_RANGE_MIN,
    MAP_RESOLUTION,
    MAX_OMEGA,
    MAX_SPEED,
    WHEEL_RADIUS,
    WHEEL_TRACK,
)
from ..geometry import wrap_angle
from .world import Scene, get_scene, mujoco_xml, scene_rgb


class MuJoCoSimulation:
    """Differential-drive vehicle with lidar, RGB-D and bird's-eye cameras."""

    def __init__(self, scene_name: str = "warehouse", seed: int = 20261026,
                 noise: bool = True):
        if mujoco is None:
            raise RuntimeError(f"MuJoCo is not importable: {_MUJOCO_IMPORT_ERROR}")
        self.scene: Scene = get_scene(scene_name)
        self.noise = bool(noise)
        self.rng = np.random.default_rng(seed)
        self.model = mujoco.MjModel.from_xml_string(mujoco_xml(self.scene))
        self.data = mujoco.MjData(self.model)
        self.body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
        self.left_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "left")
        self.right_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "right")
        self.geom_names = [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, i) or ""
            for i in range(self.model.ngeom)
        ]
        self.geom_group = np.asarray(self.model.geom_group, dtype=np.int32)
        self.world_geoms = np.array(
            [
                (g in (0, 1)) and self.geom_names[i] != "floor"
                for i, g in enumerate(self.geom_group)
            ],
            dtype=bool,
        )
        self.scene_rgb = scene_rgb(self.scene)
        self.scene_h, self.scene_w = self.scene_rgb.shape[:2]

        self._angles = np.linspace(0.0, 2.0 * np.pi, LIDAR_BEAMS, endpoint=False)
        self._cos = np.cos(self._angles)
        self._sin = np.sin(self._angles)
        self._laser_geomgroup = np.array([1, 1, 0, 0, 0, 0], dtype=np.uint8)

        # Odometry error model: 2 % wheel-scale error, small yaw bias/noise.
        self.odom_scale = 1.0 + (0.02 if self.noise else 0.0) * self.rng.normal()
        self.odom_bias = (0.01 if self.noise else 0.0) * self.rng.normal()
        self.odom = np.zeros(3, dtype=np.float64)
        self.truth = np.zeros(3, dtype=np.float64)
        self.collisions = 0
        self.collision_active = False
        self.contact_steps = 0
        self.time = 0.0
        self.last_cmd = np.zeros(2, dtype=np.float64)
        self.last_wheels = np.zeros(2, dtype=np.float64)
        self.reset()

    # ------------------------------------------------------------------ reset
    def reset(self, pose: Optional[Sequence[float]] = None) -> None:
        if pose is None:
            x, y, yaw = self.scene.start[0], self.scene.start[1], self.scene.start_yaw
        else:
            x, y, yaw = float(pose[0]), float(pose[1]), float(pose[2])
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[0:3] = [x, y, CHASSIS_Z]
        self.data.qpos[3:7] = [math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)]
        self.data.qvel[:] = 0.0
        self.data.ctrl[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        # Let the caster and wheels settle on the floor.
        for _ in range(60):
            mujoco.mj_step(self.model, self.data)
        self.truth = self._read_truth()
        self.odom = self.truth.copy()
        self.collisions = 0
        self.collision_active = False
        self.contact_steps = 0
        self.time = 0.0
        self.last_cmd[:] = 0.0
        self.last_wheels[:] = 0.0

    def set_pose(self, x: float, y: float, yaw: float) -> None:
        self.reset((x, y, yaw))

    # ------------------------------------------------------------------ state
    def _read_truth(self) -> np.ndarray:
        p = self.data.xpos[self.body_id]
        r = self.data.xmat[self.body_id].reshape(3, 3)
        yaw = math.atan2(r[1, 0], r[0, 0])
        return np.array([float(p[0]), float(p[1]), float(yaw)], dtype=np.float64)

    def _read_velocity(self) -> Tuple[float, float]:
        q = self.data.qvel
        yaw = self.truth[2]
        v = float(q[0] * math.cos(yaw) + q[1] * math.sin(yaw))
        w = float(q[5])
        return v, w

    def state(self) -> Dict[str, object]:
        v, w = self._read_velocity()
        return {
            "x": float(self.truth[0]),
            "y": float(self.truth[1]),
            "yaw": float(self.truth[2]),
            "v": v,
            "w": w,
            "odom": self.odom.copy(),
            "collisions": int(self.collisions),
            "collision_active": bool(self.collision_active),
            "time": float(self.time),
            "goal_distance": float(
                math.hypot(self.scene.goal[0] - self.truth[0],
                           self.scene.goal[1] - self.truth[1])
            ),
            "scene": self.scene.name,
            "cmd": self.last_cmd.copy(),
        }

    # ------------------------------------------------------------------ update
    def _detect_collision(self) -> bool:
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            g1, g2 = int(c.geom1), int(c.geom2)
            if self.world_geoms[g1] and self.geom_group[g2] == 3:
                return True
            if self.world_geoms[g2] and self.geom_group[g1] == 3:
                return True
        return False

    def step(self, v: float, w: float, dt: float = 0.02) -> Dict[str, object]:
        """Apply a body-twist command for ``dt`` seconds."""
        v = float(np.clip(v, -0.6 * MAX_SPEED, MAX_SPEED))
        w = float(np.clip(w, -MAX_OMEGA, MAX_OMEGA))
        self.last_cmd[:] = [v, w]
        left = (v - w * WHEEL_TRACK / 2.0) / WHEEL_RADIUS
        right = (v + w * WHEEL_TRACK / 2.0) / WHEEL_RADIUS
        self.last_wheels[:] = [left, right]
        left = float(np.clip(left, -25.0, 25.0))
        right = float(np.clip(right, -25.0, 25.0))
        n = max(1, int(round(dt / float(self.model.opt.timestep))))
        hit = False
        for _ in range(n):
            self.data.ctrl[0] = left
            self.data.ctrl[1] = right
            mujoco.mj_step(self.model, self.data)
            if self._detect_collision():
                hit = True
                self.contact_steps += 1
        if hit and not self.collision_active:
            self.collisions += 1
        self.collision_active = hit
        self.truth = self._read_truth()
        self.time += dt

        # Wheel-encoder odometry, intentionally biased/drifting.
        wl = float(self.data.qvel[self.model.jnt_dofadr[self.left_joint]])
        wr = float(self.data.qvel[self.model.jnt_dofadr[self.right_joint]])
        v_meas = WHEEL_RADIUS * (wl + wr) / 2.0 * self.odom_scale
        w_meas = WHEEL_RADIUS * (wr - wl) / WHEEL_TRACK * self.odom_scale + self.odom_bias
        if self.noise:
            v_meas += self.rng.normal(0.0, 0.006)
            w_meas += self.rng.normal(0.0, 0.008)
        mid = self.odom[2] + 0.5 * w_meas * dt
        self.odom[0] += v_meas * math.cos(mid) * dt
        self.odom[1] += v_meas * math.sin(mid) * dt
        self.odom[2] = wrap_angle(self.odom[2] + w_meas * dt)
        return self.state()

    # ------------------------------------------------------------------ lidar
    def scan(self, clean: bool = False) -> np.ndarray:
        p = self.data.xpos[self.body_id]
        pnt = np.array([p[0], p[1], p[2] + LIDAR_HEIGHT], dtype=np.float64)
        yaw = self.truth[2]
        ranges = np.empty(LIDAR_BEAMS, dtype=np.float64)
        for i in range(LIDAR_BEAMS):
            a = yaw + self._angles[i]
            vec = np.array([math.cos(a), math.sin(a), 0.0], dtype=np.float64)
            d = mujoco.mj_ray(self.model, self.data, pnt, vec,
                              self._laser_geomgroup, 1, -1, None)
            ranges[i] = LIDAR_RANGE_MAX if d < 0 else min(LIDAR_RANGE_MAX, max(LIDAR_RANGE_MIN, d))
        if self.noise and not clean:
            sigma = 0.005 + 0.0022 * ranges
            ranges += self.rng.normal(0.0, 1.0, LIDAR_BEAMS) * sigma
            drop = self.rng.random(LIDAR_BEAMS) < 0.008
            ranges[drop] = LIDAR_RANGE_MAX
            outlier = self.rng.random(LIDAR_BEAMS) < 0.004
            ranges[outlier] = self.rng.uniform(0.2, LIDAR_RANGE_MAX, int(outlier.sum()))
        return np.clip(ranges, LIDAR_RANGE_MIN, LIDAR_RANGE_MAX).astype(np.float32)

    # ------------------------------------------------------------------ cameras
    def bev_image(self) -> np.ndarray:
        """64x64 RGB bird's-eye camera centred on the robot, forward = up."""
        n = BEV_SIZE
        res = BEV_EXTENT / n
        ys, xs = np.mgrid[0:n, 0:n]
        lx = (xs - (n - 1) / 2.0) * res
        ly = ((n - 1) / 2.0 - ys) * res
        c, s = math.cos(self.truth[2]), math.sin(self.truth[2])
        wx = c * lx - s * ly + self.truth[0]
        wy = s * lx + c * ly + self.truth[1]
        gx = np.clip((wx / MAP_RESOLUTION).astype(np.int32), 0, self.scene_w - 1)
        gy = np.clip((wy / MAP_RESOLUTION).astype(np.int32), 0, self.scene_h - 1)
        img = self.scene_rgb[gy, gx].copy()
        if self.noise:
            img += self.rng.normal(0.0, 0.012, img.shape)
        img8 = np.clip(img * 255.0, 0, 255).astype(np.uint8)
        pil = Image.fromarray(img8, mode="RGB")
        draw = ImageDraw.Draw(pil)

        # Goal marker / border arrow, so a pure-image policy always sees the goal.
        dx = self.scene.goal[0] - self.truth[0]
        dy = self.scene.goal[1] - self.truth[1]
        glx = c * dx + s * dy
        gly = -s * dx + c * dy
        half = BEV_EXTENT / 2.0
        gu = (n - 1) / 2.0 + glx / res
        gv = (n - 1) / 2.0 - gly / res
        if 0 <= gu < n and 0 <= gv < n:
            draw.ellipse([gu - 2, gv - 2, gu + 2, gv + 2], fill=(40, 230, 90))
        else:
            ang = math.atan2(-gly, glx)
            r = half * 0.86
            ax = (n - 1) / 2.0 + r * math.cos(ang) / res
            ay = (n - 1) / 2.0 + r * math.sin(ang) / res
            bx = (n - 1) / 2.0 + (r - 1.2) * math.cos(ang) / res
            by = (n - 1) / 2.0 + (r - 1.2) * math.sin(ang) / res
            draw.line([bx, by, ax, ay], fill=(40, 230, 90), width=2)
            draw.ellipse([ax - 1, ay - 1, ax + 1, ay + 1], fill=(40, 230, 90))

        # Robot footprint at the image centre.
        draw.rectangle([n / 2 - 2, n / 2 - 3, n / 2 + 2, n / 2 + 3], fill=(30, 170, 255))
        draw.polygon([(n / 2, n / 2 - 5), (n / 2 - 2, n / 2 - 2), (n / 2 + 2, n / 2 - 2)],
                     fill=(255, 230, 40))
        out = np.asarray(pil, dtype=np.uint8)
        return out

    def depth_image(self, ranges: Optional[np.ndarray] = None) -> Tuple[np.ndarray, np.ndarray]:
        """Forward RGB-D pair (48x64) derived from the 360-degree scan.

        The horizontal field of view is 90 degrees.  Rows model a vertical
        field of view of 50 degrees; depth is scaled by 1/cos(pitch), which is
        the correct range to a vertical surface for a pinhole camera.
        """
        if ranges is None:
            ranges = self.scan()
        width, height = 64, 48
        fov = math.radians(90.0)
        ang = np.linspace(-fov / 2, fov / 2, width)
        idx = ((ang % (2 * np.pi)) / (2 * np.pi) * LIDAR_BEAMS).astype(np.int32) % LIDAR_BEAMS
        col = np.asarray(ranges, dtype=np.float64)[idx]
        pitch = np.linspace(math.radians(25.0), math.radians(-25.0), height)
        depth = col[None, :] / np.maximum(0.2, np.cos(pitch))[:, None]
        depth = np.clip(depth, LIDAR_RANGE_MIN, LIDAR_RANGE_MAX).astype(np.float32)

        # Pseudo-colour: near = warm, far = cool, upper rows = sky.
        norm = np.clip(1.0 - depth / LIDAR_RANGE_MAX, 0.0, 1.0)
        rgb = np.zeros((height, width, 3), dtype=np.float32)
        rgb[..., 0] = 0.15 + 0.85 * norm
        rgb[..., 1] = 0.25 + 0.45 * norm
        rgb[..., 2] = 0.95 - 0.65 * norm
        sky = np.where(pitch[:, None] > math.radians(12.0), 1.0, 0.0)
        sky_color = np.array([0.35, 0.45, 0.62], dtype=np.float32)
        rgb = rgb * (1.0 - sky[..., None]) + sky_color[None, None, :] * sky[..., None]
        if self.noise:
            rgb += self.rng.normal(0.0, 0.01, rgb.shape)
        rgb8 = np.clip(rgb * 255.0, 0, 255).astype(np.uint8)
        return rgb8, depth

    def sensors(self) -> Dict[str, np.ndarray]:
        ranges = self.scan()
        rgb, depth = self.depth_image(ranges)
        return {"scan": ranges, "rgb": rgb, "depth": depth, "bev": self.bev_image()}

    # ------------------------------------------------------------------ misc
    def scan_points(self, pose: Optional[Sequence[float]] = None,
                    ranges: Optional[np.ndarray] = None) -> np.ndarray:
        if ranges is None:
            ranges = self.scan()
        if pose is None:
            pose = self.truth
        a = pose[2] + self._angles
        pts = np.column_stack([ranges * np.cos(a), ranges * np.sin(a)])
        c, s = math.cos(pose[2]), math.sin(pose[2])
        return np.column_stack([c * pts[:, 0] - s * pts[:, 1] + pose[0],
                                s * pts[:, 0] + c * pts[:, 1] + pose[1]])

    @property
    def truth_pose(self) -> np.ndarray:
        return self.truth.copy()

    @property
    def odom_pose(self) -> np.ndarray:
        return self.odom.copy()
