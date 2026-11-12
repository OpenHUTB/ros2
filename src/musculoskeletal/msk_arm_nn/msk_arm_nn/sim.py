"""Two-link musculoskeletal arm driven by four Hill-type muscles.

Model
-----
Planar two-link arm (shoulder q1, elbow q2) with point masses, joint damping and
gravity, actuated by two antagonist muscle pairs (shoulder flexor/extensor,
elbow flexor/extensor).  Muscle force is a Hill-type model:

    l  = l0 + sign * r * q                    (muscle length)
    fl = exp(-((l - l0) / (0.5 l0))**2)       (force-length)
    fv = 1 - k_v * (ldot / vmax)              (force-velocity)
    F  = a * Fmax * fl * fv,   a_dot = (u - a) / tau   (activation dynamics)
    tau_joint = sign * r * F

The joint accelerations follow the standard two-link rigid-body equations,
integrated semi-implicitly at 500 Hz.  Obstacles in the arm plane are checked
along the links; collisions are counted per 0.5 s window.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from . import config as C
from .geometry import wrap_angle
from .scenes import Scene, get_scene


class MskArmSim:
    def __init__(self, scene_name: str = "reach", seed: int = C.DEFAULT_SEED,
                 noise: bool = True):
        self.scene: Scene = get_scene(scene_name)
        self.rng = np.random.default_rng(seed)
        self.noise = bool(noise)
        self._mu = C.MUSCLES
        self.lc1 = C.L1 / 2.0
        self.lc2 = C.L2 / 2.0
        self.q_sigma = C.Q_SIGMA if self.noise else 0.0
        self.a_sigma = C.A_SIGMA if self.noise else 0.0
        self.cam_sigma = 0.02 if self.noise else 0.0
        self.reset()

    # ------------------------------------------------------------------ reset
    def reset(self, pose: Optional[Sequence[float]] = None) -> None:
        if pose is None:
            self.q = np.array(self.scene.start_q, dtype=np.float64)
        else:
            # pose is the end-effector position; recover joints by IK
            q = self.ik(float(pose[0]), float(pose[1]))
            self.q = np.array(q if q is not None else self.scene.start_q, dtype=np.float64)
        self.dq = np.zeros(2, dtype=np.float64)
        self.a = np.zeros(4, dtype=np.float64)      # muscle activations
        self.u = np.zeros(4, dtype=np.float64)      # activation commands
        self.time = 0.0
        self.reached = 0
        self.target_index = 0
        self.success = False
        self.collisions = 0
        self.max_torque = 0.0
        self.effort = 0.0
        self.ee_trail: List[Tuple[float, float]] = []
        self._hold = 0.0
        self._contact = False
        self._cooldown = 0.0
        self._update_ee()
        self.trail = []

    # -------------------------------------------------------------- kinematics
    def ee(self) -> np.ndarray:
        q1, q2 = float(self.q[0]), float(self.q[1])
        x = C.L1 * math.cos(q1) + C.L2 * math.cos(q1 + q2)
        y = C.L1 * math.sin(q1) + C.L2 * math.sin(q1 + q2)
        return np.array([x, y], dtype=np.float64)

    def elbow(self) -> np.ndarray:
        q1 = float(self.q[0])
        return np.array([C.L1 * math.cos(q1), C.L1 * math.sin(q1)], dtype=np.float64)

    def _update_ee(self) -> None:
        self._ee = self.ee()
        self.ee_trail.append((float(self._ee[0]), float(self._ee[1])))
        if len(self.ee_trail) > 4000:
            self.ee_trail = self.ee_trail[-2000:]

    def ik(self, x: float, y: float) -> Optional[Tuple[float, float]]:
        """Elbow-up analytic IK for a 2-link planar arm."""
        d2 = x * x + y * y
        cos_q2 = (d2 - C.L1 * C.L1 - C.L2 * C.L2) / (2.0 * C.L1 * C.L2)
        if cos_q2 < -1.0 or cos_q2 > 1.0:
            return None
        q2 = math.acos(max(-1.0, min(1.0, cos_q2)))
        q1 = math.atan2(y, x) - math.atan2(C.L2 * math.sin(q2), C.L1 + C.L2 * math.cos(q2))
        q1 = float(np.clip(q1, C.Q1_MIN, C.Q1_MAX))
        q2 = float(np.clip(q2, C.Q2_MIN, C.Q2_MAX))
        return q1, q2

    # ------------------------------------------------------------------- step
    def step(self, cmd: Sequence[float], dt: float = 1.0 / C.CONTROL_HZ) -> None:
        """Advance ``dt`` with ``cmd = (u_shoulder_flex, u_shoulder_ext,
        u_elbow_flex, u_elbow_ext)`` muscle activation commands in [0,1]."""
        cmd = np.asarray(cmd, dtype=np.float64).ravel()
        if cmd.size < 4:
            cmd = np.pad(cmd, (0, 4 - cmd.size), constant_values=0.0)
        self.u = np.clip(cmd[:4], 0.0, 1.0)
        n_sub = max(1, int(round(dt / C.PHYS_DT)))
        for _ in range(n_sub):
            self._physics_step(C.PHYS_DT)
        self.time += dt

    def _muscle_torques(self) -> np.ndarray:
        q = self.q
        dq = self.dq
        torque = np.zeros(2, dtype=np.float64)
        self.forces = np.zeros(4, dtype=np.float64)
        for i, (name, fmax, l0, r, sign) in enumerate(self._mu):
            joint = 0 if i < 2 else 1
            length = l0 + sign * r * float(q[joint])
            ldot = sign * r * float(dq[joint])
            fl = math.exp(-((length - l0) / (0.5 * l0)) ** 2)
            fv = float(np.clip(1.0 / (1.0 + abs(ldot) / 0.25), 0.05, 1.0))
            force = float(self.a[i]) * fmax * fl * fv
            self.forces[i] = force
            torque[joint] += sign * r * force
        return torque

    def _physics_step(self, dt: float) -> None:
        # activation dynamics
        self.a += (self.u - self.a) * dt / C.MUSCLE_TAU
        self.a = np.clip(self.a, 0.0, 1.0)

        q1, q2 = float(self.q[0]), float(self.q[1])
        dq1, dq2 = float(self.dq[0]), float(self.dq[1])

        # mass matrix
        c2 = math.cos(q2)
        s2 = math.sin(q2)
        m11 = (C.I1 + C.I2 + C.M1 * self.lc1 ** 2
               + C.M2 * (C.L1 ** 2 + self.lc2 ** 2 + 2 * C.L1 * self.lc2 * c2))
        m12 = C.I2 + C.M2 * (self.lc2 ** 2 + C.L1 * self.lc2 * c2)
        m22 = C.I2 + C.M2 * self.lc2 ** 2
        h = -C.M2 * C.L1 * self.lc2 * s2
        c1 = h * (2 * dq1 * dq2 + dq2 * dq2)
        c2t = -h * dq1 * dq1
        g1 = ((C.M1 * self.lc1 + C.M2 * C.L1) * C.GRAVITY * math.cos(q1)
              + C.M2 * self.lc2 * C.GRAVITY * math.cos(q1 + q2))
        g2 = C.M2 * self.lc2 * C.GRAVITY * math.cos(q1 + q2)

        tau_m = self._muscle_torques()
        tau = tau_m - np.array([C.DAMP1 * dq1, C.DAMP2 * dq2])
        self.max_torque = max(self.max_torque, float(np.max(np.abs(tau_m))))
        self.effort += float(np.sum(self.a ** 2)) * dt

        rhs1 = tau[0] - c1 - g1
        rhs2 = tau[1] - c2t - g2
        det = m11 * m22 - m12 * m12
        ddq1 = (m22 * rhs1 - m12 * rhs2) / det
        ddq2 = (m11 * rhs2 - m12 * rhs1) / det

        self.dq[0] += ddq1 * dt
        self.dq[1] += ddq2 * dt
        self.q[0] += self.dq[0] * dt
        self.q[1] += self.dq[1] * dt

        # joint limits
        for j, (lo, hi) in enumerate(((C.Q1_MIN, C.Q1_MAX), (C.Q2_MIN, C.Q2_MAX))):
            if self.q[j] < lo:
                self.q[j] = lo
                self.dq[j] = max(0.0, self.dq[j])
            elif self.q[j] > hi:
                self.q[j] = hi
                self.dq[j] = min(0.0, self.dq[j])

        self._update_ee()
        self._check_contact(dt)
        self._check_targets(dt)

    def _check_contact(self, dt: float) -> None:
        if self._cooldown > 0.0:
            self._cooldown -= dt
        base = (0.0, 0.0)
        elbow = tuple(self.elbow())
        wrist = (float(self._ee[0]), float(self._ee[1]))
        hit = (self.scene.segment_hits_obstacle(base, elbow)
               or self.scene.segment_hits_obstacle(elbow, wrist))
        if hit:
            if not self._contact and self._cooldown <= 0.0:
                self.collisions += 1
                self._cooldown = 0.5
            self._contact = True
            # damp the motion on impact
            self.dq *= 0.6
        else:
            self._contact = False

    def _check_targets(self, dt: float) -> None:
        if self.success:
            return
        targets = self.scene.targets
        if not targets:
            return
        tx, ty = targets[self.target_index]
        dist = math.hypot(self._ee[0] - tx, self._ee[1] - ty)
        if dist <= C.TARGET_TOLERANCE:
            self._hold += dt
            if self._hold >= C.HOLD_TIME:
                self.reached += 1
                self._hold = 0.0
                if self.target_index + 1 >= len(targets):
                    self.success = True
                else:
                    self.target_index += 1
        else:
            self._hold = max(0.0, self._hold - dt)

    # ----------------------------------------------------------------- sensors
    def pose(self) -> np.ndarray:
        ee = self._ee
        return np.array([ee[0], ee[1], 0.0, 0.0], dtype=np.float64)

    @property
    def speed(self) -> float:
        return float(np.hypot(*self.jacobian_velocity()))

    def jacobian_velocity(self) -> np.ndarray:
        q1, q2 = float(self.q[0]), float(self.q[1])
        dq1, dq2 = float(self.dq[0]), float(self.dq[1])
        j11 = -C.L1 * math.sin(q1) - C.L2 * math.sin(q1 + q2)
        j12 = -C.L2 * math.sin(q1 + q2)
        j21 = C.L1 * math.cos(q1) + C.L2 * math.cos(q1 + q2)
        j22 = C.L2 * math.cos(q1 + q2)
        return np.array([j11 * dq1 + j12 * dq2, j21 * dq1 + j22 * dq2])

    def joints(self) -> np.ndarray:
        q = self.q.copy()
        if self.noise:
            q += self.q_sigma * self.rng.normal(size=2)
        return q

    def muscle_activations(self) -> np.ndarray:
        a = self.a.copy()
        if self.noise:
            a += self.a_sigma * self.rng.normal(size=4)
        return np.clip(a, 0.0, 1.0)

    def muscle_forces(self) -> np.ndarray:
        return getattr(self, "forces", np.zeros(4)).copy()

    def depth_camera(self) -> np.ndarray:
        """Depth image (32x32) of the workspace plane around the arm base."""
        img = np.zeros((C.CAM_HEIGHT, C.CAM_WIDTH), dtype=np.float32)
        half = C.CAM_FOV / 2.0
        base_angle = 0.6           # camera looks slightly up towards the target area
        for col in range(C.CAM_WIDTH):
            ang = base_angle - half + C.CAM_FOV * (col + 0.5) / C.CAM_WIDTH
            dist = self._ray_distance(math.cos(ang), math.sin(ang), C.CAM_RANGE)
            prox = float(np.clip(1.0 - dist / C.CAM_RANGE, 0.0, 1.0))
            h = int(round(C.CAM_HEIGHT * prox))
            if h > 0:
                img[C.CAM_HEIGHT - h:, col] = 1.0
        for row in range(C.CAM_HEIGHT):
            img[row, :] = np.maximum(img[row, :], 0.05 + 0.10 * row / C.CAM_HEIGHT)
        if self.noise:
            img = np.clip(img + self.cam_sigma * self.rng.normal(size=img.shape), 0, 1)
        return img

    def camera(self) -> np.ndarray:
        depth = self.depth_camera()
        return np.stack([depth, depth, depth], axis=-1)

    def _ray_distance(self, dx: float, dy: float, max_range: float) -> float:
        step = 0.01
        dist = 0.05
        while dist < max_range:
            x, y = dx * dist, dy * dist
            if not self.scene.is_free(x, y, radius=0.006):
                return dist
            dist += step
        return max_range

    def gnss(self) -> np.ndarray:
        pos = self._ee.copy()
        if self.noise:
            pos += 0.002 * self.rng.normal(size=2)
        return pos

    def integrate_odometry(self, dt: float) -> np.ndarray:
        """Vision/proprioception-fused end-effector estimate (simple complement)."""
        self.odom_xy = getattr(self, "odom_xy", self._ee.copy())
        pred = self.odom_xy + self.jacobian_velocity() * dt
        meas = self.gnss()
        self.odom_xy = 0.8 * pred + 0.2 * meas
        return np.array([self.odom_xy[0], self.odom_xy[1], 0.0, 0.0])

    def imu(self) -> Tuple[float, float, float]:
        gyro = float(self.dq[0] + self.dq[1])
        return float(self.q[0]), float(self.q[1]), gyro

    # ---------------------------------------------------- dynamics for control
    def inertia_matrix(self) -> np.ndarray:
        q2 = float(self.q[1])
        c2 = math.cos(q2)
        m11 = (C.I1 + C.I2 + C.M1 * self.lc1 ** 2
               + C.M2 * (C.L1 ** 2 + self.lc2 ** 2 + 2 * C.L1 * self.lc2 * c2))
        m12 = C.I2 + C.M2 * (self.lc2 ** 2 + C.L1 * self.lc2 * c2)
        m22 = C.I2 + C.M2 * self.lc2 ** 2
        return np.array([[m11, m12], [m12, m22]])

    def coriolis(self) -> np.ndarray:
        q2 = float(self.q[1])
        dq1, dq2 = float(self.dq[0]), float(self.dq[1])
        h = -C.M2 * C.L1 * self.lc2 * math.sin(q2)
        return np.array([h * (2 * dq1 * dq2 + dq2 * dq2), -h * dq1 * dq1])

    def gravity_torque(self) -> np.ndarray:
        q1, q2 = float(self.q[0]), float(self.q[1])
        g1 = ((C.M1 * self.lc1 + C.M2 * C.L1) * C.GRAVITY * math.cos(q1)
              + C.M2 * self.lc2 * C.GRAVITY * math.cos(q1 + q2))
        g2 = C.M2 * self.lc2 * C.GRAVITY * math.cos(q1 + q2)
        return np.array([g1, g2])

    def activation_map(self) -> np.ndarray:
        """2x4 matrix B with tau = B @ a (force-length/velocity neglected)."""
        b = np.zeros((2, 4), dtype=np.float64)
        for i, (name, fmax, l0, r, sign) in enumerate(C.MUSCLES):
            joint = 0 if i < 2 else 1
            length = l0 + sign * r * float(self.q[joint])
            fl = math.exp(-((length - l0) / (0.5 * l0)) ** 2)
            b[joint, i] = sign * r * fmax * fl
        return b

    def max_activations(self) -> np.ndarray:
        return np.ones(4, dtype=np.float64)
    # ------------------------------------------------------------------ extras
    def goal(self) -> np.ndarray:
        targets = self.scene.targets
        idx = min(self.target_index, len(targets) - 1)
        tx, ty = targets[idx]
        return np.array([tx, ty, 0.0], dtype=np.float64)

    def scene_map(self, resolution: float = C.PLAN_RESOLUTION) -> np.ndarray:
        return self.scene.coarse_map(resolution)
