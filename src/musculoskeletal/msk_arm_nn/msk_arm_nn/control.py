"""Muscle-level control for the musculoskeletal arm (MSK-Arm).

:class:`MuscleController` is the mission-level controller: it plans an
end-effector path in the workspace (A*), follows it with a computed-torque
(PD + gravity / Coriolis / damping compensation) law, and maps the desired joint
torques to the four Hill-type muscle activations through the (pseudo-inverse of
the) activation map, with a small balanced co-contraction regulariser.

The command returned by :meth:`MuscleController.update` is exactly the
simulator command ``(u_shoulder_flex, u_shoulder_ext, u_elbow_flex,
u_elbow_ext)`` in ``[0, 1]``, so ``sim.step(controller.update(...), dt)`` is the
intended usage.

Arm model (same constants as :mod:`sim`)::

    tau = B(q) @ a,      B[j, 2j] = +r Fmax fl,  B[j, 2j+1] = -r Fmax fl
    B a  = M(q) ddq + C(q, dq) + G(q) + D dq
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

import numpy as np

from . import config as C
from .geometry import wrap_angle
from .planning import (GridPlanner, densify_path, lookahead_point,
                       scene_planning_grid)

__all__ = ["PID", "SafetyFilter", "MuscleController", "MissionController",
           "inverse_kinematics", "forward_kinematics", "inertia_matrix",
           "coriolis", "gravity_torque", "activation_map",
           "torque_to_activation"]

_EPS = 1e-9


# ------------------------------------------------------------- rigid body
def forward_kinematics(q: Sequence[float]) -> np.ndarray:
    """End-effector position ``(x, y)`` for joint angles ``(q1, q2)``."""
    q1, q2 = float(q[0]), float(q[1])
    x = C.L1 * math.cos(q1) + C.L2 * math.cos(q1 + q2)
    y = C.L1 * math.sin(q1) + C.L2 * math.sin(q1 + q2)
    return np.array([x, y], dtype=np.float64)


def elbow_position(q: Sequence[float]) -> np.ndarray:
    q1 = float(q[0])
    return np.array([C.L1 * math.cos(q1), C.L1 * math.sin(q1)],
                    dtype=np.float64)


def inverse_kinematics(x: float, y: float) -> Optional[np.ndarray]:
    """Elbow-up analytic IK; ``None`` when the target is out of reach."""
    d2 = x * x + y * y
    denom = 2.0 * C.L1 * C.L2
    cos_q2 = (d2 - C.L1 * C.L1 - C.L2 * C.L2) / denom
    if cos_q2 < -1.0 - 1e-9 or cos_q2 > 1.0 + 1e-9:
        return None
    q2 = math.acos(max(-1.0, min(1.0, cos_q2)))
    q1 = math.atan2(y, x) - math.atan2(C.L2 * math.sin(q2),
                                       C.L1 + C.L2 * math.cos(q2))
    q1 = float(np.clip(q1, C.Q1_MIN, C.Q1_MAX))
    q2 = float(np.clip(q2, C.Q2_MIN, C.Q2_MAX))
    return np.array([q1, q2], dtype=np.float64)


def stretch_joints(x: float, y: float) -> np.ndarray:
    """Closest full-extension joint configuration towards ``(x, y)``."""
    ang = math.atan2(float(y), float(x))
    q1 = float(np.clip(ang, C.Q1_MIN, C.Q1_MAX))
    return np.array([q1, 0.0], dtype=np.float64)


def inertia_matrix(q: Sequence[float]) -> np.ndarray:
    """2x2 mass matrix ``M(q)`` of the two-link arm."""
    lc1 = C.L1 / 2.0
    lc2 = C.L2 / 2.0
    q2 = float(q[1])
    c2 = math.cos(q2)
    m11 = (C.I1 + C.I2 + C.M1 * lc1 ** 2
           + C.M2 * (C.L1 ** 2 + lc2 ** 2 + 2.0 * C.L1 * lc2 * c2))
    m12 = C.I2 + C.M2 * (lc2 ** 2 + C.L1 * lc2 * c2)
    m22 = C.I2 + C.M2 * lc2 ** 2
    return np.array([[m11, m12], [m12, m22]], dtype=np.float64)


def coriolis(q: Sequence[float], dq: Sequence[float]) -> np.ndarray:
    """Coriolis/centrifugal vector ``C(q, dq)``."""
    lc2 = C.L2 / 2.0
    q2 = float(q[1])
    dq1, dq2 = float(dq[0]), float(dq[1])
    h = -C.M2 * C.L1 * lc2 * math.sin(q2)
    return np.array([h * (2.0 * dq1 * dq2 + dq2 * dq2),
                     -h * dq1 * dq1], dtype=np.float64)


def gravity_torque(q: Sequence[float]) -> np.ndarray:
    """Gravity vector ``G(q)``."""
    lc1 = C.L1 / 2.0
    lc2 = C.L2 / 2.0
    q1, q2 = float(q[0]), float(q[1])
    g1 = ((C.M1 * lc1 + C.M2 * C.L1) * C.GRAVITY * math.cos(q1)
          + C.M2 * lc2 * C.GRAVITY * math.cos(q1 + q2))
    g2 = C.M2 * lc2 * C.GRAVITY * math.cos(q1 + q2)
    return np.array([g1, g2], dtype=np.float64)


def damping_torque(dq: Sequence[float]) -> np.ndarray:
    """Joint viscous damping ``D dq`` (compensated by the controller)."""
    return np.array([C.DAMP1 * float(dq[0]),
                     C.DAMP2 * float(dq[1])], dtype=np.float64)


def activation_map(q: Sequence[float]) -> np.ndarray:
    """2x4 activation matrix: ``tau = B(q) @ a`` (force-length included)."""
    b = np.zeros((2, 4), dtype=np.float64)
    for i, (_name, fmax, l0, r, sign) in enumerate(C.MUSCLES):
        joint = 0 if i < 2 else 1
        length = l0 + sign * r * float(q[joint])
        fl = math.exp(-((length - l0) / (0.5 * l0)) ** 2)
        b[joint, i] = sign * r * fmax * fl
    return b


def torque_to_activation(tau: Sequence[float], b: np.ndarray,
                         co_activation: float = 0.02) -> np.ndarray:
    """Map desired joint torques to non-negative muscle activations.

    Each joint has an antagonist pair (``b[j, 2j] > 0 > b[j, 2j+1]``).  The
    torque is produced by the muscle in the required direction; a small balanced
    co-contraction is added on both muscles (zero net torque, higher joint
    stiffness -- the co-activation regulariser).  Activations clip to ``[0, 1]``.
    """
    a = np.zeros(4, dtype=np.float64)
    for j in (0, 1):
        bf = float(b[j, 2 * j])
        be = float(b[j, 2 * j + 1])
        if bf <= _EPS or be >= -_EPS:
            continue
        t = float(tau[j])
        if co_activation > 0.0:
            wf = abs(be) / (abs(bf) + abs(be))
            we = abs(bf) / (abs(bf) + abs(be))
            a[2 * j] += co_activation * wf
            a[2 * j + 1] += co_activation * we
        if t >= 0.0:
            a[2 * j] += t / bf
        else:
            a[2 * j + 1] += t / be
    return np.clip(a, 0.0, 1.0)


class PID:
    """Discrete PID controller with output and integral clamping."""

    def __init__(self, kp: float, ki: float, kd: float,
                 limit: Optional[float] = None,
                 i_limit: Optional[float] = None) -> None:
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.limit = None if limit is None else abs(float(limit))
        self.i_limit = None if i_limit is None else abs(float(i_limit))
        self.reset()

    def reset(self) -> None:
        self._integral = 0.0
        self._prev_error: Optional[float] = None

    def update(self, error: float, dt: float) -> float:
        error = float(error)
        dt = max(float(dt), 1e-6)
        if self._prev_error is None:
            derivative = 0.0
        else:
            derivative = (error - self._prev_error) / dt
        self._prev_error = error
        self._integral += error * dt
        if self.i_limit is not None:
            self._integral = max(-self.i_limit, min(self.i_limit, self._integral))
        out = self.kp * error + self.ki * self._integral + self.kd * derivative
        if self.limit is not None:
            out = max(-self.limit, min(self.limit, out))
        return float(out)


class SafetyFilter:
    """Compatibility shim for the shared template.

    The arm has no lidar; this filter simply clamps a 4-muscle activation
    command to ``[0, 1]`` (kept so template nodes importing it keep working).
    """

    def __init__(self, *args, **kwargs) -> None:
        pass

    def apply(self, cmd: Sequence[float], scan=None, beam_angles=None
              ) -> np.ndarray:
        out = np.asarray(cmd, dtype=np.float64).ravel()[:4].copy()
        if out.size < 4:
            out = np.pad(out, (0, 4 - out.size), constant_values=0.0)
        return np.clip(out, 0.0, 1.0)

    def frontal_clearance(self, scan=None, beam_angles=None) -> float:
        return float(C.CAM_RANGE)


class MuscleController:
    """Mission-level controller for the musculoskeletal arm.

    Parameters
    ----------
    scene_or_map:
        :class:`scenes.Scene` (its workspace occupancy is used) or a 2-D boolean
        blocked grid over the workspace.
    resolution:
        Workspace planning grid resolution (m).
    lookahead:
        End-effector path lookahead (m) used to generate the joint target.
    kp, kd:
        Joint-space proportional / derivative gains of the computed-torque law.
    co_activation:
        Balanced co-contraction regulariser added to both antagonist muscles.
    """

    def __init__(self, scene_or_map,
                 resolution: float = C.PLAN_RESOLUTION,
                 lookahead: float = 0.05,
                 kp: float = 60.0, kd: float = 14.0,
                 co_activation: float = 0.02) -> None:
        scene = None
        if hasattr(scene_or_map, "occupancy") or hasattr(scene_or_map, "coarse_map"):
            scene = scene_or_map
            base = scene_planning_grid(scene, resolution)
        else:
            base = np.asarray(scene_or_map, dtype=bool)
        self.scene = scene
        self.resolution = float(resolution)
        self.lookahead = float(lookahead)
        self.kp = float(kp)
        self.kd = float(kd)
        self.co_activation = float(co_activation)
        self._base_grid = base
        self._radii = (max(C.EE_RADIUS, C.INFLATION_RADIUS),
                       C.EE_RADIUS + 0.01)
        self._planners: List[GridPlanner] = []
        self.planner = self._get_planner(0)
        self.targets = list(getattr(scene, "targets", []) or [])
        self.reset()

    # ---------------------------------------------------------------- helpers
    def _get_planner(self, index: int) -> GridPlanner:
        while len(self._planners) <= index:
            radius = self._radii[min(index, len(self._radii) - 1)]
            self._planners.append(GridPlanner(
                self._base_grid, self.resolution, radius / self.resolution))
        return self._planners[index]

    def reset(self, pose: Optional[Sequence[float]] = None) -> None:
        """Reset the per-episode state (path, counters and estimated joints)."""
        self.path: List[Tuple[float, float]] = []
        self.replans = 0
        self.success = False
        self.reached = 0
        self.target_index = 0
        self.target_ee = np.zeros(2, dtype=np.float64)
        self.reach_error = float("inf")
        self.hold_time = 0.0
        self.last_tau = np.zeros(2, dtype=np.float64)
        self.last_activation = np.zeros(4, dtype=np.float64)
        self._goal: Optional[Tuple[float, float]] = None
        self._q_prev: Optional[np.ndarray] = None
        self._dq = np.zeros(2, dtype=np.float64)
        self._progress_hold = 0.0
        self._time = 0.0
        self._time_step = 0.02
        if pose is not None:
            pose = np.asarray(pose, dtype=np.float64).ravel()
            q = inverse_kinematics(float(pose[0]), float(pose[1]))
            if q is not None:
                self._q_prev = q

    def _joints_from_pose(self, x: float, y: float,
                          q_sensor: Optional[Sequence[float]] = None,
                          dq_sensor: Optional[Sequence[float]] = None
                          ) -> Tuple[np.ndarray, np.ndarray]:
        """Recover joint angles/velocities (analytic IK + filtered difference)."""
        if q_sensor is not None:
            q = np.asarray(q_sensor, dtype=np.float64).ravel()[:2].copy()
        else:
            q_ik = inverse_kinematics(x, y)
            if q_ik is None:
                q = (self._q_prev.copy() if self._q_prev is not None
                     else np.array(self.scene.start_q if self.scene is not None
                                   else (0.2, 0.9), dtype=np.float64))
            else:
                q = q_ik
        if dq_sensor is not None:
            dq = np.asarray(dq_sensor, dtype=np.float64).ravel()[:2].copy()
        elif self._q_prev is None:
            dq = np.zeros(2, dtype=np.float64)
        else:
            raw = (q - self._q_prev) / max(self._time_step, 1e-4)
            dq = 0.6 * self._dq + 0.4 * raw
            dq = np.clip(dq, -12.0, 12.0)
        self._q_prev = np.array(q, dtype=np.float64, copy=True)
        self._dq = np.array(dq, dtype=np.float64, copy=True)
        return np.array(q, dtype=np.float64), np.array(dq, dtype=np.float64)

    def _path_distance(self, x: float, y: float) -> float:
        if not self.path:
            return float("inf")
        best = float("inf")
        pts = self.path
        for j in range(len(pts) - 1):
            ax, ay = pts[j]
            bx, by = pts[j + 1]
            abx, aby = bx - ax, by - ay
            seg2 = abx * abx + aby * aby
            t = 0.0 if seg2 <= 1e-12 else \
                max(0.0, min(1.0, ((x - ax) * abx + (y - ay) * aby) / seg2))
            best = min(best, math.hypot(ax + t * abx - x, ay + t * aby - y))
        return best

    def _replan(self, x: float, y: float, goal_xy: Tuple[float, float]) -> None:
        path = None
        for index in range(len(self._radii)):
            planner = self._get_planner(index)
            path = planner.plan((float(x), float(y)), goal_xy)
            if path:
                self.planner = planner
                break
        self.path = densify_path(path) if path else []
        self._goal = (float(goal_xy[0]), float(goal_xy[1]))
        self.replans += 1

    def _needs_replan(self, x: float, y: float,
                      goal_xy: Tuple[float, float]) -> bool:
        if self._goal is None or not self.path:
            return True
        if math.hypot(goal_xy[0] - self._goal[0],
                      goal_xy[1] - self._goal[1]) > 1e-4:
            return True
        if math.hypot(goal_xy[0] - x, goal_xy[1] - y) < 0.08:
            return False
        return self._path_distance(x, y) > 0.06

    def _update_progress(self, x: float, y: float, dt: float) -> None:
        if not self.targets:
            return
        idx = min(self.target_index, len(self.targets) - 1)
        tx, ty = float(self.targets[idx][0]), float(self.targets[idx][1])
        if math.hypot(x - tx, y - ty) <= C.TARGET_TOLERANCE:
            self._progress_hold += dt
            if self._progress_hold >= C.HOLD_TIME:
                self._progress_hold = 0.0
                self.reached = min(self.reached + 1, len(self.targets))
                if self.target_index + 1 < len(self.targets):
                    self.target_index += 1
                else:
                    self.success = True
        else:
            self._progress_hold = max(0.0, self._progress_hold - dt)

    # ----------------------------------------------------------------- update
    def update(self, pose: Sequence[float], goal: Sequence[float],
               scan=None, beam_angles=None, dt: float = 0.02,
               q: Optional[Sequence[float]] = None,
               dq: Optional[Sequence[float]] = None) -> np.ndarray:
        """Compute the four muscle activation commands for ``MskArmSim.step``.

        ``pose`` is the end-effector pose from :meth:`sim.MskArmSim.pose`,
        ``goal`` the current target (``[x, y]`` or ``[x, y, 0]``; pass
        ``sim.goal()`` so multi-target scenes advance automatically).  ``scan``
        and ``beam_angles`` are accepted for template compatibility and ignored.
        ``q``/``dq`` are optional joint-sensor values (recovered by analytical
        IK / finite differences when omitted).
        """
        pose = np.asarray(pose, dtype=np.float64).ravel()
        goal = np.asarray(goal, dtype=np.float64).ravel()
        dt = max(float(dt), 1e-4)
        self._time_step = dt
        self._time += dt
        x = float(pose[0])
        y = float(pose[1])
        gx = float(goal[0])
        gy = float(goal[1])
        goal_xy = (gx, gy)
        self.target_ee = np.array([gx, gy], dtype=np.float64)
        self.reach_error = math.hypot(gx - x, gy - y)

        q_cur, dq_cur = self._joints_from_pose(x, y, q_sensor=q, dq_sensor=dq)

        # ---- plan / re-plan the end-effector path
        if self._needs_replan(x, y, goal_xy):
            self._replan(x, y, goal_xy)

        # ---- end-effector target from the path (lookahead) or the goal itself
        if self.path:
            tx, ty = lookahead_point(self.path, x, y, self.lookahead)
            if self.reach_error <= self.lookahead:
                tx, ty = gx, gy
        else:
            tx, ty = gx, gy
        q_des_raw = inverse_kinematics(tx, ty)
        if q_des_raw is None:
            q_des_raw = stretch_joints(tx, ty)
        else:
            q_des_raw = np.array(q_des_raw, dtype=np.float64)

        # ---- inside the tolerance: hold precisely on the goal
        if self.reach_error <= C.TARGET_TOLERANCE:
            q_goal = inverse_kinematics(gx, gy)
            if q_goal is None:
                q_goal = stretch_joints(gx, gy)
            q_des_raw = np.array(q_goal, dtype=np.float64)
            self.hold_time += dt
        else:
            self.hold_time = 0.0

        # ---- shape the joint reference: bounded rate, gentle elbow approach
        # (hitting the q2 = 0 joint limit transfers momentum to the shoulder,
        #  which is what makes the arm overshoot near full extension)
        max_step = np.array([2.0 * dt, 2.5 * dt], dtype=np.float64)
        q_des = q_cur + np.clip(q_des_raw - q_cur, -max_step, max_step)
        if q_des_raw[1] < q_cur[1] and q_cur[1] < 0.35:
            elbow_rate = min(2.0, 0.15 + 3.0 * q_cur[1])
            q_des[1] = max(0.0, q_cur[1] - elbow_rate * dt)

        # ---- computed torque: PD + Coriolis + gravity + damping compensation
        kp_eff = self.kp
        kd_eff = self.kd
        if q_cur[1] < 0.20:
            # close to full extension (kinematic singularity): soft position gain
            # and strong velocity damping
            kp_eff *= 0.6
            kd_eff *= 2.0
        m_mat = inertia_matrix(q_cur)
        c_vec = coriolis(q_cur, dq_cur)
        g_vec = gravity_torque(q_cur)
        d_vec = damping_torque(dq_cur)
        error = q_des - q_cur
        tau = m_mat @ (kp_eff * error - kd_eff * dq_cur) + c_vec + g_vec + d_vec

        # ---- map to muscle activations (with per-joint saturation)
        b_mat = activation_map(q_cur)
        for j in (0, 1):
            limit = 0.95 * max(abs(float(b_mat[j, 2 * j])),
                               abs(float(b_mat[j, 2 * j + 1])))
            tau[j] = float(np.clip(tau[j], -limit, limit))
        activation = torque_to_activation(tau, b_mat, self.co_activation)

        self.last_tau = np.array(tau, dtype=np.float64)
        self.last_activation = np.array(activation, dtype=np.float64)
        self._update_progress(x, y, dt)
        return self.last_activation


# The task template / ROS nodes use the name ``MissionController``.
MissionController = MuscleController
