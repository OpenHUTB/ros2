"""Quadrotor-like kinematic model used by the closed-loop demos.

The model is deliberately simple (double integrator with a first-order velocity
lag and a yaw-rate limit) because the contribution of this project is the
neural perception/planning/control stack, not a high-fidelity flight dynamics
model.  The same interface is used by the ROS nodes, so replacing it with
PX4/Gazebo is a matter of swapping this class.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from .world import World


@dataclass
class DroneState:
    pos: np.ndarray
    vel: np.ndarray = field(default_factory=lambda: np.zeros(3))
    yaw: float = 0.0

    def copy(self) -> "DroneState":
        return DroneState(self.pos.copy(), self.vel.copy(), float(self.yaw))


class Drone:
    def __init__(self, world: World, max_speed: float = 1.5, tau: float = 0.28,
                 yaw_rate_max: float = 2.4, radius: float = 0.32, dt: float = 0.1):
        self.world = world
        self.max_speed = max_speed
        self.tau = tau
        self.yaw_rate_max = yaw_rate_max
        self.radius = radius
        self.dt = dt
        self.state = DroneState(world.start.copy())
        self.t = 0.0
        self.path_length = 0.0
        self.min_clearance = float("inf")
        self.max_accel = 0.0
        self._prev_vel = np.zeros(3)

    def reset(self):
        self.state = DroneState(self.world.start.copy())
        self.t = 0.0
        self.path_length = 0.0
        self.min_clearance = float("inf")
        self.max_accel = 0.0
        self._prev_vel = np.zeros(3)

    def step(self, cmd_body: np.ndarray) -> None:
        """``cmd_body`` = (vx, vy) expressed in the body frame [m/s]."""
        cmd = np.asarray(cmd_body, dtype=float).reshape(-1)
        vx, vy = float(cmd[0]), float(cmd[1])
        c, s = math.cos(self.state.yaw), math.sin(self.state.yaw)
        target = np.array([c * vx - s * vy, s * vx + c * vy, 0.0])
        speed = float(np.linalg.norm(target[:2]))
        if speed > self.max_speed:
            target[:2] *= self.max_speed / speed
        alpha = min(1.0, self.dt / self.tau)
        new_vel = self.state.vel + alpha * (target - self.state.vel)
        accel = (new_vel - self.state.vel) / max(self.dt, 1e-6)
        self.max_accel = max(self.max_accel, float(np.linalg.norm(accel)))
        self.state.vel = new_vel
        self.state.pos = self.state.pos + new_vel * self.dt
        self.state.pos[2] = self.world.flight_z
        sp = float(np.linalg.norm(new_vel[:2]))
        if sp > 0.08:
            desired_yaw = math.atan2(new_vel[1], new_vel[0])
            err = (desired_yaw - self.state.yaw + math.pi) % (2 * math.pi) - math.pi
            lim = self.yaw_rate_max * self.dt
            self.state.yaw += max(-lim, min(lim, err))
        self.t += self.dt
        self.path_length += float(np.linalg.norm(self.state.pos - self.world.start)) * 0.0
        self.path_length += float(np.linalg.norm(new_vel[:2])) * self.dt
        self.min_clearance = min(self.min_clearance, self.world.clearance(self.state.pos))
        self._prev_vel = new_vel.copy()

    # ------------------------------------------------------------------ status
    def reached(self, tol: float = 1.4) -> bool:
        return bool(np.linalg.norm(self.state.pos - self.world.goal) < tol)

    def collided(self) -> bool:
        return self.world.clearance(self.state.pos) < self.radius

    def out_of_bounds(self) -> bool:
        return not self.world.inside(self.state.pos, margin=0.5)

    def body_velocity(self) -> np.ndarray:
        c, s = math.cos(self.state.yaw), math.sin(self.state.yaw)
        v = self.state.vel
        return np.array([c * v[0] + s * v[1], -s * v[0] + c * v[1]])

    def to_world(self, v_body: np.ndarray) -> np.ndarray:
        c, s = math.cos(self.state.yaw), math.sin(self.state.yaw)
        return np.array([c * v_body[0] - s * v_body[1], s * v_body[0] + c * v_body[1]])

    def to_body(self, v_world: np.ndarray) -> np.ndarray:
        c, s = math.cos(self.state.yaw), math.sin(self.state.yaw)
        return np.array([c * v_world[0] + s * v_world[1], -s * v_world[0] + c * v_world[1]])
