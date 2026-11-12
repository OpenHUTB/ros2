"""Lightweight quadrotor waypoint environment for DRL training.

This is a refactor of the PyBullet environment in the reference project
``zhaoran2000/DRL-drones`` (``Sol/Model/Environments/PBDroneEnv.py``).  The
reference simulation is physically rich but heavy and hard to deploy inside a
ROS graph; this module keeps the *interface* that matters for reinforcement
learning -- 12-D kinematic state, normalised continuous action, multi-waypoint
rewards -- and replaces the physics backend with a deterministic first-order
velocity model plus analytic cylinder ray-casting.  Training is therefore fast
enough to run on a laptop CPU, while the observation/action layout is shared
verbatim with the ROS node (``drl_nav_node.py``).

中文说明：本文件把参考仓库中基于 PyBullet 的 ``PBDroneEnv`` 重构为一个轻量、可
快速训练的 Gymnasium 环境。动力学采用一阶速度响应 + 圆柱解析求交，保留参考实现的
多航点奖励结构（距离塑形、进度奖励、朝向奖励、平滑惩罚、航点/终点奖励、碰撞惩罚），
观测与动作定义与 ROS 部署节点完全一致。

Developer: 汤湘粤 (学号: 2535030067)
"""

from __future__ import annotations

import math
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

import numpy as np

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError as exc:  # pragma: no cover - training-only dependency
    raise ImportError(
        "drone_env.py needs gymnasium; install it with "
        "`pip install gymnasium numpy pyyaml`"
    ) from exc

from common import (
    DEFAULT_CONFIG_PATH,
    action_to_velocity,
    apply_altitude_hold,
    body_to_world,
    build_observation,
    load_config,
    observation_dim,
    observation_layout,
    path_length,
    raycast_histogram,
    world_to_body,
    wrap_angle,
)

GRAVITY = 9.81


class DronePathEnv(gym.Env):
    """Multi-waypoint drone navigation environment.

    Observation (default 28-D)
        goal relative position (3) + rpy (3) + body velocity (3) +
        body angular velocity (3) + 16-sector lidar histogram.
    Action (4-D, ``[-1, 1]``)
        body-frame ``(vx, vy, vz, yaw_rate)`` scaled by the configured limits.
    """

    metadata = {"render_modes": []}

    def __init__(self,
                 cfg: Optional[Mapping[str, Any]] = None,
                 render_mode: Optional[str] = None) -> None:
        super().__init__()
        self.cfg = dict(cfg) if cfg is not None else load_config(DEFAULT_CONFIG_PATH)
        self.render_mode = render_mode

        env_cfg = self.cfg["env"]
        self.dt = float(env_cfg["dt"])
        self.max_steps = int(env_cfg["max_steps"])
        self.tolerance = float(env_cfg["waypoint_tolerance"])
        self.drone_radius = float(env_cfg["drone_radius"])
        self.arena = np.asarray(env_cfg["arena"], dtype=np.float64)
        self.start = np.asarray(env_cfg["start"], dtype=np.float64)
        self.start_yaw = float(env_cfg["start_yaw"])
        self.randomize_start = bool(env_cfg.get("randomize_start", False))
        self.start_noise_xy = float(env_cfg.get("start_noise_xy", 0.0))
        self.start_noise_yaw = float(env_cfg.get("start_noise_yaw", 0.0))
        self.obstacles = [tuple(float(v) for v in item)
                          for item in env_cfg.get("obstacles", [])]

        self.waypoints = np.asarray(self.cfg["goal"]["waypoints"], dtype=np.float64)
        if self.waypoints.ndim != 2 or self.waypoints.shape[1] != 3:
            raise ValueError("goal.waypoints must be a list of [x, y, z] triples")
        if self.waypoints.shape[0] == 0:
            raise ValueError("goal.waypoints must contain at least one waypoint")

        self.observation_cfg = self.cfg["observation"]
        self.sectors = int(self.observation_cfg["sectors"])
        self.lidar_range = float(self.observation_cfg["lidar_range"])
        self.observation_dim = observation_dim(self.cfg)

        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(4,), dtype=np.float32
        )
        observation_low = np.full(self.observation_dim, -1.0, dtype=np.float32)
        observation_low[observation_layout(self.sectors)["lidar"]] = 0.0
        self.observation_space = spaces.Box(
            low=observation_low,
            high=np.ones(self.observation_dim, dtype=np.float32),
            dtype=np.float32,
        )

        # Internal state (initialised in reset()).
        self.position = self.start.copy()
        self.yaw = self.start_yaw
        self.rpy = np.zeros(3, dtype=np.float64)
        self.velocity_world = np.zeros(3, dtype=np.float64)
        self.angular_velocity_body = np.zeros(3, dtype=np.float64)
        self.histogram = np.ones(self.sectors, dtype=np.float32)
        self.waypoint_index = 0
        self.steps = 0
        self.prev_distance = 0.0
        self.prev_command = np.zeros(4, dtype=np.float64)
        self.travelled = 0.0
        self.reached = 0
        self._trajectory = [self.position.copy()]

    # ------------------------------------------------------------------
    # Gymnasium API
    # ------------------------------------------------------------------

    def reset(self,
              *,
              seed: Optional[int] = None,
              options: Optional[Dict[str, Any]] = None) -> Tuple[np.ndarray, Dict[str, Any]]:
        """Reset the vehicle to the configured start (optionally randomised)."""
        super().reset(seed=seed)
        options = options or {}

        start = np.asarray(options.get("start", self.start), dtype=np.float64)
        yaw = float(options.get("yaw", self.start_yaw))
        if self.randomize_start and "start" not in options:
            noise_xy = self.np_random.uniform(
                -self.start_noise_xy, self.start_noise_xy, size=2
            )
            start = start + np.array([noise_xy[0], noise_xy[1], 0.0])
            yaw += float(self.np_random.uniform(
                -self.start_noise_yaw, self.start_noise_yaw
            ))

        self.position = start.astype(np.float64)
        self.yaw = wrap_angle(yaw)
        self.rpy = np.array([0.0, 0.0, self.yaw], dtype=np.float64)
        self.velocity_world = np.zeros(3, dtype=np.float64)
        self.angular_velocity_body = np.zeros(3, dtype=np.float64)
        self.waypoint_index = 0
        self.steps = 0
        self.reached = 0
        self.travelled = 0.0
        self.prev_command = np.zeros(4, dtype=np.float64)
        self.prev_distance = float(np.linalg.norm(self._goal() - self.position))
        self.histogram = self._scan()
        self._trajectory = [self.position.copy()]

        observation = self._observation()
        info = {
            "waypoint_index": self.waypoint_index,
            "distance": self.prev_distance,
        }
        return observation, info

    def step(self, action: Sequence[float]) -> Tuple[
            np.ndarray, float, bool, bool, Dict[str, Any]]:
        """Advance the first-order kinematic model by ``dt`` seconds."""
        env_cfg = self.cfg["env"]
        reward_cfg = env_cfg["reward"]
        normalized_action = np.clip(
            np.asarray(action, dtype=np.float64).reshape(-1), -1.0, 1.0
        )
        normalized_action = apply_altitude_hold(
            normalized_action, self.position, self._goal(), self.cfg
        )
        command = action_to_velocity(normalized_action, self.cfg)

        previous_position = self.position.copy()
        desired_world = body_to_world(
            [command[0], command[1], 0.0], self.yaw
        ) + np.array([0.0, 0.0, command[2]])
        tau = 0.15
        alpha = 1.0 - math.exp(-self.dt / tau)
        self.velocity_world += (desired_world - self.velocity_world) * alpha
        self.position = self.position + self.velocity_world * self.dt
        self.travelled += float(np.linalg.norm(self.position - previous_position))

        self.yaw = wrap_angle(self.yaw + command[3] * self.dt)

        velocity_body = world_to_body(self.velocity_world, self.yaw)
        acceleration_body = (command[0:3] - velocity_body) / tau
        new_roll = float(np.clip(
            math.atan2(acceleration_body[1], GRAVITY), -0.6, 0.6
        ))
        new_pitch = float(np.clip(
            -math.atan2(acceleration_body[0], GRAVITY), -0.6, 0.6
        ))
        self.angular_velocity_body = np.array([
            (new_roll - self.rpy[0]) / self.dt,
            (new_pitch - self.rpy[1]) / self.dt,
            command[3],
        ], dtype=np.float64)
        self.rpy = np.array([new_roll, new_pitch, self.yaw], dtype=np.float64)

        self.steps += 1
        self.histogram = self._scan()
        self._trajectory.append(self.position.copy())

        collision = self._collision()
        out_of_bounds = not self._inside_arena()
        distance = float(np.linalg.norm(self._goal() - self.position))
        reached_goal = distance <= self.tolerance

        reward = 0.0
        terminated = False
        if collision:
            reward += float(reward_cfg["collision_penalty"])
            terminated = True
        elif out_of_bounds:
            reward += float(reward_cfg["collision_penalty"])
            terminated = True
        elif reached_goal:
            self.reached += 1
            if self.waypoint_index >= len(self.waypoints) - 1:
                reward += float(reward_cfg["goal_bonus"])
                terminated = True
            else:
                reward += float(reward_cfg["waypoint_bonus"])
                self.waypoint_index += 1
                self.prev_distance = float(
                    np.linalg.norm(self._goal() - self.position)
                )
        else:
            goal_body = world_to_body(self._goal() - self.position, self.yaw)
            heading_error = abs(math.atan2(goal_body[1], goal_body[0]))
            delta_command = command - self.prev_command
            smoothness = float(np.dot(delta_command[0:3], delta_command[0:3]))
            smoothness += 0.1 * float(delta_command[3] ** 2)
            reward += float(reward_cfg["distance_gain"]) * math.exp(-2.0 * distance)
            reward += float(reward_cfg["progress_gain"]) * (
                self.prev_distance - distance
            )
            if heading_error > float(reward_cfg["heading_threshold"]):
                reward -= float(reward_cfg["heading_gain"])
            reward -= float(reward_cfg["smoothness_gain"]) * smoothness
            reward -= float(reward_cfg.get("altitude_gain", 0.0)) * abs(
                self.position[2] - self._goal()[2]
            )
            reward -= float(reward_cfg["time_penalty"])

        self.prev_distance = distance
        self.prev_command = command.copy()
        truncated = self.steps >= self.max_steps and not terminated

        observation = self._observation()
        info = {
            "success": bool(terminated and reached_goal and not collision
                            and not out_of_bounds),
            "collision": bool(collision or out_of_bounds),
            "waypoint_index": int(self.waypoint_index),
            "waypoints_reached": int(self.reached),
            "distance": distance,
            "path_length": self.travelled,
            "steps": self.steps,
        }
        return observation, float(reward), bool(terminated), bool(truncated), info

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _goal(self) -> np.ndarray:
        """Current target waypoint."""
        return self.waypoints[min(self.waypoint_index, len(self.waypoints) - 1)]

    def _scan(self) -> np.ndarray:
        """Lidar histogram at the current pose."""
        return raycast_histogram(
            self.position,
            self.yaw,
            self.obstacles,
            self.sectors,
            self.lidar_range,
            height_min=float(self.observation_cfg["obstacle_height_min"]),
        )

    def _observation(self) -> np.ndarray:
        """Build the shared observation vector."""
        return build_observation(
            self.position,
            self.rpy,
            world_to_body(self.velocity_world, self.yaw),
            self.angular_velocity_body,
            self._goal(),
            self.histogram,
            self.cfg,
        )

    def _collision(self) -> bool:
        """True when the drone intersects a cylinder."""
        x, y, z = self.position
        for ox, oy, radius, height in self.obstacles:
            if z <= height and math.hypot(x - ox, y - oy) < radius + self.drone_radius:
                return True
        return False

    def _inside_arena(self) -> bool:
        """True when the drone is inside the configured box."""
        x, y, z = self.position
        x_min, y_min, z_min, x_max, y_max, z_max = self.arena
        return (x_min <= x <= x_max and y_min <= y <= y_max
                and z_min <= z <= z_max)

    @property
    def trajectory(self) -> np.ndarray:
        """Visited positions, useful for offline evaluation/plots."""
        return np.asarray(self._trajectory, dtype=np.float64)

    def episode_summary(self) -> Dict[str, float]:
        """Aggregate metrics for the episode that just finished."""
        return {
            "steps": float(self.steps),
            "waypoints_reached": float(self.reached),
            "path_length": float(self.travelled),
            "path_length_3d": path_length(self._trajectory),
            "final_distance": float(np.linalg.norm(self._goal() - self.position)),
        }


def make_env(cfg: Optional[Mapping[str, Any]] = None) -> DronePathEnv:
    """Factory kept small so SB3 ``make_vec_env``-style code can reuse it."""
    return DronePathEnv(cfg)
