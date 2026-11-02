"""Shared configuration for the offline environment and the ROS node.

This module deliberately has no PyBullet, Gymnasium or ROS imports.  The
navigator needs the task geometry and the observation dimensions at runtime,
but it must start on a machine that only has the course simulator installed
(AirSim or MuJoCo) and no offline verification dependencies.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class EnvConfig:
    """All tunable parameters of the navigation task."""

    # ---- timing ---------------------------------------------------------
    ctrl_freq: int = 30              # [Hz] RL / controller rate
    pyb_freq: int = 240              # [Hz] physics rate (ctrl_freq must divide it)
    max_steps: int = 900             # RL steps per episode (30 s)

    # ---- arena ----------------------------------------------------------
    arena_x: float = 12.6            # corridor length along +x
    arena_y: float = 3.2             # half width
    z_min: float = 0.30              # below this -> crash
    z_max: float = 3.20              # above this -> out of the arena

    # ---- gates ----------------------------------------------------------
    # The gates form a slalom rather than a straight line: a constant forward
    # velocity command must fail, otherwise the task degenerates into holding
    # a heading.
    n_gates: int = 3
    first_gate_x: float = 3.8
    gate_spacing: float = 3.6
    gate_altitude: float = 1.50
    gate_y_offsets: tuple = (0.0, 1.55, -1.55)   # lateral weave
    gate_z_offsets: tuple = (0.0, 0.45, -0.35)   # vertical weave
    gate_aperture: float = 0.58      # [m] inner half-height/half-width
    gate_bar: float = 0.055          # [m] half thickness of the frame bars

    # ---- obstacles ------------------------------------------------------
    n_static: int = 7
    n_dynamic: int = 4
    static_radius: tuple = (0.22, 0.42)
    dynamic_radius: tuple = (0.17, 0.28)
    dynamic_speed: tuple = (0.35, 0.95)   # [m/s] peak sweeping speed
    min_spawn_clearance: float = 1.30     # [m] keep the start region clear

    # ---- wind -----------------------------------------------------------
    wind_mean: tuple = (0.12, 0.0, 0.0)   # [m/s] steady component
    wind_ou_sigma: float = 0.30           # [m/s] Ornstein-Uhlenbeck noise scale
    wind_ou_theta: float = 0.9            # mean-reversion rate
    gust_prob: float = 0.020              # per-step probability of a gust burst
    gust_magnitude: tuple = (0.8, 2.4)    # [m/s] burst size range
    gust_duration: tuple = (3, 10)        # [steps] burst length

    # ---- vehicle / control ---------------------------------------------
    max_velocity: float = 2.6        # [m/s] action scaling and CBF ball radius
    max_yaw_rate: float = 2.0        # [rad/s]
    spawn_noise: float = 0.10        # [m] start position jitter
    drone_radius: float = 0.10       # [m] bounding sphere of the airframe
    use_cbf: bool = False            # apply the CBF filter to the action
    cbf_d_safe: float = 0.30         # [m] barrier offset
    cbf_gamma: float = 0.55          # [-]  discrete CBF gain
    # Effective braking authority used by the barrier.  It is lower than the
    # controller's +-7.5 m/s^2 so the guarantee survives the attitude transient
    # and the 30 Hz discretisation.  See ``safety.CBFConfig``.
    cbf_a_max: float = 3.0           # [m/s^2]
    cbf_brake_gain: float = 1.05     # [-] divisor on admissible speed (>=1 safer)

    # ---- reward weights -------------------------------------------------
    # The weights were tuned against measured baselines.  An earlier setting
    # (w_alive = +0.02, w_gate = 6, w_success = 18) made idling optimal: a
    # do-nothing hover scored +10.0 while a policy that threaded all three
    # gates scored -4.8, and PPO converged to hovering.  The survival term is
    # now a small time penalty and the completion bonuses dominate the
    # per-step running costs over a full episode.
    w_progress: float = 1.0          # potential-based shaping gain
    w_energy: float = 0.03           # rotor power penalty
    w_action: float = 0.02           # command smoothness
    w_alive: float = -0.02           # per-step time penalty (not a bonus)
    w_gate: float = 10.0             # gate bonus (exceeds the ~3.6 m potential jump)
    w_success: float = 40.0          # terminal success bonus
    w_crash: float = -25.0           # terminal crash penalty
    w_proximity: float = 0.15        # soft proximity penalty
    gamma: float = 0.99              # discount used by the shaping term

    # ---- cost / constraint ---------------------------------------------
    # The episodic cost is bounded in [0, 1 + cost_barrier_weight]:
    #     cost = 1{collision} + barrier_weight * (violating steps / max_steps)
    # Expressing the barrier term as a rate keeps the Lagrange multiplier in a
    # usable range; a summed per-step indicator saturated the dual variable.
    cost_limit: float = 0.6          # allowed mean episodic cost
    cost_barrier_weight: float = 0.5  # weight on the margin-encroachment rate

    # ---- observation ----------------------------------------------------
    n_rays: int = 24
    lidar_max: float = 3.0
    n_nearest_obs: int = 4

    # ---- misc -----------------------------------------------------------
    gui: bool = False
    seed: int = 0

    @property
    def dt(self) -> float:
        return 1.0 / self.ctrl_freq

    @property
    def substeps(self) -> int:
        return max(1, int(round(self.pyb_freq / self.ctrl_freq)))

    @property
    def gate_positions(self) -> np.ndarray:
        n = self.n_gates
        xs = self.first_gate_x + self.gate_spacing * np.arange(n)
        ys = np.array([self.gate_y_offsets[i % len(self.gate_y_offsets)] for i in range(n)])
        zs = self.gate_altitude + np.array(
            [self.gate_z_offsets[i % len(self.gate_z_offsets)] for i in range(n)]
        )
        return np.stack([xs, ys, zs], axis=1)


def _ray_directions(n_rays: int) -> np.ndarray:
    """Deterministic quasi-uniform unit directions (3 elevation rings)."""
    dirs = []
    rings = 3
    per_ring = max(1, n_rays // rings)
    elevations = np.linspace(-0.45, 0.45, rings)
    for k, elev in enumerate(elevations):
        az = np.linspace(0.0, 2 * np.pi, per_ring, endpoint=False) + k * (np.pi / per_ring)
        for a in az:
            dirs.append(
                [math.cos(elev) * math.cos(a), math.cos(elev) * math.sin(a), math.sin(elev)]
            )
    dirs = np.array(dirs[:n_rays])
    if len(dirs) < n_rays:  # pad by repeating
        pad = np.repeat(dirs[-1:], n_rays - len(dirs), axis=0)
        dirs = np.vstack([dirs, pad])
    return dirs
