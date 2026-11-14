"""
SafeDRL-DroneNav-ROS : constrained quadrotor navigation environment.

Task
----
A CrazyFlie-class quadrotor must traverse a *multi-gate corridor* along +x
while avoiding

  * static spherical obstacles,
  * **dynamic** obstacles that sweep across the corridor,
  * discrete **wind gusts** acting on the airframe,
  * the ground and the arena boundary.

The episode terminates successfully when every gate has been threaded in order.

What is new with respect to the reference project
(``eRGiBi/DRL-DroneNavigation``)
--------------------------------------------------------------------
1. **Safety constraint.**  Every environment returns an explicit *cost*
   signal (barrier / collision violation) in ``info["cost"]``, so the same
   environment can train a constrained (Lagrangian PPO) policy as well as an
   unconstrained one.  A control-barrier-function filter
   (:mod:`safety`) can additionally be layered on top of the action.
2. **Dynamic obstacles + wind gusts**, both fully parameterised and
   reproducible from a seed.
3. **Reward redesign.**  Guidance uses *potential-based reward shaping*
   (Ng, Harada & Russell, 1999) with the gate distance potential, which
   provably leaves the optimal policy unchanged, plus an **energy
   consumption** penalty derived from the rotor aerodynamic power ``sum(rpm^3)``
   and a command-smoothness term.  The safety margin enters as a *cost*, not
   as a hand-tuned reward penalty, which is the clean constrained-RL split.

Action interface
----------------
``action in [-1, 1]^4  ->  (vx, vy, vz) in m/s  +  yaw-rate in rad/s``
The desired velocity is the interface the CBF safety filter guards, see
:mod:`safety`.

Observation (53-dim)
--------------------
``[ pos_rel_gate(3), vel(3), rpy(3), 4 x nearest_obstacle(pos_rel(3), radius(1)),
    virtual_lidar(24), prev_action(4) ]``
"""

from __future__ import annotations

import math
from dataclasses import asdict

import numpy as np

try:  # gymnasium is the modern API used by SB3 >= 2.0
    import gymnasium as gym
    from gymnasium import spaces
    _GYM_LEGACY = False
except ImportError:  # pragma: no cover - fallback for gym-only installs
    import gym
    from gym import spaces
    _GYM_LEGACY = True

import pybullet as p
import pybullet_data  # noqa: F401  (ensures the data path is importable)

from drone_dynamics import CF2X, GeometricController, QuadrotorBody
from nav_config import EnvConfig, _ray_directions
from safety import CBFConfig, CBFFilter


# ==========================================================================
# Environment
# ==========================================================================


class SafeNavEnv(gym.Env):
    """PyBullet based safe-navigation environment (Gym / Gymnasium API)."""

    metadata = {"render_modes": ["human", "direct"], "render_fps": 30}

    def __init__(self, cfg: EnvConfig | None = None):
        super().__init__()
        self.cfg = cfg or EnvConfig()
        c = self.cfg

        self.rng = np.random.default_rng(c.seed)
        self._client = -1
        self._physics_ready = False

        # spaces ---------------------------------------------------------
        obs_dim = 3 + 3 + 3 + c.n_nearest_obs * 4 + c.n_rays + 4
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(obs_dim,), dtype=np.float32)
        self.action_space = spaces.Box(-1.0, 1.0, shape=(4,), dtype=np.float32)

        # runtime state ---------------------------------------------------
        self.gates = c.gate_positions
        self.obstacles: list[dict] = []
        self._frame_bodies: list[int] = []
        self._static_bodies: list[int] = []
        self._dynamic_bodies: list[int] = []
        self.body: QuadrotorBody | None = None
        self.controller = GeometricController(max_yaw_rate=c.max_yaw_rate)
        self.cbf = CBFFilter(
            CBFConfig(
                d_safe=c.cbf_d_safe,
                gamma=c.cbf_gamma,
                v_max=c.max_velocity,
                a_max=c.cbf_a_max,
                brake_gain=c.cbf_brake_gain,
            ),
            dt=c.dt,
        )
        self.wind = np.zeros(3)
        self._gust_steps = 0
        self._gust_vec = np.zeros(3)
        self._step_count = 0
        self._prev_action = np.zeros(4)
        self._prev_dist = 0.0
        self._gate_idx = 0
        self._crashed = False
        self._success = False
        self._min_margin = np.inf
        self._min_clearance = np.inf
        self._episode_cost = 0.0
        self._episode_energy = 0.0
        self._time = 0.0

        # trace buffers (filled when ``trace`` is enabled) ------------------
        self.trace: list[dict] = []
        self.trace_enabled = False

    # ------------------------------------------------------------------
    # Gym API
    # ------------------------------------------------------------------
    def reset(self, *, seed=None, options=None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        c = self.cfg

        self._connect()
        self._build_world()

        # initial pose ---------------------------------------------------
        start = np.array([0.0, 0.0, c.gate_altitude])
        start[:2] += self.rng.uniform(-c.spawn_noise, c.spawn_noise, size=2)
        self.body.reset(start, quat=(0.0, 0.0, 0.0, 1.0))
        self.controller.reset(yaw=0.0)
        self.cbf.reset_stats()

        # episode bookkeeping --------------------------------------------
        self._step_count = 0
        self._prev_action = np.zeros(4)
        self._gate_idx = 0
        self._crashed = False
        self._success = False
        self._min_margin = np.inf
        self._min_clearance = np.inf
        self._episode_cost = 0.0
        self._episode_energy = 0.0
        self._time = 0.0
        self.wind = np.asarray(c.wind_mean, dtype=np.float64).copy()
        self._gust_steps = 0
        self._gust_vec = np.zeros(3)
        self._prev_dist = float(np.linalg.norm(start - self.gates[0]))
        self.trace = []

        obs = self._compute_obs()
        info = self._base_info()
        return (obs, info) if not _GYM_LEGACY else obs

    def step(self, action):
        c = self.cfg
        action = np.clip(np.asarray(action, dtype=np.float64).reshape(-1), -1.0, 1.0)

        # ---- 1. safety filter on the velocity command -------------------
        # The measured velocity is passed in so the filter can compensate for
        # the plant's first-order velocity lag (see CBFFilter._command_bound).
        v_des = action[:3] * c.max_velocity
        h_vals, grad_h = self._barrier_terms()
        _, _, _, v_measured, _ = self.body.state()
        if c.use_cbf:
            v_safe = self.cbf.filter(v_des, h_vals, grad_h, v_actual=v_measured)
        else:
            v_safe = v_des
            # keep the filter statistics meaningful even when it is bypassed
            self.cbf.filter(v_des, h_vals, grad_h, v_actual=v_measured)

        # ---- 2. low-level control + physics -----------------------------
        rpm_cmd = self.controller.compute(
            self.body, v_safe, action[3] * c.max_yaw_rate, c.dt
        )
        for _ in range(c.substeps):
            self._advance_wind()
            self.body.apply_rpm(rpm_cmd, c.dt / c.substeps, wind_world=self.wind)
            p.stepSimulation(physicsClientId=self._client)
        self._time += c.dt
        self._move_dynamic_obstacles()

        # ---- 3. measurements --------------------------------------------
        pos, _, euler, vel, _ = self.body.state()
        # The reported safety margin is the *true* minimum of every barrier
        # (obstacles, ground, ceiling, walls), so the metrics cannot silently
        # ignore a constraint that the filter is enforcing.
        h_now, _ = self._barrier_terms()
        margin = float(np.min(h_now)) if h_now.size else float(c.lidar_max)
        dist_obs, idx = self._nearest_obstacle_distance(pos)
        self._min_margin = min(self._min_margin, margin)
        self._min_clearance = min(self._min_clearance, margin + c.cbf_d_safe)

        self._update_gate_progress(pos)
        self._crashed = self._check_crash(pos)
        self._success = self._gate_idx >= c.n_gates

        dist_gate = float(np.linalg.norm(pos - self.gates[min(self._gate_idx, c.n_gates - 1)]))

        # ---- 4. reward ---------------------------------------------------
        reward, terms = self._compute_reward(action, pos, dist_gate, margin, vel)
        terminated = bool(self._crashed or self._success)
        truncated = bool(self._step_count + 1 >= c.max_steps and not terminated)

        # ---- 5. constraint cost -----------------------------------------
        # Rate-normalised barrier term (see EnvConfig.cost_limit): the depth of
        # the encroachment is scaled by 1/max_steps, so a full episode spent
        # hard against the boundary contributes exactly `cost_barrier_weight`
        # rather than growing without bound with the horizon.
        barrier_violation = float(np.clip(-margin / max(c.cbf_d_safe, 1e-6), 0.0, 1.0))
        collision = 1.0 if (self._crashed and not self._success) else 0.0
        cost = collision + c.cost_barrier_weight * barrier_violation / c.max_steps
        self._episode_cost += cost

        # ---- 6. bookkeeping ---------------------------------------------
        self._step_count += 1
        self._prev_action = action
        self._prev_dist = dist_gate
        self._episode_energy += terms["energy"]

        if self.trace_enabled:
            self.trace.append(
                {
                    "t": self._time,
                    "pos": pos.copy(),
                    "vel": np.asarray(vel, dtype=np.float64).copy(),
                    "rpy": np.asarray(euler, dtype=np.float64).copy(),
                    "v_des": np.asarray(v_des, dtype=np.float64).copy(),
                    "v_safe": np.asarray(v_safe, dtype=np.float64).copy(),
                    "action": action.copy(),
                    "rpm": self.body.rpm.copy(),
                    "margin": float(margin),
                    "contact_clearance": float(margin + c.cbf_d_safe),
                    "dist_gate": dist_gate,
                    "wind": self.wind.copy(),
                    "reward": float(reward),
                    "cost": float(cost),
                    "gate_idx": self._gate_idx,
                }
            )

        obs = self._compute_obs()
        info = self._base_info()
        info.update(
            {
                "cost": float(cost),
                "is_success": bool(self._success),
                "is_collision": bool(self._crashed and not self._success),
                "gate_index": int(self._gate_idx),
                "min_margin": float(self._min_margin),
                "margin": float(margin),
                # distance to *actual* contact (margin + d_safe); > 0 means the
                # barrier held and no obstacle was ever touched
                "contact_clearance": float(margin + c.cbf_d_safe),
                # running episode minimum of the above - this is the number the
                # safety claim is stated in, so it is exposed at every step
                # rather than only in episode_extra
                "min_contact_clearance": float(self._min_clearance),
                "energy": float(terms["energy"]),
                "episode_cost": float(self._episode_cost),
                "episode_energy": float(self._episode_energy),
                "time": float(self._time),
                "dist_gate": dist_gate,
                "reward_terms": {k: float(v) for k, v in terms.items()},
                **self.cbf.stats.as_dict(),
            }
        )
        if terminated or truncated:
            info["episode_extra"] = {
                "success": float(self._success),
                "collision": float(self._crashed and not self._success),
                "min_margin": float(self._min_margin),
                "min_contact_clearance": float(self._min_clearance),
                "episode_cost": float(self._episode_cost),
                "episode_energy": float(self._episode_energy),
                "arrival_time": float(self._time) if self._success else float("nan"),
                "gates_passed": int(self._gate_idx),
            }
        if not _GYM_LEGACY:
            return obs, float(reward), terminated, truncated, info
        return obs, float(reward), terminated, info

    def close(self):
        if self._client >= 0:
            try:
                p.disconnect(physicsClientId=self._client)
            except Exception:
                pass
            self._client = -1

    def render(self):  # pragma: no cover - GUI only
        return None

    # ------------------------------------------------------------------
    # world construction
    # ------------------------------------------------------------------
    def _connect(self):
        if self._client < 0:
            mode = p.GUI if self.cfg.gui else p.DIRECT
            self._client = p.connect(mode)
            if self.cfg.gui:
                p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=self._client)
                p.resetDebugVisualizerCamera(
                    cameraDistance=6.0, cameraYaw=35.0, cameraPitch=-22.0,
                    cameraTargetPosition=[5.0, 0.0, 1.2], physicsClientId=self._client,
                )
        if not self._physics_ready:
            p.setTimeStep(1.0 / self.cfg.pyb_freq, physicsClientId=self._client)
            p.setGravity(0.0, 0.0, -CF2X.gravity, physicsClientId=self._client)
            p.setPhysicsEngineParameter(
                numSolverIterations=60, physicsClientId=self._client
            )
            self._physics_ready = True

    def _build_world(self):
        """(Re)create ground, gates, obstacles and the drone."""
        c = self.cfg
        cl = self._client

        if self.body is not None:  # reset(): wipe everything but the ground
            for bid in self._frame_bodies + self._static_bodies + self._dynamic_bodies:
                p.removeBody(bid, physicsClientId=cl)
            p.removeBody(self.body.body_id, physicsClientId=cl)
        self._frame_bodies, self._static_bodies, self._dynamic_bodies = [], [], []

        # ---- ground (created once per simulator connection) ----------------
        if getattr(self, "_ground_body", None) is None:
            self._ground = p.createCollisionShape(
                p.GEOM_BOX, halfExtents=[c.arena_x, c.arena_y + 2.0, 0.1], physicsClientId=cl
            )
            ground_vis = p.createVisualShape(
                p.GEOM_BOX, halfExtents=[c.arena_x, c.arena_y + 2.0, 0.1],
                rgbaColor=[0.91, 0.93, 0.90, 1.0], physicsClientId=cl,
            )
            self._ground_body = p.createMultiBody(
                0, self._ground, ground_vis, basePosition=[c.arena_x * 0.5, 0.0, -0.1],
                physicsClientId=cl,
            )

        # ---- gates -------------------------------------------------------
        ap, bar = c.gate_aperture, c.gate_bar
        for g in self.gates:
            frame_colour = [0.10, 0.68, 0.42, 1.0]
            v_half = ap + bar                     # half length of the vertical bars
            for sy in (-1.0, 1.0):
                col = p.createCollisionShape(
                    p.GEOM_BOX, halfExtents=[bar, bar, v_half], physicsClientId=cl
                )
                vis = p.createVisualShape(
                    p.GEOM_BOX, halfExtents=[bar, bar, v_half],
                    rgbaColor=frame_colour, physicsClientId=cl,
                )
                bid = p.createMultiBody(
                    0, col, vis, basePosition=[g[0], sy * (ap + bar), g[2]], physicsClientId=cl
                )
                self._frame_bodies.append(bid)
            for sz in (-1.0, 1.0):
                col = p.createCollisionShape(
                    p.GEOM_BOX, halfExtents=[bar, v_half, bar], physicsClientId=cl
                )
                vis = p.createVisualShape(
                    p.GEOM_BOX, halfExtents=[bar, v_half, bar],
                    rgbaColor=frame_colour, physicsClientId=cl,
                )
                bid = p.createMultiBody(
                    0, col, vis, basePosition=[g[0], 0.0, g[2] + sz * (ap + bar)], physicsClientId=cl
                )
                self._frame_bodies.append(bid)

        # ---- obstacles ---------------------------------------------------
        self.obstacles = []
        for i in range(c.n_static):
            self._spawn_static_obstacle(i)
        for i in range(c.n_dynamic):
            self._spawn_dynamic_obstacle(i)

        # ---- drone -------------------------------------------------------
        self.body = QuadrotorBody(cl, start_pos=[0.0, 0.0, c.gate_altitude])

        # keep the drone out of contact with anything at spawn
        p.performCollisionDetection(physicsClientId=cl)

    def _far_from_gates(self, xy_pos: np.ndarray, clearance: float = 0.75) -> bool:
        for g in self.gates:
            if np.linalg.norm(np.array([g[0], g[1]]) - xy_pos) < clearance:
                return False
        return True

    def _spawn_static_obstacle(self, idx: int):
        c = self.cfg
        cl = self._client
        lo, hi = c.static_radius
        for _ in range(200):
            r = float(self.rng.uniform(lo, hi))
            x = float(self.rng.uniform(1.6, c.arena_x - 0.8))
            y = float(self.rng.uniform(-c.arena_y * 0.72, c.arena_y * 0.72))
            z = float(self.rng.uniform(c.z_min + r + 0.15, c.z_max - r - 0.15))
            centre = np.array([x, y, z])
            if np.linalg.norm(centre) < c.min_spawn_clearance + r:
                continue
            if not self._far_from_gates(centre[:2]):
                continue
            break
        else:  # pragmatic fallback
            r, centre = 0.3, np.array([c.first_gate_x * 0.5, 0.9, 1.0])

        col = p.createCollisionShape(p.GEOM_SPHERE, radius=r, physicsClientId=cl)
        vis = p.createVisualShape(
            p.GEOM_SPHERE, radius=r, rgbaColor=[0.85, 0.33, 0.28, 0.95], physicsClientId=cl
        )
        bid = p.createMultiBody(0, col, vis, basePosition=list(centre), physicsClientId=cl)
        self._static_bodies.append(bid)
        self.obstacles.append(
            {"kind": "static", "id": bid, "centre": centre, "radius": r,
             "body": bid, "centre0": centre.copy()}
        )

    def _spawn_dynamic_obstacle(self, idx: int):
        c = self.cfg
        cl = self._client
        lo, hi = c.dynamic_radius
        slo, shi = c.dynamic_speed
        for _ in range(200):
            r = float(self.rng.uniform(lo, hi))
            x = float(self.rng.uniform(c.first_gate_x * 0.55, c.arena_x - 0.8))
            y0 = float(self.rng.uniform(-c.arena_y * 0.55, c.arena_y * 0.55))
            z0 = float(self.rng.uniform(c.z_min + 0.55, c.z_max - 0.55))
            if not self._far_from_gates(np.array([x, y0]), clearance=0.55):
                continue
            break
        else:
            r, x, y0, z0 = 0.22, c.first_gate_x + 1.0, 0.0, 1.5

        col = p.createCollisionShape(p.GEOM_SPHERE, radius=r, physicsClientId=cl)
        vis = p.createVisualShape(
            p.GEOM_SPHERE, radius=r, rgbaColor=[0.95, 0.66, 0.16, 0.95], physicsClientId=cl
        )
        centre = np.array([x, y0, z0])
        bid = p.createMultiBody(0, col, vis, basePosition=list(centre), physicsClientId=cl)
        self._dynamic_bodies.append(bid)

        speed = float(self.rng.uniform(slo, shi))
        # sweep direction: mostly across the corridor (y), sometimes vertical (z)
        if self.rng.random() < 0.7:
            axis = np.array([0.0, 1.0, 0.0])
            amp = float(self.rng.uniform(0.8, c.arena_y * 0.75))
        else:
            axis = np.array([0.0, 0.0, 1.0])
            amp = float(self.rng.uniform(0.5, 0.9))
        omega = speed / max(amp, 1e-6)
        self.obstacles.append(
            {
                "kind": "dynamic", "id": bid, "body": bid,
                "centre": centre, "centre0": centre.copy(), "radius": r,
                "axis": axis, "amp": amp, "omega": float(omega),
                "phase": float(self.rng.uniform(0.0, 2 * np.pi)),
            }
        )

    def _move_dynamic_obstacles(self):
        cl = self._client
        for ob in self.obstacles:
            if ob["kind"] != "dynamic":
                continue
            offset = ob["axis"] * ob["amp"] * math.sin(ob["omega"] * self._time + ob["phase"])
            centre = ob["centre0"] + offset
            ob["centre"] = centre
            p.resetBasePositionAndOrientation(
                ob["body"], list(centre), [0, 0, 0, 1], physicsClientId=cl
            )

    # ------------------------------------------------------------------
    # wind
    # ------------------------------------------------------------------
    def _advance_wind(self):
        c = self.cfg
        dt = c.dt / c.substeps
        mean = np.asarray(c.wind_mean, dtype=np.float64)
        theta, sigma = c.wind_ou_theta, c.wind_ou_sigma
        noise = self.rng.normal(0.0, 1.0, size=3)
        self.wind += theta * (mean - self.wind) * dt + sigma * math.sqrt(dt) * noise

        # discrete gust bursts
        if self._gust_steps > 0:
            self._gust_steps -= 1
            self.wind += self._gust_vec * dt
        elif self.rng.random() < c.gust_prob * dt * c.ctrl_freq:
            self._gust_steps = int(self.rng.integers(c.gust_duration[0], c.gust_duration[1] + 1))
            mag = float(self.rng.uniform(*c.gust_magnitude))
            direction = self.rng.normal(0.0, 1.0, size=3)
            direction[2] *= 0.35
            n = np.linalg.norm(direction)
            self._gust_vec = mag * direction / max(n, 1e-9)
        self.wind = np.clip(self.wind, -4.0, 4.0)

    # ------------------------------------------------------------------
    # observation
    # ------------------------------------------------------------------
    def _barrier_terms(self):
        """Return ``(h_k, grad_h_k)`` for every active barrier - the CBF input.

        The safe set is the complement of (a) the inflated obstacle spheres,
        (b) the ground half-space and (c) the ceiling / arena-lateral
        half-spaces.  Including the floor matters in practice: an untrained
        policy's most common failure is a dive into the ground, and that is a
        hard constraint the barrier should enforce just like a collision.

        The gate frames are *not* barriers - the task requires
        flying through the aperture, so a repulsive term there would fight the
        objective.  Aperture (0.58 m) versus airframe radius (0.10 m) leaves a
        5.8x clearance, and the reward's collision cost covers the residual.
        """
        c = self.cfg
        pos, _, _, _, _ = self.body.state()

        # ``h`` is the free distance beyond the safety boundary.  The
        # speed-dependent braking requirement is *not* baked in here - it is
        # applied inside CBFFilter._approach_bound, which keeps the geometry
        # and the speed constraint cleanly separated.
        hs, grads = [], []
        for ob in self.obstacles:
            d = pos - ob["centre"]
            n = float(np.linalg.norm(d))
            hs.append(n - ob["radius"] - c.drone_radius - c.cbf_d_safe)
            grads.append(d / max(n, 1e-9))

        # ground plane:  h = z - z_min      grad = +z
        hs.append(pos[2] - c.z_min)
        grads.append(np.array([0.0, 0.0, 1.0]))
        # ceiling:       h = z_max - z      grad = -z
        hs.append(c.z_max - pos[2])
        grads.append(np.array([0.0, 0.0, -1.0]))
        # lateral walls of the corridor
        hs.append(c.arena_y - pos[1])
        grads.append(np.array([0.0, -1.0, 0.0]))
        hs.append(c.arena_y + pos[1])
        grads.append(np.array([0.0, 1.0, 0.0]))

        return np.asarray(hs), np.asarray(grads)

    def _nearest_obstacle_distance(self, pos: np.ndarray):
        if not self.obstacles:
            return float(self.cfg.lidar_max), -1
        best, best_i = np.inf, -1
        for i, ob in enumerate(self.obstacles):
            d = float(np.linalg.norm(pos - ob["centre"])) - ob["radius"] - self.cfg.drone_radius
            if d < best:
                best, best_i = d, i
        return best, best_i

    def _lidar(self, pos: np.ndarray) -> np.ndarray:
        """Analytic virtual 3-D lidar: normalised range to the nearest obstacle."""
        c = self.cfg
        out = np.full(c.n_rays, 1.0)
        dirs = _ray_directions(c.n_rays)
        for i, d in enumerate(dirs):
            best_t = c.lidar_max
            for ob in self.obstacles:
                oc = pos - ob["centre"]
                b = float(d @ oc)
                cc = float(oc @ oc) - ob["radius"] ** 2
                disc = b * b - cc
                if disc < 0.0:
                    continue
                t = -b - math.sqrt(disc)
                if 1e-6 < t < best_t:
                    best_t = t
            out[i] = min(best_t, c.lidar_max) / c.lidar_max
        return out

    def _compute_obs(self) -> np.ndarray:
        c = self.cfg
        pos, _, euler, vel, _ = self.body.state()
        gate = self.gates[min(self._gate_idx, c.n_gates - 1)]

        pos_rel = (pos - gate) / np.array([c.arena_x, c.arena_y, c.z_max])
        vel_n = np.asarray(vel, dtype=np.float64) / c.max_velocity
        rpy_n = np.asarray(euler, dtype=np.float64) / math.pi

        # nearest K obstacles
        order = np.argsort(
            [np.linalg.norm(pos - ob["centre"]) - ob["radius"] for ob in self.obstacles]
        )[: c.n_nearest_obs]
        obs_feat = np.zeros((c.n_nearest_obs, 4))
        for j, oi in enumerate(order):
            ob = self.obstacles[oi]
            obs_feat[j, :3] = (ob["centre"] - pos) / c.lidar_max
            obs_feat[j, 3] = ob["radius"] / 0.5

        lidar = self._lidar(pos)
        obs = np.concatenate(
            [pos_rel, vel_n, rpy_n, obs_feat.ravel(), lidar, self._prev_action]
        )
        return np.nan_to_num(obs, nan=0.0, posinf=1.0, neginf=-1.0).astype(np.float32)

    # ------------------------------------------------------------------
    # reward / termination
    # ------------------------------------------------------------------
    def _compute_reward(self, action, pos, dist_gate, margin, vel):
        c = self.cfg
        rpm = self.body.rpm / CF2X.max_rpm
        energy = float(np.sum(rpm ** 3))          # aerodynamic power ~ rpm^3
        smooth = float(np.linalg.norm(action - self._prev_action) ** 2)

        # Ng et al. potential-based shaping:  F = gamma*Phi(s') - Phi(s),
        # with Phi(s) = -dist_to_active_gate.  Policy-invariant by construction.
        shaping = c.gamma * (-dist_gate) - (-self._prev_dist)

        proximity = float(max(0.0, 1.0 - margin / c.cbf_d_safe)) if margin < c.cbf_d_safe else 0.0

        terms = {
            "progress": c.w_progress * shaping,
            "energy": c.w_energy * energy,
            "action": c.w_action * smooth,
            "proximity": -c.w_proximity * proximity,
            "alive": c.w_alive,
            "gate": 0.0,
            "terminal": 0.0,
        }

        reward = (
            terms["progress"]
            - terms["energy"]
            - terms["action"]
            + terms["proximity"]
            + terms["alive"]
        )

        if self._gate_passed_this_step:
            terms["gate"] = c.w_gate
            reward += c.w_gate
        if self._success:
            terms["terminal"] = c.w_success
            reward += c.w_success
        elif self._crashed:
            terms["terminal"] = c.w_crash
            reward += c.w_crash

        return float(reward), terms

    def _update_gate_progress(self, pos: np.ndarray):
        self._gate_passed_this_step = False
        if self._gate_idx >= self.cfg.n_gates:
            return
        g = self.gates[self._gate_idx]
        radial = math.hypot(pos[1] - g[1], pos[2] - g[2])
        if pos[0] > g[0] and radial < self.cfg.gate_aperture:
            self._gate_idx += 1
            self._gate_passed_this_step = True

    def _check_crash(self, pos: np.ndarray) -> bool:
        c = self.cfg
        if pos[2] < c.z_min or pos[2] > c.z_max:
            return True
        if abs(pos[1]) > c.arena_y or pos[0] > c.arena_x or pos[0] < -1.5:
            return True
        contacts = p.getContactPoints(bodyA=self.body.body_id, physicsClientId=self._client)
        return len(contacts) > 0

    # ------------------------------------------------------------------
    def _base_info(self) -> dict:
        return {
            "step": self._step_count,
            "gate_index": self._gate_idx,
            "n_gates": self.cfg.n_gates,
            "wind_norm": float(np.linalg.norm(self.wind)),
            "cbf_enabled": float(self.cfg.use_cbf),
        }

    # ------------------------------------------------------------------
    # helpers used by the ROS bridge / plotting
    # ------------------------------------------------------------------
    def obstacle_states(self):
        return [
            {"kind": ob["kind"], "centre": ob["centre"].copy(), "radius": ob["radius"]}
            for ob in self.obstacles
        ]

    def drone_state(self) -> dict:
        pos, quat, euler, vel, omega = self.body.state()
        return {
            "position": pos, "quaternion": quat, "euler": euler,
            "velocity": vel, "angular_velocity": omega,
            "rpm": self.body.rpm.copy(), "time": self._time,
        }

    def config_dict(self) -> dict:
        return asdict(self.cfg)
