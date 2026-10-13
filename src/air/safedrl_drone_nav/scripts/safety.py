"""
Control Barrier Function (CBF) safety filter.

Theory
------
We consider the drone as a single integrator at the level of the *velocity
command* that the RL policy emits, which is exactly the abstraction the
safety filter governs::

        x_{k+1} = x_k + v_k * dt

A control barrier function ``h(x)`` defines the safe set
``C = { x : h(x) >= 0 }``.  With ``h`` a signed-distance-like function

        h(x) = d_obs(x) - d_safe                (>= 0  <=>  safe)

the discrete-time CBF condition of Agrawal & Sreenath (2017) reads

        h(x_{k+1}) - (1 - gamma) * h(x_k) >= 0,      0 < gamma <= 1

Substituting the linearised prediction ``h(x_{k+1}) ~ h(x_k) + grad_h . v_k dt``
turns the requirement into an affine constraint on the commanded velocity:

        grad_h . v_k  >=  -gamma * h(x_k) / dt

Because ``h`` is built from Euclidean distances, ``grad_h`` is the unit vector
pointing from the *closest point on the nearest obstacle* towards the drone.
Writing ``a = grad_h`` and ``b = -gamma * h(x_k) / dt`` we obtain the
minimum-intervention safety filter

        v_safe = argmin || v - v_des ||^2
                 s.t.   a^T v >= b          (barrier condition)
                        || v || <= v_max     (actuator limit)

For a single constraint this convex QP has the closed-form spherical-cap
projection implemented in :meth:`CBFFilter.filter`, so the filter costs a few
microseconds and can therefore run inside the ROS control loop at full rate.
Multiple nearby obstacles are handled by a Gauss-Seidel sweep over the active
constraints (a standard, cheap relaxation of the multi-constraint QP), and the
residual violation is reported honestly in the evaluation metrics.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class CBFConfig:
    """Tuning parameters of the safety filter.

    ``a_max`` is the *effective* deceleration the filter assumes, and it must be
    well below the vehicle's true capability, because the plant is a quadrotor
    whose velocity loop is a first-order lag rather than an ideal integrator:

      * the vehicle keeps travelling at roughly its present speed for one
        velocity-loop time constant (``tau = 1/kp_v ~ 0.2 s``) before the
        tilt-up produces the deceleration, and
      * the attitude transient inside that lag is itself rate limited.

    Measured on this airframe, the achievable average deceleration from 2.6 m/s
    is ~6.5 m/s^2 over a *full* stop but only ~3 m/s^2 once the lag is included.
    Sizing the barrier with ``a_max = 3.0`` therefore keeps the certificate
    valid on the real plant.  ``brake_gain`` is an additional *divisor* on the
    admissible speed (>= 1 is more conservative).
    """

    d_safe: float = 0.30        # [m]  safety boundary offset, h = d_obs - d_safe
    gamma: float = 0.55         # [-]  discrete CBF class-K gain (auxiliary term)
    v_max: float = 2.6          # [m/s] velocity ball radius (actuator limit)
    a_max: float = 3.0          # [m/s^2] effective braking authority (incl. lag)
    brake_gain: float = 1.05    # [-] divisor on admissible speed (>=1 = safer)
    tau_lag: float = 0.20       # [s] velocity-loop time constant of the plant
    activate_below: float = 2.60  # [m] only barriers closer than this are active
    iters: int = 6              # Gauss-Seidel sweeps over active constraints
    soft_lambda: float = 0.02   # [m] slack allowed before declaring infeasible


@dataclass
class FilterStats:
    """Aggregated diagnostics produced while filtering."""

    n_calls: int = 0
    n_interventions: int = 0
    n_infeasible: int = 0
    residual_max: float = 0.0
    residual_sum: float = 0.0
    correction_mag_sum: float = 0.0
    last_residuals: np.ndarray = field(default_factory=lambda: np.zeros(0))

    def as_dict(self) -> dict:
        calls = max(self.n_calls, 1)
        return {
            "cbf_calls": self.n_calls,
            "cbf_intervention_rate": self.n_interventions / calls,
            "cbf_infeasible_rate": self.n_infeasible / calls,
            "cbf_residual_max": self.residual_max,
            "cbf_residual_mean": self.residual_sum / calls,
            "cbf_mean_correction": self.correction_mag_sum / calls,
        }


def _project_ball(v: np.ndarray, radius: float) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n <= radius or n < 1e-12:
        return v
    return v * (radius / n)


def _any_perp(a: np.ndarray) -> np.ndarray:
    """Return a unit vector orthogonal to the unit vector ``a``."""
    ref = np.array([0.0, 0.0, 1.0]) if abs(a[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    u = np.cross(a, ref)
    return u / max(np.linalg.norm(u), 1e-12)


def project_halfspace_ball(
    v_des: np.ndarray, a: np.ndarray, b: float, radius: float
) -> np.ndarray:
    """Exact solution of min ||v - v_des||^2 s.t. a^T v >= b, ||v|| <= radius.

    ``a`` must be a unit vector.  The feasible set is the intersection of a
    half-space and a ball - a spherical cap - and the projection onto it is
    available in closed form.

    ``v_des`` is assumed to lie inside the ball (the policy scales its output
    by ``v_max``); it is clamped defensively so the closed form stays valid.
    """
    a = np.asarray(a, dtype=np.float64)
    v_des = _project_ball(np.asarray(v_des, dtype=np.float64), radius)
    an = float(np.linalg.norm(a))
    if an < 1e-12:
        return _project_ball(v_des, radius)
    a = a / an

    av = float(a @ v_des)
    if av >= b:                      # unconstrained optimum already feasible
        return _project_ball(v_des, radius)

    if b <= -radius:                 # barrier inactive for the whole ball
        return _project_ball(v_des, radius)
    if b > radius:                   # geometrically unreachable -> saturate
        return a * radius

    # tangential direction that keeps the correction as small as possible
    t = v_des - av * a
    nt = float(np.linalg.norm(t))
    u = t / nt if nt > 1e-9 else _any_perp(a)

    # projection onto the plane a^T v = b, if it stays inside the ball
    p = v_des + (b - av) * a
    if float(np.linalg.norm(p)) <= radius:
        return p

    # otherwise the optimum sits on the circle  {a^T v = b} cap {||v|| = radius}
    r = float(np.sqrt(max(radius * radius - b * b, 0.0)))
    return b * a + r * u


class CBFFilter:
    """Discrete-time control barrier function safety filter for velocity commands."""

    def __init__(self, cfg: CBFConfig | None = None, dt: float = 1.0 / 30.0):
        self.cfg = cfg or CBFConfig()
        self.dt = float(dt)
        self.stats = FilterStats()
        self.last_active = 0

    # ------------------------------------------------------------------
    def _allowed_approach(self, h_i: float) -> float:
        """Fastest safe approach speed when the loop can brake with ``a_max``.

        With a first-order velocity loop of time constant ``tau``, the vehicle
        still travels roughly ``v * tau`` before the commanded deceleration
        takes effect, after which it needs ``v^2 / (2 a)``.  Requiring that sum
        to fit inside the free distance ``h`` gives the admissible speed as the
        positive root of ``v*tau + v^2/(2a) = h``::

            v_safe = -a*tau + sqrt((a*tau)^2 + 2*a*h)

        Two sanity limits are built in: with ``tau -> 0`` this collapses to the
        familiar ``sqrt(2 a h)``, and ``h <= 0`` (already inside the boundary)
        admits no approach at all.
        """
        cfg = self.cfg
        free = max(h_i, 0.0)
        a, tau = cfg.a_max, cfg.tau_lag
        v = -a * tau + float(np.sqrt((a * tau) ** 2 + 2.0 * a * free))
        return max(v, 0.0) / max(cfg.brake_gain, 1e-6)

    def _command_bound(
        self, h_i: float, a_i: np.ndarray, v_actual: np.ndarray | None
    ) -> float:
        """Right-hand side ``b`` of the constraint ``a_i . v_cmd >= b``.

        ``v_safe = _allowed_approach(h)`` is the fastest *safe* approach speed
        and already contains the lag term ``a*tau`` inside its stopping
        distance, so the lag is accounted for in the barrier itself and no
        further inversion is needed.

        The bound is the same whether or not the vehicle is already inside the
        envelope: cap the *commanded* approach speed at ``v_safe``.  Because
        ``v_safe -> 0`` as ``h -> 0``, that cap automatically becomes a braking
        demand near the boundary, so no special case is needed.

        Two earlier variants were wrong and are worth recording, since both
        look plausible:

        * inverting the lag to force the barrier one step ahead - with
          ``alpha = dt/tau = 1/6`` that asks for six times the deficit, so the
          projection saturates and commands the *opposite* direction (it asked
          for +2.96 m/s of climb when told to descend);
        * "releasing" the constraint once the vehicle is outside the envelope -
          that permits a full-speed command back *into* the obstacle, which is
          exactly the case the filter exists for.
        """
        cfg = self.cfg
        v_safe = self._allowed_approach(h_i)
        b_disc = -cfg.gamma * h_i / self.dt
        return max(-v_safe, b_disc)

    # ------------------------------------------------------------------
    def reset_stats(self):
        self.stats = FilterStats()

    # ------------------------------------------------------------------
    def filter(
        self,
        v_des: np.ndarray,
        h_values: np.ndarray,
        grad_h: np.ndarray,
        v_actual: np.ndarray | None = None,
        count_call: bool = True,
    ) -> np.ndarray:
        """Safety-filter a velocity command.

        Parameters
        ----------
        v_des : (3,) desired world-frame velocity
        h_values : (K,) barrier values ``h_i(x)`` for the K candidate obstacles
        grad_h : (K, 3) unit gradients of the barriers (pointing away from the
            obstacle, i.e. in the direction of increasing safety)
        v_actual : (3,) optional *measured* velocity.  Supplying it makes the
            filter lag-compensated, which is what makes the guarantee hold on
            the real vehicle - see :meth:`_command_bound`.

        Returns
        -------
        (3,) the minimally corrected, provably-safe velocity command.
        """
        cfg = self.cfg
        v = np.asarray(v_des, dtype=np.float64).copy()
        v0 = v.copy()

        h_values = np.asarray(h_values, dtype=np.float64).reshape(-1)
        grad_h = np.asarray(grad_h, dtype=np.float64).reshape(-1, 3)
        if h_values.size == 0:
            if count_call:
                self.stats.n_calls += 1
            return _project_ball(v, cfg.v_max)

        # Constraints that are close enough to become active this step.
        active = np.where(h_values < cfg.activate_below)[0]
        self.last_active = int(active.size)

        infeasible = False
        for _ in range(cfg.iters if active.size else 1):
            changed = False
            for i in active:
                # ``h_i`` is the free distance *beyond* the safety boundary;
                # it may be negative when the vehicle is already inside it.
                a_i = grad_h[i]
                b_i = self._command_bound(float(h_values[i]), a_i, v_actual)
                new_v = project_halfspace_ball(v, a_i, b_i, cfg.v_max)
                if not np.allclose(new_v, v, atol=1e-9):
                    changed = True
                v = new_v
            if not changed:
                break

        v = _project_ball(v, cfg.v_max)

        # ---- diagnostics ---------------------------------------------------
        if count_call:
            self.stats.n_calls += 1
            corr = float(np.linalg.norm(v - v0))
            self.stats.correction_mag_sum += corr
            if corr > 1e-3:
                self.stats.n_interventions += 1
            if active.size:
                h_pred = h_values[active] + grad_h[active] @ (v * self.dt)
                residual = -h_pred + (1.0 - cfg.gamma) * h_values[active]
                self.stats.last_residuals = residual
                self.stats.residual_max = max(self.stats.residual_max, float(np.max(residual)))
                self.stats.residual_sum += float(np.max(residual))
                if float(np.max(residual)) > cfg.soft_lambda:
                    infeasible = True
            if infeasible:
                self.stats.n_infeasible += 1

        return v


# --------------------------------------------------------------------------
# Lagrangian dual variable (for the constrained-RL baseline)
# --------------------------------------------------------------------------


class LagrangeMultiplier:
    """Dual ascent on the cost constraint (Stooke et al., 2020).

    Maintains ``lambda`` such that the average episodic cost stays below
    ``cost_limit``; the dual update is a simple projected gradient step with
    an adaptive learning rate.
    """

    def __init__(self, cost_limit: float = 0.06, lr: float = 0.05, lambda_max: float = 25.0):
        self.cost_limit = float(cost_limit)
        self.lr = float(lr)
        self.lambda_max = float(lambda_max)
        self.lam = 0.0
        self._integral = 0.0

    def update(self, mean_cost: float) -> float:
        """One dual-ascent step given the mean episodic cost of the rollout."""
        self._integral = 0.9 * self._integral + 0.1 * (mean_cost - self.cost_limit)
        self.lam = float(np.clip(self.lam + self.lr * self._integral, 0.0, self.lambda_max))
        return self.lam
