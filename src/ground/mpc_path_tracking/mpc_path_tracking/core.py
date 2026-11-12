"""Unicycle model, reference path and a box-constrained QP solver for MPC.

The controller is a linear time-varying MPC: the unicycle is linearised around
the reference at every step, the horizon is condensed into one quadratic
program in the control increments, and the box constraints are handled by a
projected-gradient loop (the problem is small enough that this beats pulling in
a QP dependency).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

EPS = 1e-9


def wrap(angle):
    return (np.asarray(angle, dtype=float) + np.pi) % (2.0 * np.pi) - np.pi


@dataclass(frozen=True)
class Unicycle:
    """Differential-drive kinematics with box limits on the commands."""

    max_speed: float = 1.0
    max_yaw_rate: float = 1.2
    max_accel: float = 1.5
    max_yaw_accel: float = 3.0

    def step(self, state, control, dt: float):
        x, y, theta = state
        speed, yaw_rate = control
        theta_mid = theta + 0.5 * yaw_rate * dt
        return np.array(
            [
                x + speed * dt * np.cos(theta_mid),
                y + speed * dt * np.sin(theta_mid),
                float(wrap(theta + yaw_rate * dt)),
            ]
        )


def linearize(state, control, dt: float):
    """Discrete-time Jacobians of :meth:`Unicycle.step`."""
    _, _, theta = state
    speed, yaw_rate = control
    theta_mid = theta + 0.5 * yaw_rate * dt
    jacobian_state = np.array(
        [
            [1.0, 0.0, -speed * dt * np.sin(theta_mid)],
            [0.0, 1.0, speed * dt * np.cos(theta_mid)],
            [0.0, 0.0, 1.0],
        ]
    )
    jacobian_control = np.array(
        [
            [dt * np.cos(theta_mid), -0.5 * speed * dt * dt * np.sin(theta_mid)],
            [dt * np.sin(theta_mid), 0.5 * speed * dt * dt * np.cos(theta_mid)],
            [0.0, dt],
        ]
    )
    return jacobian_state, jacobian_control


def solve_box_qp(hessian, gradient, lower, upper, iterations: int = 80):
    """Box-constrained QP by cyclic coordinate descent.

    Each coordinate is replaced by the clipped one-dimensional minimiser
    `u_i <- clip(u_i - (H u + g)_i / H_ii, lower_i, upper_i)`.  For a strictly
    convex QP this converges to the global optimum, and it needs no factorisation
    or active-set bookkeeping.
    """
    hessian = np.asarray(hessian, dtype=float)
    gradient = np.asarray(gradient, dtype=float)
    lower = np.asarray(lower, dtype=float)
    upper = np.asarray(upper, dtype=float)
    if hessian.ndim != 2 or hessian.shape[1] != gradient.size:
        raise ValueError("inconsistent QP dimensions")
    if np.any(lower > upper):
        raise ValueError("lower bound exceeds upper bound")
    diagonal = np.diag(hessian).copy()
    diagonal[diagonal < 1e-12] = 1e-12
    solution = np.clip(np.zeros_like(gradient), lower, upper)
    for _ in range(iterations):
        largest_change = 0.0
        for index in range(solution.size):
            residual = gradient[index] + hessian[index] @ solution
            updated = np.clip(solution[index] - residual / diagonal[index],
                              lower[index], upper[index])
            largest_change = max(largest_change, abs(updated - solution[index]))
            solution[index] = updated
        if largest_change < 1e-10:
            break
    return solution


@dataclass(frozen=True)
class MpcConfig:
    """Horizon, weights and limits of the tracker."""

    horizon: int = 25
    dt: float = 0.1
    weight_position: float = 30.0
    weight_heading: float = 2.0
    weight_control: float = 0.05
    weight_increment: float = 0.8
    max_speed: float = 1.0
    max_yaw_rate: float = 1.2
    max_accel: float = 1.5
    max_yaw_accel: float = 3.0

    def validate(self) -> None:
        if self.horizon < 2:
            raise ValueError("horizon must be at least two steps")
        if self.dt <= 0.0:
            raise ValueError("dt must be positive")
        if min(self.weight_position, self.weight_heading, self.weight_control,
               self.weight_increment) < 0.0:
            raise ValueError("weights must be non-negative")


def _condense(reference_states, reference_controls, state, last_control, config: MpcConfig):
    """Build the condensed QP in the control increments."""
    horizon = config.horizon
    state_dim, control_dim = 3, 2
    dynamics_a = []
    dynamics_b = []
    current = np.asarray(state, dtype=float).copy()
    nominal_controls = []
    for index in range(horizon):
        nominal = reference_controls[min(index, len(reference_controls) - 1)]
        jacobian_state, jacobian_control = linearize(current, nominal, config.dt)
        dynamics_a.append(jacobian_state)
        dynamics_b.append(jacobian_control)
        current = np.asarray(Unicycle().step(current, nominal, config.dt))
        nominal_controls.append(np.asarray(nominal, dtype=float))

    # State sensitivity to the control increments (u = u0 + sum(delta)).
    sensitivity = np.zeros((horizon * state_dim, horizon * control_dim))
    for row in range(horizon):
        block = np.zeros((state_dim, horizon * control_dim))
        for column in range(row + 1):
            factor = np.eye(state_dim)
            for inner in range(row, column, -1):
                factor = factor @ dynamics_a[inner]
            block[:, column * control_dim : (column + 1) * control_dim] = factor @ dynamics_b[column]
        sensitivity[row * state_dim : (row + 1) * state_dim, :] = block

    # Nominal rollout keeps the increments at zero.
    nominal_states = []
    current = np.asarray(state, dtype=float).copy()
    for index in range(horizon):
        current = np.asarray(Unicycle().step(current, nominal_controls[index], config.dt))
        nominal_states.append(current)
    deviation = np.concatenate(
        [reference_states[index] - nominal_states[index] for index in range(horizon)]
    )

    weight_state = np.kron(np.eye(horizon), np.diag(
        [config.weight_position, config.weight_position, config.weight_heading]
    ))
    weight_control = np.kron(np.eye(horizon), np.eye(control_dim) * config.weight_control)
    weight_increment = np.eye(horizon * control_dim) * config.weight_increment

    hessian = (
        sensitivity.T @ weight_state @ sensitivity
        + weight_control
        + weight_increment
    )
    hessian = hessian + 1e-6 * np.eye(hessian.shape[0])
    gradient = -sensitivity.T @ weight_state @ deviation
    return hessian, gradient, nominal_controls, sensitivity, nominal_states


def mpc_command(reference_states, reference_controls, state, last_control, config: MpcConfig):
    """Solve the QP and return ``(first_control, info)``."""
    config.validate()
    horizon = config.horizon
    hessian, gradient, nominal_controls, sensitivity, nominal_states = _condense(
        reference_states, reference_controls, state, last_control, config
    )
    lower = np.empty(horizon * 2)
    upper = np.empty(horizon * 2)
    for index in range(horizon):
        # Deviation from the feed-forward command; the absolute limits are
        # applied after the solve.  A narrow window here would leave the
        # controller without authority to reject a large initial offset.
        lower[index * 2] = -config.max_speed
        upper[index * 2] = config.max_speed
        lower[index * 2 + 1] = -config.max_yaw_rate
        upper[index * 2 + 1] = config.max_yaw_rate
    increments = solve_box_qp(hessian, gradient, lower, upper)
    # Absolute command limits are enforced by clipping after accumulation.
    commands = []
    for index in range(horizon):
        base = nominal_controls[index]
        increment = increments[index * 2 : (index + 1) * 2]
        commands.append(base + increment)
    first = np.asarray(commands[0], dtype=float)
    first[0] = np.clip(first[0], -config.max_speed, config.max_speed)
    first[1] = np.clip(first[1], -config.max_yaw_rate, config.max_yaw_rate)
    return first, {
        "increments": increments,
        "nominal": np.asarray(nominal_controls),
        "deviation": float(np.linalg.norm(increments)),
    }
