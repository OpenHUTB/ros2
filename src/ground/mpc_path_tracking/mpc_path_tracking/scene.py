"""Reference path, closed-loop simulation and the pure-pursuit baseline."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .core import MpcConfig, Unicycle, mpc_command, wrap


@dataclass(frozen=True)
class TrackConfig:
    """Reference path and closed-loop settings."""

    duration_s: float = 40.0
    control_hz: float = 10.0
    speed: float = 0.9
    lateral_offset: float = 1.2
    heading_offset_deg: float = 20.0
    lookahead: float = 1.0

    def validate(self) -> None:
        if self.duration_s <= 0.0:
            raise ValueError("duration_s must be positive")
        if self.control_hz <= 0.0:
            raise ValueError("control_hz must be positive")


class ReferencePath:
    """Smooth S-curve reference sampled at uniform arc length."""

    def __init__(self, length: float = 40.0, amplitude: float = 3.0, period: float = 18.0):
        count = int(length / 0.02)
        self.x = np.linspace(-5.0, length - 5.0, count)
        self.y = amplitude * np.sin(2.0 * np.pi * self.x / period)
        self.theta = np.arctan2(np.gradient(self.y, self.x), 1.0)
        self.s = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(self.x), np.diff(self.y)))])
        self.spacing = 0.02
        self.length = float(self.s[-1])
        self.speed = 0.9
        self.curvature_s = np.gradient(self.theta, self.s)

    def sample(self, arc_length) -> tuple[np.ndarray, np.ndarray]:
        s = np.atleast_1d(np.asarray(arc_length, dtype=float)) % self.length
        x = np.interp(s, self.s, self.x)
        y = np.interp(s, self.s, self.y)
        theta = np.interp(s, self.s, self.theta)
        return np.stack([x, y, theta], axis=1), s

    def curvature(self, arc_length):
        return np.interp(np.asarray(arc_length, dtype=float) % self.length, self.s, self.curvature_s)

    def project(self, point, hint_s: float = 0.0, window: float = 3.0) -> float:
        distances = np.hypot(self.x - point[0], self.y - point[1])
        if window > 0.0:
            mask = np.abs(self.s - hint_s) < window
            if np.any(mask):
                return float(self.s[np.flatnonzero(mask)[np.argmin(distances[mask])]])
        return float(self.s[int(np.argmin(distances))])


def build_reference(path: ReferencePath, horizon: int, speed: float) -> tuple[np.ndarray, np.ndarray]:
    """Reference states and controls for one horizon starting at arc length 0."""
    dt = 0.0
    states = []
    controls = []
    for index in range(horizon):
        sample, _ = path.sample(index * 0.1 * speed)
        states.append(sample[0])
        controls.append([speed, 0.0])
        dt += 0.0
    return np.asarray(states), np.asarray(controls)


def run_mpc(path: ReferencePath, track: TrackConfig, mpc: MpcConfig) -> dict:
    """Closed loop with the MPC tracker."""
    track.validate()
    mpc.validate()
    dt = 1.0 / track.control_hz
    steps = int(round(track.duration_s * track.control_hz))
    start, _ = path.sample(0.0)
    state = np.array(
        [
            start[0][0],
            start[0][1] + track.lateral_offset,
            start[0][2] + np.deg2rad(track.heading_offset_deg),
        ]
    )
    model = Unicycle(max_speed=mpc.max_speed, max_yaw_rate=mpc.max_yaw_rate)
    states = []
    controls = []
    times = []
    hint = 0.0
    previous = np.zeros(2)
    for index in range(steps):
        hint = path.project(state[:2], hint)
        reference_states = []
        reference_controls = []
        for offset in range(1, mpc.horizon + 1):
            sample, _ = path.sample(hint + offset * track.speed * mpc.dt)
            reference_states.append(sample[0])
            arc = hint + offset * track.speed * mpc.dt
            reference_controls.append([track.speed, track.speed * float(path.curvature(arc))])
        command, _ = mpc_command(
            np.asarray(reference_states),
            np.asarray(reference_controls),
            state,
            previous,
            mpc,
        )
        previous = command
        states.append(state.copy())
        controls.append(command.copy())
        times.append(index * dt)
        state = model.step(state, command, dt)
    return {
        "time_s": np.asarray(times),
        "states": np.asarray(states),
        "controls": np.asarray(controls),
    }


def run_pure_pursuit(path: ReferencePath, track: TrackConfig) -> dict:
    """Closed loop with the pure-pursuit baseline."""
    track.validate()
    dt = 1.0 / track.control_hz
    steps = int(round(track.duration_s * track.control_hz))
    start, _ = path.sample(0.0)
    state = np.array(
        [
            start[0][0],
            start[0][1] + track.lateral_offset,
            start[0][2] + np.deg2rad(track.heading_offset_deg),
        ]
    )
    model = Unicycle()
    states = []
    controls = []
    times = []
    hint = 0.0
    for index in range(steps):
        hint = path.project(state[:2], hint)
        target, _ = path.sample(hint + track.lookahead)
        target = target[0]
        bearing = np.arctan2(target[1] - state[1], target[0] - state[0])
        alpha = float(wrap(bearing - state[2]))
        yaw_rate = float(np.clip(2.0 * track.speed * np.sin(alpha) / track.lookahead,
                                 -1.2, 1.2))
        command = np.array([track.speed, yaw_rate])
        states.append(state.copy())
        controls.append(command.copy())
        times.append(index * dt)
        state = model.step(state, command, dt)
    return {
        "time_s": np.asarray(times),
        "states": np.asarray(states),
        "controls": np.asarray(controls),
    }


def tracking_metrics(path: ReferencePath, run: dict) -> dict:
    """Lateral error and control smoothness of a closed-loop run."""
    errors = []
    for state in run["states"]:
        nearest = path.project(state[:2], 0.0, window=0.0)
        sample, _ = path.sample(nearest)
        errors.append(float(np.hypot(*(state[:2] - sample[0][:2]))))
    errors = np.asarray(errors)
    controls = run["controls"]
    speed = controls[:, 0]
    yaw_rate = controls[:, 1]
    time_s = run["time_s"]
    steady = errors[time_s > 5.0]
    within = np.flatnonzero(errors < 0.15)
    recovery = float(time_s[within[0]]) if within.size else float("nan")
    return {
        "lateral_rmse_m": float(np.sqrt(np.mean(errors**2))),
        "lateral_steady_rmse_m": float(np.sqrt(np.mean(steady**2))) if steady.size else float("nan"),
        "recovery_s": recovery,
        "lateral_max_m": float(np.max(errors)),
        "lateral_final_m": float(errors[-1]),
        "yaw_rate_rms_rad_s": float(np.sqrt(np.mean(yaw_rate**2))),
        "yaw_rate_abs_max": float(np.max(np.abs(yaw_rate))),
        "yaw_rate_jerk_rms": float(np.sqrt(np.mean(np.diff(yaw_rate) ** 2))) if len(yaw_rate) > 1 else 0.0,
        "speed_mean_m_s": float(np.mean(speed)),
        "speed_max_m_s": float(np.max(np.abs(speed))),
        "constraint_violations": int(np.sum(np.abs(speed) > 1.0 + 1e-6) + np.sum(np.abs(yaw_rate) > 1.2 + 1e-6)),
        "steps": int(len(errors)),
    }
