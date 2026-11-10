#!/usr/bin/env python3
"""UGV PID Cruise Speed Tracking Simulation.

Simulates a 1D ground-vehicle speed channel under a headwind gust
using a PID cruise controller with integral anti-windup and accel
limiting.  This module keeps a single implementation; ``main.py``
and the regression test both import from this file.
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


class UGVPlant:
    """Simplified 1D ground vehicle dynamics."""

    def __init__(self, mass=1.0, dt=0.01):
        self.m = mass
        self.dt = dt
        self.v = 0.0
        self.x = 0.0

    def step(self, accel_cmd, disturbance=0.0):
        a = accel_cmd / self.m + disturbance
        self.v = max(0.0, self.v + a * self.dt)
        self.x += self.v * self.dt
        return self.v, self.x


class PIDCruiseController:
    """PID speed controller with anti-windup and accel limiting."""

    def __init__(self, dt=0.01, kp=3.0, ki=0.3, kd=0.5,
                 i_limit=3.0, a_limit=5.0):
        self.dt = dt
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_limit = i_limit
        self.a_limit = a_limit
        self.integral = 0.0
        self.prev_err = 0.0

    def update(self, v_target, v_meas):
        err = v_target - v_meas
        self.integral = np.clip(self.integral + err * self.dt,
                                -self.i_limit, self.i_limit)
        deriv = (err - self.prev_err) / self.dt
        self.prev_err = err
        accel = self.kp * err + self.ki * self.integral + self.kd * deriv
        return np.clip(accel, -self.a_limit, self.a_limit)


def speed_profile(t):
    """Step speed profile: 0 -> 5 -> 8 -> 3 m/s."""
    if t < 2.0:
        return 0.0
    elif t < 10.0:
        return 5.0
    elif t < 15.0:
        return 8.0
    else:
        return 3.0


def wind_gust(t):
    """Headwind gust between t=12 and t=14."""
    if 12.0 < t < 14.0:
        return -1.0
    return 0.0


def simulate(dt=0.01, t_end=20.0):
    """Run the closed-loop simulation and return arrays.

    Returns
    -------
    times, vs, vts, xs, accels : np.ndarray
        Time, actual/target speed, distance, accel command.
    """
    plant = UGVPlant(dt=dt)
    ctrl = PIDCruiseController(dt=dt)
    times, vs, vts, xs, accels = [], [], [], [], []
    for t in np.arange(0, t_end, dt):
        vt = speed_profile(t)
        u = ctrl.update(vt, plant.v)
        v, x = plant.step(u, wind_gust(t))
        times.append(t)
        vs.append(v)
        vts.append(vt)
        xs.append(x)
        accels.append(u)
    return (np.array(times), np.array(vs), np.array(vts),
            np.array(xs), np.array(accels))


def evaluate(times, vs, vts, xs):
    """Report metrics with step windows separated from the gust period.

    The headwind gust acts at t=12..14s.  Step-response metrics
    (steady error, overshoot) are measured before the gust; the gust
    peak and post-gust residual are reported separately so they do not
    contaminate the step response numbers.
    """
    # 8 m/s step steady error: 11.0-11.9s (gust starts at t=12).
    step_mask = (times >= 11.0) & (times < 11.9)
    steady_err = float(np.mean(np.abs(vts[step_mask] - vs[step_mask])))
    # 8 m/s step overshoot: 10.0-12.0s (before gust onset).
    pre_gust = vs[(times >= 10.0) & (times < 12.0)]
    overshoot = float(np.max(pre_gust) - 8.0)
    # Gust peak: 12.0-14.0s, max deviation from the 8 m/s target.
    gust_mask = (times >= 12.0) & (times < 14.0)
    gust_peak = float(np.max(np.abs(vts[gust_mask] - vs[gust_mask])))
    # Residual 0.5s after gust ends (14.5-14.9s, before the 3 m/s step).
    res_mask = (times >= 14.5) & (times < 14.9)
    residual = float(np.max(np.abs(vts[res_mask] - vs[res_mask])))
    return {
        "steady_err": steady_err,
        "overshoot": overshoot,
        "gust_peak": gust_peak,
        "residual": residual,
        "final_speed": float(vs[-1]),
        "distance": float(xs[-1]),
    }


def main():
    print("=== UGV PID Cruise Speed Tracking ===")
    import sys
    print(f"Python: {sys.version.split()[0]} NumPy")

    times, vs, vts, xs, accels = simulate()
    m = evaluate(times, vs, vts, xs)

    print(f"Final speed: {m['final_speed']:.2f} m/s (target {vts[-1]:.1f})")
    print(f"Step steady error (11.0-11.9s): {m['steady_err']:.3f} m/s")
    print(f"Step overshoot (10-12s): {m['overshoot']:.3f} m/s")
    print(f"Gust peak (12-14s): {m['gust_peak']:.3f} m/s")
    print(f"Residual after gust: {m['residual']:.3f} m/s")
    print(f"Distance traveled: {m['distance']:.1f} m")

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    ax1.plot(times, vts, 'r--', label='target')
    ax1.plot(times, vs, 'b-', label='actual')
    ax1.set_ylabel('Speed (m/s)'); ax1.legend(); ax1.grid(alpha=0.3)
    ax1.set_title('UGV PID Cruise Speed Tracking')
    ax2.plot(times, accels, 'g-', label='accel cmd')
    ax2.set_ylabel('Accel (m/s²)'); ax2.set_xlabel('Time (s)')
    ax2.legend(); ax2.grid(alpha=0.3)
    fig.tight_layout()
    out = "cruise_response.png"
    fig.savefig(out, dpi=100)
    print(f"Saved: {out}")
    return m


if __name__ == "__main__":
    main()
