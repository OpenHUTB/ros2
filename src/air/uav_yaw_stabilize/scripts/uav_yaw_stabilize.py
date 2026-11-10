#!/usr/bin/env python3
"""UAV Yaw Stabilization PD Controller Simulation.

Simulates a simplified UAV yaw channel under wind gust torque
using a PD heading stabilizer with angle wrapping.
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


class UAVYawPlant:
    """Simplified UAV yaw dynamics (rotational about Z)."""
    def __init__(self, Izz=0.02, dt=0.01):
        self.Izz = Izz
        self.dt = dt
        self.yaw = 0.0
        self.yaw_rate = 0.0

    def step(self, torque, disturbance=0.0):
        accel = (torque + disturbance) / self.Izz
        self.yaw_rate += accel * self.dt
        self.yaw += self.yaw_rate * self.dt
        # Wrap to [-pi, pi]
        while self.yaw > np.pi:
            self.yaw -= 2 * np.pi
        while self.yaw < -np.pi:
            self.yaw += 2 * np.pi
        return self.yaw, self.yaw_rate


class PDYawController:
    """PD yaw stabilizer."""
    def __init__(self, dt=0.01, kp=3.0, kd=2.0):
        self.dt = dt
        self.kp = kp
        self.kd = kd

    def update(self, yaw_target, yaw_meas, yaw_rate_meas):
        err = yaw_target - yaw_meas
        return self.kp * err - self.kd * yaw_rate_meas


def yaw_profile(t):
    """Step heading: 45 deg for t<7, then -30 deg."""
    if t < 7.0:
        return np.pi / 4
    else:
        return -np.pi / 6


def wind_torque(t):
    """Gust torque at t=5..6, oscillating after t=10."""
    td = 0.0
    if 5.0 < t < 6.0:
        td = 0.5
    if t > 10.0:
        td += 0.2 * np.sin(2 * np.pi * 0.8 * (t - 10.0))
    return td


def main():
    print("=== UAV Yaw Stabilization ===")
    import sys
    print(f"Python: {sys.version.split()[0]} NumPy")

    dt = 0.01
    t_end = 15.0
    plant = UAVYawPlant(dt=dt)
    ctrl = PDYawController(dt=dt)

    times, yaws, torques = [], [], []
    for t in np.arange(0, t_end, dt):
        target = yaw_profile(t)
        u = ctrl.update(target, plant.yaw, plant.yaw_rate)
        y, r = plant.step(u, wind_torque(t))
        times.append(t); yaws.append(y); torques.append(u)

    times = np.array(times); yaws = np.array(yaws); torques = np.array(torques)

    # --- Step response metrics (before gust, t < 5s) ---
    # Gust torque acts at t=5..6s, so step windows must end before t=5s.
    # Steady-state error over 4.0-4.9s.
    step_mask = (times >= 4.0) & (times < 4.9)
    steady_err = np.degrees(np.mean(np.abs(np.pi / 4 - yaws[step_mask])))
    # Overshoot over 0-5s (before gust onset).
    pre_gust = yaws[times < 5.0]
    overshoot = np.degrees(np.max(pre_gust) - np.pi / 4)

    # --- Gust metrics (5-6s) ---
    gust_mask = (times >= 5.0) & (times < 6.0)
    gust_peak = np.degrees(np.max(np.abs(np.pi / 4 - yaws[gust_mask])))

    # --- Recovery after gust (t > 6s) ---
    # Residual 0.9s after gust ends (t=6.9s, before the second step at t=7s).
    res_mask = (times >= 6.9) & (times < 7.0)
    residual = np.degrees(np.max(np.abs(np.pi / 4 - yaws[res_mask])))
    # Recovery time: first time after t=6s the error returns within 1% band
    # of the 45 deg heading.
    band = np.radians(0.01 * 45.0)
    post_gust = times >= 6.0
    rec_idx = np.where(post_gust & (np.abs(np.pi / 4 - yaws) <= band))[0]
    recovery_time = (times[rec_idx[0]] - 6.0) if len(rec_idx) else None

    print(f"Final yaw: {np.degrees(yaws[-1]):.1f} deg (target -30)")
    print(f"Step steady error (4.0-4.9s): {steady_err:.3f} deg")
    print(f"Step overshoot (0-5s): {overshoot:.3f} deg")
    print(f"Gust peak (5-6s): {gust_peak:.3f} deg")
    print(f"Residual 0.9s after gust: {residual:.3f} deg")
    if recovery_time is not None:
        print(f"Recovery time to 1% band: {recovery_time:.2f} s")
    else:
        print("Recovery time to 1% band: not reached before next step")

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    tgt = np.array([yaw_profile(t) for t in times])
    ax1.plot(times, np.degrees(tgt), 'r--', label='target')
    ax1.plot(times, np.degrees(yaws), 'b-', label='actual')
    ax1.set_ylabel('Yaw (deg)'); ax1.legend(); ax1.grid(alpha=0.3)
    ax1.set_title('UAV Yaw Stabilization PD')
    ax2.plot(times, torques, 'g-', label='torque')
    ax2.set_ylabel('Torque (N·m)'); ax2.set_xlabel('Time (s)')
    ax2.legend(); ax2.grid(alpha=0.3)
    fig.tight_layout()
    out = "yaw_response.png"
    fig.savefig(out, dpi=100)
    print(f"Saved: {out}")


if __name__ == "__main__":
    main()
