#!/usr/bin/env python3
"""UGV PID Cruise Speed Tracking Simulation.

Simulates a simple ground vehicle speed channel under wind gust
disturbance using a PID cruise controller with anti-windup.
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


def main():
    print("=== UGV PID Cruise Speed Tracking ===")
    import sys
    print(f"Python: {sys.version.split()[0]} NumPy")

    dt = 0.01
    t_end = 20.0
    plant = UGVPlant(dt=dt)
    ctrl = PIDCruiseController(dt=dt)

    times, vs, vts, xs, accels = [], [], [], [], []
    for t in np.arange(0, t_end, dt):
        vt = speed_profile(t)
        u = ctrl.update(vt, plant.v)
        v, x = plant.step(u, wind_gust(t))
        times.append(t); vs.append(v); vts.append(vt); xs.append(x); accels.append(u)

    times = np.array(times); vs = np.array(vs); vts = np.array(vts)
    xs = np.array(xs); accels = np.array(accels)

    # Metrics
    steady_err = np.mean(np.abs(vts[1100:1400] - vs[1100:1400]))
    overshoot = np.max(vs[1000:1300]) - 8.0
    print(f"Final speed: {vs[-1]:.2f} m/s (target {vts[-1]:.1f})")
    print(f"Steady error at 8m/s: {steady_err:.3f} m/s")
    print(f"Overshoot to 8m/s: {overshoot:.3f} m/s")
    print(f"Distance traveled: {xs[-1]:.1f} m")

    # Plot
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


if __name__ == "__main__":
    main()
