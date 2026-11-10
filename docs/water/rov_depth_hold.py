#!/usr/bin/env python3
"""ROV Depth Hold PID Controller Simulation.

Simulates a simplified ROV vertical channel under buoyancy disturbance
and wave slosh using a PID depth-hold controller.
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


class ROVDepthPlant:
    """Simplified ROV vertical dynamics (mass + added mass)."""
    def __init__(self, mass=5.0, dt=0.01):
        self.m = mass
        self.dt = dt
        self.depth = 0.0
        self.vel = 0.0

    def step(self, thrust, disturbance=0.0):
        accel = thrust / self.m + disturbance
        self.vel += accel * self.dt
        self.depth = max(0.0, self.depth + self.vel * self.dt)
        return self.depth, self.vel


class PIDDepthController:
    """PID depth hold controller with anti-windup."""
    def __init__(self, dt=0.01, kp=8.0, ki=1.0, kd=4.0, i_limit=1.0):
        self.dt = dt
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_limit = i_limit
        self.integral = 0.0
        self.prev_err = 0.0

    def update(self, depth_target, depth_meas, vel_meas):
        err = depth_target - depth_meas
        self.integral = np.clip(self.integral + err * self.dt,
                                -self.i_limit, self.i_limit)
        deriv = (err - self.prev_err) / self.dt
        self.prev_err = err
        return self.kp * err + self.ki * self.integral + self.kd * deriv


def buoyancy_disturbance(t):
    """Upward payload shift at t=5..8, wave slosh after t=12."""
    d = 0.0
    if 5.0 < t < 8.0:
        d = -1.5
    if t > 12.0:
        d += 0.5 * np.sin(2 * np.pi * 0.3 * (t - 12.0))
    return d


def main():
    print("=== ROV Depth Hold PID ===")
    import sys
    print(f"Python: {sys.version.split()[0]} NumPy")

    dt = 0.01
    t_end = 20.0
    target = 2.0
    plant = ROVDepthPlant(dt=dt)
    ctrl = PIDDepthController(dt=dt)

    times, depths, thrusts = [], [], []
    for t in np.arange(0, t_end, dt):
        u = ctrl.update(target, plant.depth, plant.vel)
        d, v = plant.step(u, buoyancy_disturbance(t))
        times.append(t); depths.append(d); thrusts.append(u)

    times = np.array(times); depths = np.array(depths); thrusts = np.array(thrusts)

    steady_err = np.mean(np.abs(target - depths[1500:]))
    max_dev = np.max(np.abs(target - depths[500:1500]))
    print(f"Final depth: {depths[-1]:.3f} m (target {target})")
    print(f"Steady error after 15s: {steady_err:.3f} m")
    print(f"Max deviation after disturbance: {max_dev:.3f} m")

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    ax1.axhline(y=target, color='r', ls='--', label='target')
    ax1.plot(times, depths, 'b-', label='depth')
    ax1.set_ylabel('Depth (m)'); ax1.legend(); ax1.grid(alpha=0.3)
    ax1.set_title('ROV Depth Hold PID')
    ax2.plot(times, thrusts, 'g-', label='thrust')
    ax2.set_ylabel('Thrust (N)'); ax2.set_xlabel('Time (s)')
    ax2.legend(); ax2.grid(alpha=0.3)
    fig.tight_layout()
    out = "depth_response.png"
    fig.savefig(out, dpi=100)
    print(f"Saved: {out}")


if __name__ == "__main__":
    main()
