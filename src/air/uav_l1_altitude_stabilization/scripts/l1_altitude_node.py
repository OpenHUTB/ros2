#!/usr/bin/env python3
"""UAV L1 Adaptive Altitude Controller Simulation.

Simulates a point-mass UAV altitude channel under gust disturbance
using an L1 adaptive controller (PD feedback + disturbance estimation).
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


class UAVAltitudePlant:
    """Simplified point-mass altitude dynamics (vertical only)."""
    def __init__(self, mass=1.0, dt=0.01):
        self.m = mass
        self.dt = dt
        self.z = 0.0
        self.v = 0.0
        self.g = 9.81

    def step(self, thrust, disturbance=0.0):
        accel = thrust / self.m - self.g + disturbance
        self.v += accel * self.dt
        self.z += self.v * self.dt
        return self.z, self.v


class L1AltitudeController:
    """L1 adaptive altitude controller: PD feedback + disturbance compensation."""
    def __init__(self, dt=0.01, kp=12.0, kd=6.0, gamma=20.0):
        self.dt = dt
        self.kp = kp
        self.kd = kd
        self.gamma = gamma
        self.theta_hat = 0.0

    def update(self, z_setpoint, z_meas, v_meas):
        e = z_setpoint - z_meas
        feedback = self.kp * e - self.kd * v_meas
        # L1 adaptation: estimate lumped disturbance
        self.theta_hat += self.gamma * e * self.dt
        self.theta_hat = 0.95 * self.theta_hat + 0.05 * (self.gamma * e * self.dt * 10)
        self.u = 9.81 + feedback + self.theta_hat
        return self.u


def gust(t):
    """Gust: step at t=5s (+2 m/s^2), sine at t=8s."""
    base = 0.0
    if t > 5.0:
        base += 2.0
    if t > 8.0:
        base += 1.5 * np.sin(2 * np.pi * 0.5 * (t - 8.0))
    return base


def run_sim(ctrl, t_end=15.0):
    dt = 0.01
    plant = UAVAltitudePlant(dt=dt)
    times, zs, us, ds = [], [], [], []
    for t in np.arange(0, t_end, dt):
        u = ctrl.update(5.0, plant.z, plant.v)
        z, v = plant.step(u, gust(t))
        times.append(t); zs.append(z); us.append(u); ds.append(gust(t))
    return np.array(times), np.array(zs), np.array(us), np.array(ds)


def main():
    print("=== UAV L1 Altitude Stabilization ===")
    ctrl = L1AltitudeController()
    t, z, u, d = run_sim(ctrl)
    print(f"Final altitude: {z[-1]:.3f} m (target 5.0)")
    print(f"Max overshoot: {np.max(z)-5.0:.3f} m")
    print(f"Steady-state error: {abs(z[-1]-5.0):.4f} m")
    post_gust = t > 5.0
    print(f"Max deviation after gust: {np.max(np.abs(z[post_gust]-5.0)):.3f} m")

    fig, ax = plt.subplots(3, 1, figsize=(8,8), sharex=True)
    ax[0].plot(t, z, label="Altitude", linewidth=1.5)
    ax[0].axhline(5.0, color='r', ls='--', label="Setpoint (5m)")
    ax[0].set_ylabel("Altitude (m)")
    ax[0].set_title("UAV L1 Adaptive Altitude Control under Gust Disturbance")
    ax[0].legend(); ax[0].grid(True, alpha=0.3)
    ax[1].plot(t, u, color='g', linewidth=1.5)
    ax[1].set_ylabel("Thrust (N)"); ax[1].grid(True, alpha=0.3)
    ax[2].plot(t, d, color='orange', linewidth=1.5)
    ax[2].set_ylabel("Disturbance (m/s^2)"); ax[2].set_xlabel("Time (s)")
    ax[2].grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig("altitude_response.png", dpi=150)
    print("Saved altitude_response.png")


if __name__ == "__main__":
    main()
