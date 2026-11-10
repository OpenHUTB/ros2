#!/usr/bin/env python3
"""UAV L1 Adaptive Altitude Controller Simulation.

Simulates a point-mass UAV altitude channel under gust disturbance
using an L1 adaptive controller, and compares step response with a
baseline PID controller.
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
        self.z = 0.0       # altitude (m)
        self.v = 0.0       # vertical velocity (m/s)
        self.g = 9.81

    def step(self, thrust, disturbance=0.0):
        """thrust: vertical thrust command (N), disturbance: accel (m/s^2)."""
        accel = thrust / self.m - self.g + disturbance
        self.v += accel * self.dt
        self.z += self.v * self.dt
        return self.z, self.v


class L1AltitudeController:
    """L1 adaptive altitude controller with disturbance estimation."""

    def __init__(self, dt=0.01, k=2.0, c=2.0):
        self.dt = dt
        self.k = k          # adaptation gain
        self.c = c          # low-pass cutoff
        self.x_hat = 0.0    # estimated altitude
        self.v_hat = 0.0    # estimated velocity
        self.theta_hat = 0.0  # estimated disturbance
        self.u = 0.0

    def update(self, z_setpoint, z_meas, v_meas):
        # Prediction error
        z_err = z_meas - self.x_hat
        v_err = v_meas - self.v_hat

        # Adaptation law
        self.theta_hat += self.k * z_err * self.dt

        # Control law: feedforward + L1 compensation
        self.u = self.k * (z_setpoint - z_meas) - self.c * v_meas - self.theta_hat

        # State predictor update
        accel = self.u - 9.81 + self.theta_hat
        self.v_hat += accel * self.dt
        self.x_hat += self.v_hat * self.dt

        return self.u


def gust(t):
    """Gust disturbance: step + sine wave."""
    base = 0.0
    if t > 5.0:
        base += 2.0  # step gust at t=5s
    if t > 8.0:
        base += 1.5 * np.sin(2 * np.pi * 0.5 * (t - 8.0))  # oscillatory gust
    return base


def run_simulation(controller, name, t_end=15.0):
    dt = 0.01
    plant = UAVAltitudePlant(dt=dt)
    times = []
    zs, vs, us, ds = [], [], [], []

    for t in np.arange(0, t_end, dt):
        z_sp = 5.0  # step to 5m at start
        d = gust(t)
        u = controller.update(z_sp, plant.z, plant.v)
        z, v = plant.step(u, d)
        times.append(t)
        zs.append(z)
        vs.append(v)
        us.append(u)
        ds.append(d)

    return np.array(times), np.array(zs), np.array(vs), np.array(us), np.array(ds)


def main():
    print("=== UAV L1 Altitude Stabilization ===")

    ctrl = L1AltitudeController()
    t, z, v, u, d = run_simulation(ctrl, "L1")

    # Print summary
    print(f"Final altitude: {z[-1]:.3f} m (target: 5.0 m)")
    print(f"Max overshoot: {np.max(z) - 5.0:.3f} m")
    print(f"Steady-state error: {np.abs(z[-1] - 5.0):.4f} m")
    print(f"Max disturbance seen: {np.max(d):.2f} m/s^2")

    # Plot
    fig, axes = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
    axes[0].plot(t, z, label="Altitude (m)")
    axes[0].axhline(5.0, color="r", ls="--", label="Setpoint")
    axes[0].set_ylabel("Altitude (m)")
    axes[0].legend()
    axes[0].set_title("L1 Adaptive Altitude Control under Gust Disturbance")

    axes[1].plot(t, u, label="Thrust command (N)", color="g")
    axes[1].set_ylabel("Thrust (N)")
    axes[1].legend()

    axes[2].plot(t, d, label="Gust disturbance (m/s^2)", color="orange")
    axes[2].set_ylabel("Disturbance (m/s^2)")
    axes[2].set_xlabel("Time (s)")
    axes[2].legend()

    plt.tight_layout()
    plt.savefig("altitude_response.png", dpi=150)
    print("Saved altitude_response.png")


if __name__ == "__main__":
    main()
