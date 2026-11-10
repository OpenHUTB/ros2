#!/usr/bin/env python3
"""UAV L1 Adaptive Altitude Controller Simulation."""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


class UAVAltitudePlant:
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
    def __init__(self, dt=0.01, k=2.0, c=2.0):
        self.dt = dt
        self.k = k
        self.c = c
        self.x_hat = 0.0
        self.v_hat = 0.0
        self.theta_hat = 0.0
        self.u = 0.0

    def update(self, z_setpoint, z_meas, v_meas):
        z_err = z_meas - self.x_hat
        self.theta_hat += self.k * z_err * self.dt
        self.u = self.k * (z_setpoint - z_meas) - self.c * v_meas - self.theta_hat
        accel = self.u - 9.81 + self.theta_hat
        self.v_hat += accel * self.dt
        self.x_hat += self.v_hat * self.dt
        return self.u


def gust(t):
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
    fig, ax = plt.subplots(3, 1, figsize=(8,8), sharex=True)
    ax[0].plot(t,z); ax[0].axhline(5,color='r',ls='--')
    ax[0].set_ylabel('Altitude (m)')
    ax[1].plot(t,u,color='g'); ax[1].set_ylabel('Thrust (N)')
    ax[2].plot(t,d,color='orange'); ax[2].set_ylabel('Disturbance')
    ax[2].set_xlabel('Time (s)')
    plt.tight_layout(); plt.savefig('altitude_response.png', dpi=150)
    print("Saved altitude_response.png")


if __name__ == "__main__":
    main()
