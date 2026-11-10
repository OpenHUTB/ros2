#!/usr/bin/env python3
"""ROV depth hold PID simulation."""
import numpy as np

print("=== ROV Depth Hold PID ===")
import sys
print(f"Python: {sys.version.split()[0]} NumPy")

dt = 0.01
t_end = 20.0
depth = 0.0  # starts at surface
vel = 0.0
target = 2.0  # hold at 2m depth
kp, ki, kd = 8.0, 1.0, 4.0
integral = 0.0
prev_err = 0.0

times, ds, us = [], [], []
for t in np.arange(0, t_end, dt):
    # Buoyancy disturbance: at t=5, a payload shifts causing upward force
    disturbance = 0.0
    if t > 5 and t < 8: disturbance = -1.5  # upward (negative depth direction)
    if t > 12: disturbance += 0.5 * np.sin(2*np.pi*0.3*(t-12))  # wave slosh

    err = target - depth
    integral += err * dt
    integral = np.clip(integral, -1, 1)
    deriv = (err - prev_err) / dt
    prev_err = err
    thrust = kp*err + ki*integral + kd*deriv

    # ROV dynamics: mass=5kg, added mass effect
    accel = thrust/5.0 + disturbance
    vel += accel * dt
    depth += vel * dt
    depth = max(0, depth)
    times.append(t); ds.append(depth); us.append(thrust)

ds = np.array(ds); times = np.array(times); us = np.array(us)
print(f"Final depth: {ds[-1]:.3f} m (target {target})")
print(f"Steady error after 15s: {np.mean(np.abs(target - ds[1500:])):.3f} m")
print(f"Max deviation after disturbance: {np.max(np.abs(target - ds[500:1500])):.3f} m")
