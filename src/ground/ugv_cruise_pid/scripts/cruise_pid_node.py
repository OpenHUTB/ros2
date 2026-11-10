#!/usr/bin/env python3
"""UGV PID cruise speed tracking simulation."""
import numpy as np

print("=== UGV PID Cruise Speed Tracking ===")
import sys
print(f"Python: {sys.version.split()[0]} NumPy")

dt = 0.01
t_end = 20.0
v = 0.0
x = 0.0
kp, ki, kd = 3.0, 0.3, 0.5
integral = 0.0
prev_err = 0.0

times, vs, vts, xs = [], [], [], []
for t in np.arange(0, t_end, dt):
    if t < 2: vt = 0.0
    elif t < 10: vt = 5.0
    elif t < 15: vt = 8.0
    else: vt = 3.0

    err = vt - v
    integral += err * dt
    integral = np.clip(integral, -3, 3)
    deriv = (err - prev_err) / dt
    prev_err = err
    accel = kp*err + ki*integral + kd*deriv
    accel = np.clip(accel, -5, 5)

    # Simple vehicle dynamics
    v += accel * dt
    v = max(0, v)
    x += v * dt
    times.append(t); vs.append(v); vts.append(vt); xs.append(x)

vs = np.array(vs); vts = np.array(vts); times = np.array(times); xs = np.array(xs)
print(f"Final speed: {vs[-1]:.2f} m/s (target {vts[-1]:.1f})")
print(f"Steady error at 8m/s: {np.mean(np.abs(vts[1100:1400] - vs[1100:1400])):.3f} m/s")
print(f"Overshoot to 8m/s: {np.max(vs[1000:1300]) - 8.0:.3f} m/s")
print(f"Distance traveled: {xs[-1]:.1f} m")
