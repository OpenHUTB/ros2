#!/usr/bin/env python3
"""UAV yaw stabilization PID simulation."""
import numpy as np

print("=== UAV Yaw Stabilization ===")
import sys
print(f"Python: {sys.version.split()[0]} NumPy")

dt = 0.01
t_end = 15.0
yaw = 0.0  # radians
yaw_rate = 0.0
target = np.pi/4  # 45 degrees
kp, kd = 3.0, 2.0  # PD controller
Izz = 0.02  # moment of inertia

times, ys, rs = [], [], []
for t in np.arange(0, t_end, dt):
    # Target changes: turn to 45deg at t=0, then to -30deg at t=7
    if t < 7: target = np.pi/4
    else: target = -np.pi/6

    err = target - yaw
    # Wind gust disturbance (torque) at t=5
    torque_dist = 0.0
    if t > 5 and t < 6: torque_dist = 0.5
    if t > 10: torque_dist += 0.2 * np.sin(2*np.pi*0.8*(t-10))

    torque = kp*err - kd*yaw_rate + torque_dist
    yaw_accel = torque / Izz
    yaw_rate += yaw_accel * dt
    yaw += yaw_rate * dt
    # Normalize angle
    while yaw > np.pi: yaw -= 2*np.pi
    while yaw < -np.pi: yaw += 2*np.pi
    times.append(t); ys.append(yaw); rs.append(yaw_rate)

ys = np.array(ys); times = np.array(times)
print(f"Final yaw: {np.degrees(ys[-1]):.1f} deg (target -30)")
print(f"Steady error at 45deg: {np.degrees(np.mean(np.abs(np.pi/4 - ys[400:650]))):.2f} deg")
print(f"Max overshoot to 45deg: {np.degrees(np.max(ys[:700]) - np.pi/4):.2f} deg")
print(f"Recovery after gust: {np.degrees(np.max(np.abs(np.pi/4 - ys[500:700]))):.2f} deg")
