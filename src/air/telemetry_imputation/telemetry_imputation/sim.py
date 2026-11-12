"""Self-contained quadrotor simulator for telemetry-imputation experiments.

No external assets: rigid-body dynamics + a cascaded position/attitude controller,
integrated with RK4 at 200 Hz and recorded at 20 Hz.
"""
import numpy as np

G = 9.80665
MASS = 1.2          # kg, airframe + nominal payload
INERTIA = np.array([0.02, 0.02, 0.04])
ARM = 0.17          # m
KT = 1.6e-5         # thrust coefficient (N per rad^2/s^2, summed over rotors)
KQ = 2.6e-7         # torque coefficient
DRAG = np.array([0.12, 0.12, 0.18])   # linear drag (N per m/s)
MAX_OMEGA = 1200.0
DT = 0.005          # 200 Hz


def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])


def quat_to_rot(q):
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)],
    ])


def euler_to_quat(roll, pitch, yaw):
    cr, sr = np.cos(roll/2), np.sin(roll/2)
    cp, sp = np.cos(pitch/2), np.sin(pitch/2)
    cy, sy = np.cos(yaw/2), np.sin(yaw/2)
    return np.array([cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy,
                     cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy])


def min_jerk(t, T):
    s = np.clip(t / T, 0.0, 1.0)
    return 10*s**3 - 15*s**4 + 6*s**5, (30*s**2 - 60*s**3 + 30*s**4) / T


def waypoint_path(points, speed=1.2, rng=None):
    """Piecewise minimum-jerk path through the given points; returns p(t), v(t)."""
    points = np.asarray(points, float)
    segs = []
    t0 = 0.0
    for a, b in zip(points[:-1], points[1:]):
        dist = np.linalg.norm(b - a)
        T = max(dist / speed, 0.8)
        segs.append((t0, t0 + T, a, b))
        t0 += T
    total = t0

    def evaluate(t):
        for (ts, te, a, b) in segs:
            if t <= te or (ts, te, a, b) == segs[-1]:
                s, ds = min_jerk(t - ts, te - ts)
                return a + (b - a) * s, (b - a) * ds
        return points[-1], np.zeros(3)

    plane = [np.zeros(3)] + [p for p in points[1:]]
    return evaluate, total, plane


def controller(state, target_p, target_v, target_a, yaw_des=0.0):
    p, v, q, w = state
    e_p = target_p - p
    e_v = target_v - v
    a_des = target_a + 2.2 * e_p + 2.6 * e_v          # PD on position
    thrust_vec = MASS * (a_des + np.array([0, 0, G]))
    thrust = np.clip(np.linalg.norm(thrust_vec), 0.0, 4 * KT * MAX_OMEGA**2)
    z_body = thrust_vec / max(np.linalg.norm(thrust_vec), 1e-6)
    yaw = np.array([np.cos(yaw_des), np.sin(yaw_des), 0.0])
    x_body = np.cross(yaw, z_body)
    if np.linalg.norm(x_body) < 1e-6:
        x_body = np.array([1.0, 0, 0])
    x_body /= np.linalg.norm(x_body)
    y_body = np.cross(z_body, x_body)
    R_des = np.column_stack([x_body, y_body, z_body])
    R = quat_to_rot(q)
    e_R = 0.5 * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    e_R = 0.5 * (R_des.T @ R - R.T @ R_des)
    err = np.array([e_R[2, 1], e_R[0, 2], e_R[1, 0]])
    torque = -8.0 * err - 0.35 * w
    omega_sq = np.clip(thrust / (4 * KT), 0.0, MAX_OMEGA**2)
    return omega_sq, torque


def step(state, omega_sq, torque, wind=np.zeros(3)):
    p, v, q, w = state
    R = quat_to_rot(q)
    thrust = KT * omega_sq * 4
    f_body = np.array([0, 0, thrust])
    f = R @ f_body - DRAG * v + wind - np.array([0, 0, MASS * G])
    a = f / MASS
    w_dot = (torque - np.cross(w, INERTIA * w)) / INERTIA
    q_dot = 0.5 * quat_mul(q, np.r_[0.0, w])
    return a, w_dot, q_dot


def simulate(points, rng, wind_sigma=0.0, mass_scale=1.0, duration=None):
    """Fly a waypoint path; return arrays [t, px,py,pz, vx,vy,vz, roll,pitch,yaw, thrust].

    Control runs at 20 Hz (the recorded rate); physics integrates at 200 Hz.
    """
    global MASS
    base_mass = MASS
    MASS = base_mass * mass_scale
    try:
        path, total, _ = waypoint_path(points, rng=rng)
        if duration:
            total = max(duration, 2.0)
        state = (np.array([0, 0, 0.0]), np.zeros(3), np.array([1.0, 0, 0, 0]), np.zeros(3))
        rows = []
        dt_ctrl = 0.05
        substeps = int(round(dt_ctrl / DT))
        n_steps = int(total / dt_ctrl)
        wind = np.zeros(3)
        for k in range(n_steps):
            t = k * dt_ctrl
            if wind_sigma > 0 and k % 20 == 0:
                wind = rng.normal(0, wind_sigma, 3)
            tp, tv = path(t)
            tp2, tv2 = path(min(t + dt_ctrl, total))
            ta = (tv2 - tv) / dt_ctrl
            omega_sq, torque = controller(state, tp, tv, ta)
            for _ in range(substeps):
                a, w_dot, q_dot = step(state, omega_sq, torque, wind)
                p, v, q, w = state
                v = v + a * DT
                p = p + v * DT
                w = w + w_dot * DT
                q = q + q_dot * DT
                q = q / np.linalg.norm(q)
                state = (p, v, q, w)
            R = quat_to_rot(state[2])
            roll = np.arctan2(R[2, 1], R[2, 2])
            pitch = -np.arcsin(np.clip(R[2, 0], -1, 1))
            yaw = np.arctan2(R[1, 0], R[0, 0])
            rows.append([t, *state[0], *state[1], roll, pitch, yaw, KT * omega_sq * 4])
        return np.asarray(rows)
    finally:
        MASS = base_mass


def random_path(rng, n_points=4, radius=6.0, height=(2.0, 6.0)):
    pts = [np.zeros(3)]
    for _ in range(n_points):
        angle = rng.uniform(0, 2*np.pi)
        r = rng.uniform(1.0, radius)
        pts.append(np.array([r*np.cos(angle), r*np.sin(angle), rng.uniform(*height)]))
    pts.append(np.array([0.0, 0.0, pts[-1][2]]))
    return np.asarray(pts)
