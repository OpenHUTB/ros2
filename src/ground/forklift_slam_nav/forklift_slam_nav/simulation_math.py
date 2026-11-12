import math


def yaw_from_freejoint(qpos, adr):
    qw, qx, qy, qz = qpos[adr + 3:adr + 7]
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def advance_planar_pose(x, y, yaw, speed, yaw_rate, dt):
    yaw += yaw_rate * dt
    x += speed * math.cos(yaw) * dt
    y += speed * math.sin(yaw) * dt
    return x, y, yaw
