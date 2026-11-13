"""队形几何：按领航者朝向旋转偏移，并统计队形误差。"""

import math

from . import config


def rotate(vec, ang):
    cos_a = math.cos(ang)
    sin_a = math.sin(ang)
    return (vec[0] * cos_a - vec[1] * sin_a,
            vec[0] * sin_a + vec[1] * cos_a)


def desired_positions(leader_pos, leader_heading, offsets=None):
    """把各成员偏移旋转到世界坐标，返回期望位置列表。"""
    offsets = ([config.LEADER_OFFSET] + list(config.FOLLOWER_OFFSETS)
               if offsets is None else offsets)
    out = []
    for off in offsets:
        r = rotate(off, leader_heading)
        out.append((leader_pos[0] + r[0], leader_pos[1] + r[1]))
    return out


def formation_error(positions, desired):
    """各成员与期望位置距离的均方根。"""
    if not positions:
        return 0.0
    s = 0.0
    for p, d in zip(positions, desired):
        s += (p[0] - d[0]) ** 2 + (p[1] - d[1]) ** 2
    return math.sqrt(s / len(positions))


def leader_state(step, center=None, radius=None, omega=None, phase0=None):
    """圆周轨迹上的位置与切向朝向。"""
    center = config.CENTER if center is None else center
    radius = config.RADIUS if radius is None else radius
    omega = config.OMEGA if omega is None else omega
    phase0 = config.PHASE0 if phase0 is None else phase0
    a = phase0 + omega * step
    pos = (center[0] + radius * math.cos(a), center[1] + radius * math.sin(a))
    heading = a + math.pi / 2.0
    return pos, heading
