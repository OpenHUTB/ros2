"""视线制导（LOS）航点跟踪的纯算法实现，无 ROS 依赖。"""

import math

WAYPOINTS = [(4.0, 0.0, 1.5), (8.0, 3.0, 2.0), (12.0, 0.0, 1.5), (16.0, -3.0, 1.5)]


def wrap_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


def segment_tracking(prev, wp, pos):
    """相对上一航点-当前航点连线的纵向进度与横向误差。"""
    px, py = prev[0], prev[1]
    wx, wy = wp[0], wp[1]
    dx, dy = wx - px, wy - py
    length = math.hypot(dx, dy)
    if length < 1e-9:
        return 0.0, math.hypot(pos[0] - wx, pos[1] - wy)
    ux, uy = dx / length, dy / length
    along = (pos[0] - px) * ux + (pos[1] - py) * uy
    cross = -(pos[0] - px) * uy + (pos[1] - py) * ux
    return along, cross


def los_heading(prev, wp, pos, lookahead=2.0):
    """LOS 期望航向：路径方向减去横向偏差的修正角。"""
    path_dir = math.atan2(wp[1] - prev[1], wp[0] - prev[0])
    _, cross = segment_tracking(prev, wp, pos)
    correction = math.atan2(cross, max(lookahead, 1e-3))
    return wrap_angle(path_dir - correction)


def body_velocity(heading_error, speed, lateral_gain=0.4):
    """把期望航向转成机体系速度指令 (vx, vy)。"""
    vx = speed * math.cos(heading_error)
    vy = speed * lateral_gain * math.sin(heading_error)
    return vx, vy


def waypoint_reached(pos, wp, radius=0.8):
    return math.hypot(pos[0] - wp[0], pos[1] - wp[1]) <= radius


def guidance_step(pos, yaw, prev, wp, speed=1.5, lookahead=2.0):
    """一步制导：返回 (vx, vy, heading_error, distance)。"""
    dist = math.hypot(wp[0] - pos[0], wp[1] - pos[1])
    if dist < 1e-6:
        return 0.0, 0.0, 0.0, 0.0
    if math.hypot(wp[0] - prev[0], wp[1] - prev[1]) < 1e-6:
        desired = math.atan2(wp[1] - pos[1], wp[0] - pos[0])
    else:
        desired = los_heading(prev, wp, pos, lookahead)
    err = wrap_angle(desired - yaw)
    vx, vy = body_velocity(err, speed)
    return vx, vy, err, dist
