"""倒车纯追踪的纯算法实现（无 ROS 依赖）。"""

import math

# 默认 S 形参考路径，节点与离线复现共用
DEFAULT_PATH = [(0.0, 0.0), (2.0, 0.0), (3.5, 0.8), (5.0, 0.8), (6.5, 0.0), (8.0, 0.0)]


def wrap_angle(angle):
    """把角度归一化到 (-pi, pi]。"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


def path_length(path):
    total = 0.0
    for (x1, y1), (x2, y2) in zip(path[:-1], path[1:]):
        total += math.hypot(x2 - x1, y2 - y1)
    return total


def cumulative_s(path):
    """路径各顶点的累计弧长。"""
    s = [0.0]
    for (x1, y1), (x2, y2) in zip(path[:-1], path[1:]):
        s.append(s[-1] + math.hypot(x2 - x1, y2 - y1))
    return s


def point_at_s(path, s_query):
    """按弧长在折线上插值取点。"""
    s = cumulative_s(path)
    total = s[-1]
    q = max(0.0, min(total, s_query))
    for i in range(len(path) - 1):
        if s[i + 1] >= q - 1e-9:
            seg = s[i + 1] - s[i]
            if seg < 1e-9:
                return path[i + 1]
            r = (q - s[i]) / seg
            x1, y1 = path[i]
            x2, y2 = path[i + 1]
            return (x1 + r * (x2 - x1), y1 + r * (y2 - y1))
    return path[-1]


def project_to_path(pose, path):
    """把位姿投影到折线路径，返回 (弧长 s, 有向横向误差, 最近点)。

    横向误差以路径切线左侧为正。
    """
    x, y = pose[0], pose[1]
    s_cum = cumulative_s(path)
    best = None
    for i in range(len(path) - 1):
        x1, y1 = path[i]
        x2, y2 = path[i + 1]
        dx, dy = x2 - x1, y2 - y1
        seg2 = dx * dx + dy * dy
        if seg2 < 1e-12:
            continue
        t = max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / seg2))
        px, py = x1 + t * dx, y1 + t * dy
        dist = math.hypot(x - px, y - py)
        if best is None or dist < best[0]:
            seg_len = math.sqrt(seg2)
            lateral = (-(x - px) * dy + (y - py) * dx) / seg_len
            best = (dist, s_cum[i] + t * seg_len, lateral, (px, py))
    if best is None:
        raise ValueError("path needs at least two distinct points")
    return best[1], best[2], best[3]


def lookahead_point(path, s_now, lookahead):
    """沿路径前进方向取前视点。

    路径顶点顺序即行进方向；倒车时车头朝向与路径切线相反，因此
    前视点在车身坐标系里位于"后方"。
    """
    return point_at_s(path, s_now + lookahead)


def pure_pursuit_curvature(pose, target, reverse=False, min_lookahead=0.05):
    """纯追踪曲率：kappa = 2 sin(alpha) / L。"""
    x, y, theta = pose
    heading = theta + math.pi if reverse else theta
    dx, dy = target[0] - x, target[1] - y
    dist = math.hypot(dx, dy)
    if dist < min_lookahead:
        return 0.0, dist
    alpha = wrap_angle(math.atan2(dy, dx) - heading)
    return 2.0 * math.sin(alpha) / dist, dist


def rate_limit(current, target, max_step):
    """把 target 约束在 current ± max_step 内。"""
    delta = target - current
    if delta > max_step:
        delta = max_step
    elif delta < -max_step:
        delta = -max_step
    return current + delta
