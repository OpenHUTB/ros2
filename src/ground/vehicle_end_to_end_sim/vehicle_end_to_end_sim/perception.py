"""前方视野感知算法（Vision Cone Perception）。

模拟真实车辆前向毫米波雷达 / 摄像头：仅对车辆正前方一定角度（默认 ±40°）内的
障碍物测距，过滤车身侧方与后方的无关障碍物，避免误触发刹车。

通过相对角度差 angle_diff 判断障碍物是否落在前方视野锥内：
    angle_to_obs = atan2(dy, dx)
    angle_diff   = ((angle_to_obs - theta + π) % 2π) - π
    |angle_diff| < view_half_angle  ⇒  属于前方威胁
"""

import math

from . import config


def compute_nearest_front_distance(
    car, obstacles, view_deg=config.VISION_VIEW_DEG
):
    """返回车辆前方视野锥内最近障碍物的欧氏距离（像素）。

    参数
    ----
    car : Car
        车辆对象，用于读取位姿 (x, y, theta)。
    obstacles : iterable[(float, float)]
        障碍物世界坐标列表。
    view_deg : float
        前方视野半角（度），默认 ±40°。

    返回
    ----
    float : 前方最近障碍物距离；视野内无障碍物时返回 math.inf。
    """
    min_dist = math.inf
    half_angle = math.radians(view_deg)

    for (ox, oy) in obstacles:
        dx = ox - car.x
        dy = oy - car.y
        dist = math.hypot(dx, dy)
        angle_to_obs = math.atan2(dy, dx)
        angle_diff = (angle_to_obs - car.theta + math.pi) % (2 * math.pi) - math.pi

        if abs(angle_diff) < half_angle and dist < min_dist:
            min_dist = dist

    return min_dist
