"""落脚点序列与支撑多边形判定。"""

LEFT = 'left'
RIGHT = 'right'


class Footstep:
    """单次落脚：摆动区间、落脚位置。"""

    __slots__ = ('index', 'foot', 'swing_start', 'swing_end', 'x', 'y')

    def __init__(self, index, foot, swing_start, swing_end, x, y):
        self.index = index
        self.foot = foot
        self.swing_start = swing_start
        self.swing_end = swing_end
        self.x = x
        self.y = y


def plan_footsteps(num_steps=6,
                   step_length=0.10,
                   step_width=0.06,
                   t_wait=0.15,
                   t_swing=0.40,
                   t_interval=0.50):
    """生成左右交替的落脚点序列。

    双脚初始位于 (0, +step_width) 与 (0, -step_width)。从 t_wait 之后
    开始，每 t_interval 秒摆动一只脚，摆动时长 t_swing，摆动期间另一只
    脚支撑。返回 Footstep 列表。
    """
    steps = []
    foot = LEFT
    for i in range(num_steps):
        swing_start = t_wait + i * t_interval
        swing_end = swing_start + t_swing
        x = (i + 1) * step_length
        y = step_width if foot == LEFT else -step_width
        steps.append(Footstep(i, foot, swing_start, swing_end, x, y))
        foot = RIGHT if foot == LEFT else LEFT
    return steps


def foot_rect(foot_x, foot_y, foot_length, foot_width):
    """返回脚掌矩形 (x_min, x_max, y_min, y_max)。"""
    return (foot_x - foot_length / 2.0, foot_x + foot_length / 2.0,
            foot_y - foot_width / 2.0, foot_y + foot_width / 2.0)


def rect_corners(rect):
    """返回矩形的四个角点。"""
    x_min, x_max, y_min, y_max = rect
    return [(x_min, y_min), (x_max, y_min), (x_max, y_max), (x_min, y_max)]


def _cross(origin, a, b):
    return ((a[0] - origin[0]) * (b[1] - origin[1])
            - (a[1] - origin[1]) * (b[0] - origin[0]))


def convex_hull(points):
    """Andrew 单调链算法，返回逆时针排列的凸包顶点。"""
    ordered = sorted(set(points))
    if len(ordered) <= 2:
        return ordered
    lower = []
    for point in ordered:
        while (len(lower) >= 2
               and _cross(lower[-2], lower[-1], point) <= 0.0):
            lower.pop()
        lower.append(point)
    upper = []
    for point in reversed(ordered):
        while (len(upper) >= 2
               and _cross(upper[-2], upper[-1], point) <= 0.0):
            upper.pop()
        upper.append(point)
    return lower[:-1] + upper[:-1]


def point_in_convex(point, polygon, tolerance=1e-9):
    """判断点是否在逆时针凸多边形内（含边界）。"""
    if len(polygon) < 3:
        return False
    for index, vertex in enumerate(polygon):
        following = polygon[(index + 1) % len(polygon)]
        if _cross(vertex, following, point) < -tolerance:
            return False
    return True


def zmp_inside_support(zmp_pos, rects, margin=0.0):
    """判断 ZMP 是否落在支撑多边形内。

    rects 为已落地脚掌的 foot_rect 返回值组成的列表。支撑多边形取这些
    矩形的凸包：单脚支撑时退化为该脚矩形，双支撑时包含两脚之间的连桥
    区域。margin 为向内收缩的安全裕度。
    """
    shrunk = []
    for x_min, x_max, y_min, y_max in rects:
        if x_max - x_min > 2.0 * margin and y_max - y_min > 2.0 * margin:
            shrunk.append((x_min + margin, x_max - margin,
                           y_min + margin, y_max - margin))
    if not shrunk:
        return False
    corners = [corner for rect in shrunk for corner in rect_corners(rect)]
    return point_in_convex(zmp_pos, convex_hull(corners))
