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


def zmp_inside_support(zmp_pos, rects, margin=0.0):
    """判断 ZMP 是否落在支撑多边形（各脚掌矩形之并）内。

    rects 为 foot_rect 返回值组成的列表，margin 为向内收缩的安全裕度。
    """
    for rect in rects:
        x_min, x_max, y_min, y_max = rect
        if (x_min + margin <= zmp_pos[0] <= x_max - margin
                and y_min + margin <= zmp_pos[1] <= y_max - margin):
            return True
    return False
