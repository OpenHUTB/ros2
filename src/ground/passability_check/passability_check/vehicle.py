"""车辆几何模型与三个通过性角度（纯标准库，无 ROS 依赖）。

坐标约定
--------
车体坐标系：「后轴接地点」为原点，x 向前，z 向上，单位米。

    x = -rear_overhang            车尾最外点
    x = 0                         后轴（后轮接地点）
    x = wheelbase                 前轴（前轮接地点）
    x = wheelbase + front_overhang 车头最外点

车身下缘按三节点折线近似：

    (-rear_overhang, clearance_rear) → (mid_ratio*wheelbase, clearance_mid)
                                     → (wheelbase + front_overhang, clearance_front)

也就是说，车底最低点在轴距的 `mid_ratio` 处（默认 0.5，即轴距中点），
三个节点的高度分别对应车尾、车底中部、车头的最低离地高度。
这是几何核算里最常用的一阶模型：真车下缘是一段连续曲面，
折线模型在「会不会蹭」这个二值判断上误差通常在厘米级，
但它算不出「蹭多深」，所以本模块同时提供按剖面数值采样的复核（见 clearance.py）。
"""

import math

# 默认车辆：一台紧凑级家用轿车（轴距 2.70 m，空载离地 140 mm）
DEFAULT_WHEELBASE = 2.70
DEFAULT_FRONT_OVERHANG = 0.88
DEFAULT_REAR_OVERHANG = 0.95
DEFAULT_CLEARANCE_FRONT = 0.14
DEFAULT_CLEARANCE_MID = 0.14
DEFAULT_CLEARANCE_REAR = 0.20
DEFAULT_MID_RATIO = 0.50
DEFAULT_TRACK = 1.54
DEFAULT_WIDTH = 1.80
DEFAULT_HEIGHT = 1.47

REQUIRED_FIELDS = (
    "wheelbase",
    "front_overhang",
    "rear_overhang",
    "clearance_front",
    "clearance_mid",
    "clearance_rear",
)


class VehicleError(ValueError):
    """车辆参数不合法。"""


def _as_float(value, name):
    if value is None or (isinstance(value, str) and not value.strip()):
        raise VehicleError("%s 不能为空" % name)
    try:
        result = float(value)
    except (TypeError, ValueError):
        raise VehicleError("%s 不是数字：%r" % (name, value))
    if math.isnan(result) or math.isinf(result):
        raise VehicleError("%s 不是有限数：%r" % (name, value))
    return result


def _positive(value, name):
    result = _as_float(value, name)
    if result <= 0.0:
        raise VehicleError("%s 必须大于 0：%r" % (name, result))
    return result


def _non_negative(value, name):
    result = _as_float(value, name)
    if result < 0.0:
        raise VehicleError("%s 不能为负：%r" % (name, result))
    return result


class VehicleSpec(object):
    """车辆几何参数，以及由它决定的三个通过性角度。"""

    __slots__ = (
        "name",
        "wheelbase",
        "front_overhang",
        "rear_overhang",
        "clearance_front",
        "clearance_mid",
        "clearance_rear",
        "mid_ratio",
        "track",
        "width",
        "height",
    )

    def __init__(self, name="sedan", wheelbase=DEFAULT_WHEELBASE,
                 front_overhang=DEFAULT_FRONT_OVERHANG,
                 rear_overhang=DEFAULT_REAR_OVERHANG,
                 clearance_front=DEFAULT_CLEARANCE_FRONT,
                 clearance_mid=DEFAULT_CLEARANCE_MID,
                 clearance_rear=DEFAULT_CLEARANCE_REAR,
                 mid_ratio=DEFAULT_MID_RATIO, track=DEFAULT_TRACK,
                 width=DEFAULT_WIDTH, height=DEFAULT_HEIGHT):
        self.name = name
        self.wheelbase = _positive(wheelbase, "wheelbase")
        self.front_overhang = _positive(front_overhang, "front_overhang")
        self.rear_overhang = _positive(rear_overhang, "rear_overhang")
        self.clearance_front = _non_negative(clearance_front, "clearance_front")
        self.clearance_mid = _non_negative(clearance_mid, "clearance_mid")
        self.clearance_rear = _non_negative(clearance_rear, "clearance_rear")
        self.mid_ratio = _as_float(mid_ratio, "mid_ratio")
        if not 0.0 < self.mid_ratio < 1.0:
            raise VehicleError("mid_ratio 必须在 (0, 1) 之间：%r" % (mid_ratio,))
        self.track = _positive(track, "track")
        self.width = _positive(width, "width")
        self.height = _positive(height, "height")
        if self.width + 1e-9 < self.track:
            raise VehicleError("车宽 %.3f 小于轮距 %.3f" % (self.width, self.track))

    # ------------------------------------------------------------ 派生尺寸
    @property
    def length(self):
        """车长：前悬 + 轴距 + 后悬。"""
        return self.front_overhang + self.wheelbase + self.rear_overhang

    @property
    def mid_position(self):
        """车底最低点在后轴坐标系里的 x 位置。"""
        return self.mid_ratio * self.wheelbase

    @property
    def min_clearance(self):
        """三个测点里最低的离地高度。"""
        return min(self.clearance_front, self.clearance_mid, self.clearance_rear)

    @property
    def approach_deg(self):
        """接近角：前轮接地点到车头最低点的连线与地面的夹角。"""
        return math.degrees(math.atan2(self.clearance_front, self.front_overhang))

    @property
    def departure_deg(self):
        """离去角：后轮接地点到车尾最低点的连线与地面的夹角。"""
        return math.degrees(math.atan2(self.clearance_rear, self.rear_overhang))

    @property
    def breakover_deg(self):
        """纵向通过角：两条接地点切线的夹角。

        车底最低点在 x = mid_position、高 clearance_mid，
        两条切线相对水平面各抬起
            atan(clearance_mid / mid_position) 与 atan(clearance_mid / (L - mid_position))
        两者之和就是能跨过的最大上凸折角。mid_ratio = 0.5 时退化为
            2 * atan(2 * clearance_mid / L)
        """
        front_leg = math.atan2(self.clearance_mid, self.mid_position)
        rear_leg = math.atan2(self.clearance_mid, self.wheelbase - self.mid_position)
        return math.degrees(front_leg + rear_leg)

    # ------------------------------------------------------------ 下缘折线
    def lower_knots(self):
        """车身下缘的三个折点（车体坐标系，单位米）。"""
        return (
            (-self.rear_overhang, self.clearance_rear),
            (self.mid_position, self.clearance_mid),
            (self.wheelbase + self.front_overhang, self.clearance_front),
        )

    def lower_contour(self, samples=41):
        """把下缘折线等弧长加密成采样点，返回 [(x, z), ...]。"""
        samples = int(samples)
        if samples < 3:
            raise VehicleError("下缘采样点至少要 3 个：%d" % samples)
        knots = self.lower_knots()
        result = []
        # 按两段折线各自的长度分配采样点，保证拐点一定被采到
        lengths = []
        for index in range(len(knots) - 1):
            x0, z0 = knots[index]
            x1, z1 = knots[index + 1]
            lengths.append(math.hypot(x1 - x0, z1 - z0))
        total = sum(lengths)
        counts = []
        for index, length in enumerate(lengths):
            raw = samples * length / total
            counts.append(max(2, int(round(raw))))
        # 拐点不重复采样，所以唯一点数 = sum(counts) - (段数 - 1)。
        # 把 round 的零头补到最后一段，保证返回的点数正好等于 samples
        # （以前末尾多 append 一次，41 个点里只有 40 个唯一坐标）。
        segments = len(knots) - 1
        diff = (samples + segments - 1) - sum(counts)
        counts[-1] = max(2, counts[-1] + diff)
        for index in range(len(knots) - 1):
            (x0, z0), (x1, z1) = knots[index], knots[index + 1]
            count = counts[index]
            # 最后一段的末点由函数末尾统一追加（保证精确落在最后折点上），
            # 这里不生成，否则末点会重复一次（评审 @KKK555-MAX 指出：
            # 41 个点里只有 40 个唯一坐标）
            span = count - 1 if index == len(knots) - 2 else count
            for step in range(span):
                if index > 0 and step == 0:
                    continue  # 拐点不重复
                ratio = float(step) / (count - 1)
                result.append((x0 + (x1 - x0) * ratio, z0 + (z1 - z0) * ratio))
        result.append(knots[-1])
        return result

    def roof_contour(self):
        """车顶轮廓（只用于画图）：从前保险杠上沿到后保险杠上沿的一段矩形。"""
        return (
            (-self.rear_overhang, self.height),
            (self.wheelbase + self.front_overhang, self.height),
        )

    def body_outline(self):
        """车身侧视轮廓的上缘（车体坐标，由车尾到车头，x 单调递增）。

        只用于字符图：后保险杠上沿 → 后备箱 → 后风挡顶 → 车顶 → 前风挡下沿
        → 引擎盖后沿 → 前保险杠上沿。高度按车高 `height` 取比例。
        """
        total = self.length
        base = -self.rear_overhang
        roof_rear = base + 0.30 * total
        roof_front = base + 0.68 * total
        return (
            (base, 0.40 * self.height),
            (base + 0.13 * total, 0.58 * self.height),
            (roof_rear, self.height),
            (roof_front, self.height),
            (roof_front + 0.11 * total, 0.66 * self.height),
            (self.wheelbase + self.front_overhang, 0.40 * self.height),
        )

    def __repr__(self):
        return ("VehicleSpec(name=%r, wheelbase=%.2f, front_overhang=%.2f, rear_overhang=%.2f,"
                " clearance=%.2f/%.2f/%.2f, mid_ratio=%.2f)"
                % (self.name, self.wheelbase, self.front_overhang, self.rear_overhang,
                   self.clearance_front, self.clearance_mid, self.clearance_rear,
                   self.mid_ratio))


# 仓库里演示用的两台车：一台家用轿车、一台紧凑型 SUV
SEDAN = VehicleSpec(
    name="sedan",
    wheelbase=2.70,
    front_overhang=0.88,
    rear_overhang=0.95,
    clearance_front=0.14,
    clearance_mid=0.14,
    clearance_rear=0.20,
    track=1.54,
    width=1.80,
    height=1.47,
)

SUV = VehicleSpec(
    name="suv",
    wheelbase=2.66,
    front_overhang=0.85,
    rear_overhang=0.92,
    clearance_front=0.22,
    clearance_mid=0.20,
    clearance_rear=0.28,
    track=1.57,
    width=1.85,
    height=1.68,
)

PRESET_VEHICLES = {"sedan": SEDAN, "suv": SUV}


def get_vehicle(name):
    """按名字取预置车辆；找不到时报错并列出可选项。"""
    key = str(name).strip().lower()
    if key not in PRESET_VEHICLES:
        raise VehicleError("未知车型 %r，可选：%s"
                           % (name, ", ".join(sorted(PRESET_VEHICLES))))
    return PRESET_VEHICLES[key]
