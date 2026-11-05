#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""倒车入库轨迹规划核心逻辑。

本模块只做几何计算，不 import rospy，也不读写文件；ROS 侧的收发放进
parking_planner_node.py，因此这里可以用纯标准库离线跑测试。

坐标系约定：一律用车辆后轴中心 + 航向角（rad，逆时针为正）描述位姿，
车身从后轴沿航向向前延伸 length 米、左右各宽 width/2 米。
"""

from __future__ import division, print_function, unicode_literals

import math
import sys

# ---------------------------------------------------------------- 默认参数

DEFAULT_SLOT_KIND = "perpendicular"
DEFAULT_SLOT_YAW = -math.pi / 2.0
DEFAULT_SLOT_WIDTH = 2.50
DEFAULT_SLOT_LENGTH = 5.30
DEFAULT_WHEELBASE = 2.70
DEFAULT_VEHICLE_LENGTH = 4.50
DEFAULT_VEHICLE_WIDTH = 1.80
DEFAULT_MAX_STEER = 0.55
DEFAULT_ENTRY_LENGTH = 3.00
DEFAULT_TURN_RADIUS = 5.00
DEFAULT_STEP = 0.10
DEFAULT_TOLERANCE = 0.10
DEFAULT_CLEARANCE = 0.00
DEFAULT_MAX_PATH = 12.00
DEFAULT_PUBLISH_HZ = 10.0
DEFAULT_GRID_WIDTH = 56
DEFAULT_GRID_HEIGHT = 16

SLOT_KINDS = ("perpendicular", "parallel")

GEAR_REVERSE = -1
GEAR_FORWARD = 1

_EPS = 1e-9


class ParkingError(ValueError):
    """规划参数不合法或几何上不可达时抛出。"""


# ---------------------------------------------------------------- 小工具


def _positive(name, value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise ParkingError("%s 必须是数字，收到 %r" % (name, value))
    if number <= 0.0 or math.isnan(number) or math.isinf(number):
        raise ParkingError("%s 必须是正的有限数，收到 %r" % (name, value))
    return number


def normalize_angle(angle):
    """把角度折算到 (-pi, pi]。"""
    value = float(angle)
    while value > math.pi:
        value -= 2.0 * math.pi
    while value <= -math.pi:
        value += 2.0 * math.pi
    return value


def distance(a, b):
    """两个位姿（或两个 (x, y) 点）之间的欧氏距离。"""
    return math.hypot(a[0] - b[0], a[1] - b[1])


def yaw_to_quaternion(yaw):
    """平面航向角转四元数，返回 (x, y, z, w)。"""
    half = normalize_angle(yaw) / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


def quaternion_to_yaw(x, y, z, w):
    """四元数取回平面航向角。"""
    return normalize_angle(math.atan2(2.0 * (w * z + x * y),
                                      1.0 - 2.0 * (y * y + z * z)))


class Pose(object):
    """后轴中心 + 航向角。"""

    __slots__ = ("x", "y", "yaw")

    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x = float(x)
        self.y = float(y)
        self.yaw = float(yaw)

    def as_tuple(self):
        return (self.x, self.y, self.yaw)

    def __getitem__(self, index):
        """让 Pose 也能当下标用（distance 等函数因此可以通吃位姿与元组）。"""
        return self.as_tuple()[index]

    def __len__(self):
        return 3

    def point(self):
        return (self.x, self.y)

    def forward(self):
        return (math.cos(self.yaw), math.sin(self.yaw))

    def right(self):
        return (math.sin(self.yaw), -math.cos(self.yaw))

    def shifted(self, along=0.0, lateral=0.0):
        """沿车头方向、车身右侧方向各偏移若干米后的新位姿。"""
        fwd = self.forward()
        right = self.right()
        return Pose(
            self.x + fwd[0] * along + right[0] * lateral,
            self.y + fwd[1] * along + right[1] * lateral,
            self.yaw,
        )

    def __repr__(self):
        return "Pose(x=%.3f, y=%.3f, yaw=%.3f)" % (self.x, self.y, self.yaw)


class Vehicle(object):
    """车辆几何参数。"""

    __slots__ = ("wheelbase", "length", "width", "max_steer")

    def __init__(self, wheelbase=DEFAULT_WHEELBASE, length=DEFAULT_VEHICLE_LENGTH,
                 width=DEFAULT_VEHICLE_WIDTH, max_steer=DEFAULT_MAX_STEER):
        self.wheelbase = _positive("轴距", wheelbase)
        self.length = _positive("车长", length)
        self.width = _positive("车宽", width)
        self.max_steer = _positive("最大转角", max_steer)
        if self.max_steer >= math.pi / 2.0:
            raise ParkingError("最大转角必须小于 pi/2，收到 %r" % (max_steer,))

    @property
    def min_turn_radius(self):
        """按最大转角算出的最小转弯半径（后轴中心轨迹半径）。"""
        return self.wheelbase / math.tan(self.max_steer)

    @property
    def max_curvature(self):
        return 1.0 / self.min_turn_radius

    def corners(self, pose):
        """车身矩形四角（世界坐标），顺序：右后、左后、左前、右前。"""
        half = self.width / 2.0
        local = ((half, 0.0), (-half, 0.0), (-half, self.length), (half, self.length))
        fwd = pose.forward()
        right = pose.right()
        out = []
        for lateral, along in local:
            out.append((
                pose.x + fwd[0] * along + right[0] * lateral,
                pose.y + fwd[1] * along + right[1] * lateral,
            ))
        return out

    def __repr__(self):
        return "Vehicle(length=%.2f, width=%.2f, wheelbase=%.2f, R_min=%.2f)" % (
            self.length, self.width, self.wheelbase, self.min_turn_radius)


class Slot(object):
    """一个停车位：中心位姿 + 尺寸。yaw 表示停放时车头朝向。"""

    __slots__ = ("kind", "pose", "width", "length")

    def __init__(self, pose=None, width=DEFAULT_SLOT_WIDTH,
                 length=DEFAULT_SLOT_LENGTH, kind=DEFAULT_SLOT_KIND):
        self.pose = pose if pose is not None else Pose(0.0, 0.0, DEFAULT_SLOT_YAW)
        self.width = _positive("车位宽", width)
        self.length = _positive("车位长", length)
        if kind not in SLOT_KINDS:
            raise ParkingError("车位类型只能是 %s，收到 %r"
                               % (", ".join(SLOT_KINDS), kind))
        self.kind = kind

    def corners(self):
        """车位矩形四角。"""
        half_w = self.width / 2.0
        half_l = self.length / 2.0
        local = ((half_w, -half_l), (-half_w, -half_l),
                 (-half_w, half_l), (half_w, half_l))
        fwd = self.pose.forward()
        right = self.pose.right()
        out = []
        for lateral, along in local:
            out.append((
                self.pose.x + fwd[0] * along + right[0] * lateral,
                self.pose.y + fwd[1] * along + right[1] * lateral,
            ))
        return out

    def target_pose(self, vehicle):
        """车位正中的停车位姿（后轴中心 + 车头朝车位朝向）。"""
        return self.pose.shifted(along=-vehicle.length / 2.0)

    def neighbor_pose(self, vehicle, index):
        """相邻车位里那辆车的位姿，index = -1 左侧、+1 右侧。"""
        return self.target_pose(vehicle).shifted(lateral=self.width * index)


# ---------------------------------------------------------------- 几何工具


def _straight(start, direction, length, step, gear):
    """沿 direction 走 length 米，返回 [(里程, Pose, gear)]。"""
    count = max(1, int(round(length / step)))
    out = []
    for i in range(count + 1):
        travelled = length * i / count
        out.append((
            travelled,
            Pose(start.x + direction[0] * travelled,
                 start.y + direction[1] * travelled,
                 start.yaw),
            gear,
        ))
    return out


def _arc(center, radius, start_angle, sweep, step, gear,
         heading_start, heading_end):
    """绕 center 从 start_angle 转 sweep 弧度，返回 [(里程, Pose, gear)]。

    圆弧上航向是匀速变化的，所以直接按比例在 ``heading_start`` 与
    ``heading_end`` 之间线性插值，既精确又不用再纠结极角与航向的符号关系。
    """
    count = max(1, int(round(abs(sweep) * radius / step)))
    delta = normalize_angle(heading_end - heading_start)
    out = []
    for i in range(count + 1):
        ratio = i / count
        alpha = start_angle + sweep * ratio
        out.append((
            abs(sweep) * radius * ratio,
            Pose(center[0] + radius * math.cos(alpha),
                 center[1] + radius * math.sin(alpha),
                 normalize_angle(heading_start + delta * ratio)),
            gear,
        ))
    return out


def build_plan(vehicle=None, slot=None, turn_radius=DEFAULT_TURN_RADIUS,
               entry_length=DEFAULT_ENTRY_LENGTH, step=DEFAULT_STEP):
    """构造倒车入库路径。

    几何做法（全程倒车，gear 恒为 -1）：

    1. 目标位姿 T = 车位正中，车头朝向车道（倒车入库后车头朝外）；
    2. 倒车行驶方向 ``travel`` 与车头相反，J = T − travel·entry，即车头
       已经对准车位、只差最后一小段直线的那个点；
    3. 圆心 C 在 J 的行进方向右侧 R 处，起点 S = C − R·travel；
    4. 圆弧从 S 右转 90° 到 J，再从 J 沿 travel 倒车 entry 米到 T。

    因此路径长度恒为 ``R·pi/2 + entry``，转过的角度恒为 90°。

    返回值是 dict，包含 samples / poses / length / sweep 等。
    """
    vehicle = vehicle or Vehicle()
    slot = slot or Slot()
    radius = _positive("转弯半径", turn_radius)
    entry = _positive("直线段长度", entry_length)
    stride = _positive("采样步长", step)

    if radius < vehicle.min_turn_radius - 1e-6:
        raise ParkingError(
            "转弯半径 %.3f m 小于车辆最小转弯半径 %.3f m（最大转角 %.3f rad）"
            % (radius, vehicle.min_turn_radius, vehicle.max_steer))

    target = slot.target_pose(vehicle)
    fwd = target.forward()
    # 全程倒车：行驶方向与车头相反
    travel = (-fwd[0], -fwd[1])
    # 圆弧终点（J）落在车位正前方 entry 米处，此时车头已对准车位
    junction = Pose(target.x - travel[0] * entry, target.y - travel[1] * entry, target.yaw)

    # 圆心在 J 的行进方向右侧 R 处；起点 S 则是圆心沿行进方向反推 R
    side = (travel[1], -travel[0])
    center = (junction.x + side[0] * radius, junction.y + side[1] * radius)
    start = Pose(center[0] - travel[0] * radius, center[1] - travel[1] * radius, 0.0)

    # 圆弧从 S 转到 J，极角从 start_angle 转过 -90 度（右转）
    heading_end = target.yaw
    heading_start = normalize_angle(math.atan2(-side[1], side[0]) + math.pi)
    start_angle = math.atan2(start.y - center[1], start.x - center[0])
    arc = _arc(center, radius, start_angle, -math.pi / 2.0, stride,
               GEAR_REVERSE, heading_start, heading_end)
    start.yaw = arc[0][1].yaw

    straight = _straight(junction, travel, entry, stride, GEAR_REVERSE)

    ordered = []
    for _s, pose, gear in arc:
        ordered.append((pose, gear))
    for _s, pose, gear in straight[1:]:
        ordered.append((pose, gear))

    start = ordered[0][0]
    samples = []
    travelled = 0.0
    previous = start
    for pose, gear in ordered:
        travelled += distance(previous.point(), pose.point())
        samples.append((travelled, pose, gear))
        previous = pose

    # 折线累计长度会略小于真实弧长，所以 length 用解析值，另记 sampled_length
    analytic = abs(math.pi / 2.0) * radius + entry

    return {
        "samples": samples,
        "poses": [item[1] for item in samples],
        "length": analytic,
        "sampled_length": travelled,
        "sweep": math.pi / 2.0,
        "radius": radius,
        "entry": entry,
        "start": start,
        "junction": junction,
        "target": target,
        "neighbors": [slot.neighbor_pose(vehicle, -1), slot.neighbor_pose(vehicle, 1)],
        "slot": slot,
        "vehicle": vehicle,
    }


# ---------------------------------------------------------------- 碰撞检测


def _project(points, axis):
    values = [p[0] * axis[0] + p[1] * axis[1] for p in points]
    return min(values), max(values)


def polygon_gap(points_a, points_b):
    """两个凸多边形之间的间隔距离；相交时返回 0。"""
    widest = None
    for points in (points_a, points_b):
        count = len(points)
        for i in range(count):
            ax, ay = points[i]
            bx, by = points[(i + 1) % count]
            axis = (ay - by, bx - ax)
            norm = math.hypot(axis[0], axis[1])
            if norm < _EPS:
                continue
            axis = (axis[0] / norm, axis[1] / norm)
            min_a, max_a = _project(points_a, axis)
            min_b, max_b = _project(points_b, axis)
            gap = max(min_a - max_b, min_b - max_a)
            if widest is None or gap > widest:
                widest = gap
    if widest is None:
        return 0.0
    return max(0.0, widest)


def min_clearance(plan):
    """整条路径上，车身与左右邻车的最小间距（米）。"""
    vehicle = plan["vehicle"]
    neighbors = plan["neighbors"]
    best = None
    for _s, pose, _gear in plan["samples"]:
        body = vehicle.corners(pose)
        for neighbor in neighbors:
            gap = polygon_gap(body, vehicle.corners(neighbor))
            if best is None or gap < best:
                best = gap
    return 0.0 if best is None else best


# ---------------------------------------------------------------- 断言与报告


def evaluate_plan(plan, tolerance=DEFAULT_TOLERANCE, clearance=DEFAULT_CLEARANCE,
                  max_path=DEFAULT_MAX_PATH):
    """跑完规划后逐条判定，返回 [dict(type, passed, detail)]。"""
    vehicle = plan["vehicle"]
    results = []

    error = distance(plan["poses"][-1].point(), plan["target"].point())
    results.append({
        "type": "终点精度",
        "passed": error <= tolerance,
        "detail": "偏差 %.3f m，容差 %.2f m" % (error, tolerance),
    })

    gap = min_clearance(plan)
    results.append({
        "type": "无碰撞",
        "passed": gap > clearance,
        "detail": "与邻车最小间距 %.3f m，下限 %.2f m" % (gap, clearance),
    })

    curvature = 1.0 / plan["radius"]
    limit = vehicle.max_curvature
    results.append({
        "type": "曲率上限",
        "passed": curvature <= limit + 1e-9,
        "detail": "曲率 %.3f 1/m，车辆上限 %.3f 1/m" % (curvature, limit),
    })

    results.append({
        "type": "路径长度",
        "passed": plan["length"] <= max_path + 1e-9,
        "detail": "倒车 %.2f m，上限 %.2f m" % (plan["length"], max_path),
    })
    return results


def all_passed(results):
    return all(item["passed"] for item in results)


def _inside(points, point):
    """凸多边形内点判定（用叉积同号）。"""
    sign = 0
    for i in range(len(points)):
        ax, ay = points[i]
        bx, by = points[(i + 1) % len(points)]
        cross = (bx - ax) * (point[1] - ay) - (by - ay) * (point[0] - ax)
        if abs(cross) < _EPS:
            continue
        current = 1 if cross > 0 else -1
        if sign == 0:
            sign = current
        elif current != sign:
            return False
    return True


def render_map(plan, width=DEFAULT_GRID_WIDTH, height=DEFAULT_GRID_HEIGHT):
    """把车位、邻车、路径画成字符俯视图。"""
    vehicle = plan["vehicle"]
    slot = plan["slot"]

    xs = [pose.x for pose in plan["poses"]]
    ys = [pose.y for pose in plan["poses"]]
    for corner in slot.corners() + vehicle.corners(plan["target"]):
        xs.append(corner[0])
        ys.append(corner[1])
    for neighbor in plan["neighbors"]:
        for corner in vehicle.corners(neighbor):
            xs.append(corner[0])
            ys.append(corner[1])

    pad = 0.8
    min_x, max_x = min(xs) - pad, max(xs) + pad
    min_y, max_y = min(ys) - pad, max(ys) + pad
    span_x = max(max_x - min_x, 1e-6)
    span_y = max(max_y - min_y, 1e-6)

    grid = [[" "] * width for _ in range(height)]

    def cell(x, y):
        col = int((x - min_x) / span_x * (width - 1))
        row = int((max_y - y) / span_y * (height - 1))
        return max(0, min(height - 1, row)), max(0, min(width - 1, col))

    def put(x, y, char):
        row, col = cell(x, y)
        grid[row][col] = char

    def draw_rect(points, char):
        for i in range(len(points)):
            ax, ay = points[i]
            bx, by = points[(i + 1) % len(points)]
            steps = max(2, int(max(abs(bx - ax), abs(by - ay)) / 0.05) + 1)
            for k in range(steps + 1):
                put(ax + (bx - ax) * k / steps, ay + (by - ay) * k / steps, char)

    draw_rect(slot.corners(), "-")

    for neighbor in plan["neighbors"]:
        corners = vehicle.corners(neighbor)
        for row in range(height):
            for col in range(width):
                x = min_x + (col + 0.5) / width * span_x
                y = max_y - (row + 0.5) / height * span_y
                if _inside(corners, (x, y)):
                    grid[row][col] = "#"

    for pose in plan["poses"]:
        put(pose.x, pose.y, "o")

    draw_rect(vehicle.corners(plan["poses"][-1]), "@")

    return ["".join(row) for row in grid]


def format_report(plan, results, header=""):
    """把规划结果拼成一段可以直接打印的报告。"""
    vehicle = plan["vehicle"]
    slot = plan["slot"]
    lines = []
    if header:
        lines.append(header)
    lines.append("车辆：长 %.2f m，宽 %.2f m，轴距 %.2f m，最小转弯半径 %.2f m"
                 % (vehicle.length, vehicle.width, vehicle.wheelbase,
                    vehicle.min_turn_radius))
    lines.append("车位：%s %.2f m x %.2f m（宽 x 长），转弯半径 %.2f m，直线段 %.2f m"
                 % ("垂直车位" if slot.kind == "perpendicular" else "平行车位",
                    slot.width, slot.length, plan["radius"], plan["entry"]))
    lines.append("起点：%s" % (plan["start"],))
    lines.append("终点：%s" % (plan["poses"][-1],))
    lines.append("路径：倒车 %.2f m，转角 %.1f 度，采样 %d 个"
                 % (plan["length"], math.degrees(plan["sweep"]),
                    len(plan["samples"])))
    lines.append("最小间距：%.3f m" % (min_clearance(plan),))
    passed = sum(1 for item in results if item["passed"])
    lines.append("断言：%d 通过 / %d 失败（共 %d 条）"
                 % (passed, len(results) - passed, len(results)))
    for item in results:
        lines.append("  [%s] %s  %s"
                     % ("PASS" if item["passed"] else "FAIL",
                        item["type"], item["detail"]))
    lines.append("结论：%s" % ("PASSED" if all_passed(results) else "FAILED"))
    lines.append("")
    lines.append("俯视图（- 车位线  # 邻车  o 路径  @ 车身）：")
    lines.extend(render_map(plan))
    return "\n".join(lines)


# ---------------------------------------------------------------- 参数读取


def _as_float(get_param, name, default):
    """从 ROS 参数服务器读一个浮点数，读不到就用默认值。"""
    try:
        value = get_param(name, default)
    except Exception:
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _as_text(get_param, name, default):
    try:
        value = get_param(name, default)
    except Exception:
        return default
    if value is None:
        return default
    return "%s" % (value,)


def to_message_text(text):
    """把报告转成 ``std_msgs/String`` 能用的载荷。

    Python 3 里 rospy 可以直接收 str，Python 2 里则必须是 byte str，
    所以这里在 py2 下显式编码成 utf-8。
    """
    if isinstance(text, bytes):
        payload = text
    else:
        payload = u"%s" % (text,)
        if sys.version_info[0] < 3:
            payload = payload.encode("utf-8")
    return payload


_PARAM_DEFAULTS = {
    "slot_kind": DEFAULT_SLOT_KIND,
    "slot_width": DEFAULT_SLOT_WIDTH,
    "slot_length": DEFAULT_SLOT_LENGTH,
    "slot_x": 0.0,
    "slot_y": 0.0,
    "slot_yaw": DEFAULT_SLOT_YAW,
    "wheelbase": DEFAULT_WHEELBASE,
    "length": DEFAULT_VEHICLE_LENGTH,
    "width": DEFAULT_VEHICLE_WIDTH,
    "max_steer": DEFAULT_MAX_STEER,
    "turn_radius": DEFAULT_TURN_RADIUS,
    "entry_length": DEFAULT_ENTRY_LENGTH,
    "step": DEFAULT_STEP,
    "tolerance": DEFAULT_TOLERANCE,
    "clearance": DEFAULT_CLEARANCE,
    "max_path": DEFAULT_MAX_PATH,
    "publish_hz": DEFAULT_PUBLISH_HZ,
    "plan_topic": "/parking/plan",
    "report_topic": "/parking/report",
    "point_topic": "/parking/plan_point",
    "frame_id": "map",
}


class PlannerParams(object):
    """节点启动时从参数服务器读进来的一组参数。"""

    __slots__ = (
        "slot_kind", "slot_width", "slot_length", "slot_x", "slot_y", "slot_yaw",
        "wheelbase", "length", "width", "max_steer",
        "turn_radius", "entry_length", "step",
        "tolerance", "clearance", "max_path",
        "publish_hz", "plan_topic", "report_topic", "point_topic", "frame_id",
    )

    def __init__(self, **kwargs):
        for name in self.__slots__:
            setattr(self, name, kwargs.get(name, _PARAM_DEFAULTS[name]))

    @classmethod
    def from_params(cls, get_param):
        """按 launch / yaml 里的键名读参数，缺项回落到默认值。"""
        return cls(
            slot_kind=_as_text(get_param, "slot/kind", DEFAULT_SLOT_KIND),
            slot_width=_as_float(get_param, "slot/width", DEFAULT_SLOT_WIDTH),
            slot_length=_as_float(get_param, "slot/length", DEFAULT_SLOT_LENGTH),
            slot_x=_as_float(get_param, "slot/x", 0.0),
            slot_y=_as_float(get_param, "slot/y", 0.0),
            slot_yaw=_as_float(get_param, "slot/yaw", DEFAULT_SLOT_YAW),
            wheelbase=_as_float(get_param, "vehicle/wheelbase", DEFAULT_WHEELBASE),
            length=_as_float(get_param, "vehicle/length", DEFAULT_VEHICLE_LENGTH),
            width=_as_float(get_param, "vehicle/width", DEFAULT_VEHICLE_WIDTH),
            max_steer=_as_float(get_param, "vehicle/max_steer", DEFAULT_MAX_STEER),
            turn_radius=_as_float(get_param, "plan/turn_radius", DEFAULT_TURN_RADIUS),
            entry_length=_as_float(get_param, "plan/entry_length", DEFAULT_ENTRY_LENGTH),
            step=_as_float(get_param, "plan/step", DEFAULT_STEP),
            tolerance=_as_float(get_param, "check/tolerance", DEFAULT_TOLERANCE),
            clearance=_as_float(get_param, "check/clearance", DEFAULT_CLEARANCE),
            max_path=_as_float(get_param, "check/max_path", DEFAULT_MAX_PATH),
            publish_hz=_as_float(get_param, "rate/publish_hz", DEFAULT_PUBLISH_HZ),
            plan_topic=_as_text(get_param, "topic/plan", "/parking/plan"),
            report_topic=_as_text(get_param, "topic/report", "/parking/report"),
            point_topic=_as_text(get_param, "topic/point", "/parking/plan_point"),
            frame_id=_as_text(get_param, "frame/id", "map"),
        )

    def make_vehicle(self):
        return Vehicle(self.wheelbase, self.length, self.width, self.max_steer)

    def make_slot(self):
        return Slot(Pose(self.slot_x, self.slot_y, self.slot_yaw),
                    self.slot_width, self.slot_length, self.slot_kind)

    def __repr__(self):
        return "PlannerParams(slot=%.2fx%.2f, R=%.2f, entry=%.2f, hz=%.1f)" % (
            self.slot_width, self.slot_length, self.turn_radius,
            self.entry_length, self.publish_hz)
