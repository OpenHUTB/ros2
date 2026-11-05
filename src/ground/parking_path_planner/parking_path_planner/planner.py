"""Hybrid A* 泊车路径规划器（纯标准库实现，无第三方依赖）。

坐标约定
--------
* 世界坐标系：车位由 :class:`SlotParams` 的中心点与朝向描述。
* 车位坐标系：原点在车位中心，``+Y`` 指向车位内部，入口位于 ``Y = -depth/2``，
  车道为 ``Y < -depth/2``。车辆从车道倒车进入车位。
* 车辆状态为 ``(x, y, yaw)``，参考点为车辆几何中心（低速运动学简化）。

碰撞模型
--------
车位两侧视为已停放车辆，车位后方为墙。因此 ``Y >= -depth/2`` 的区域只有
``|X| <= width/2`` 可通行；``Y > depth/2`` 为后墙。该假设可通过
``PlannerConfig.neighbors_parked`` 关闭。
"""

import heapq
import math
from dataclasses import dataclass, field

EPS = 1e-6


# --------------------------------------------------------------------------- #
# 参数
# --------------------------------------------------------------------------- #
@dataclass
class VehicleParams:
    """车辆几何与转向能力（默认值参考阿克曼小车）。"""

    wheelbase: float = 2.70
    width: float = 1.80
    length: float = 4.50
    max_steer_deg: float = 35.0

    @property
    def max_steer(self) -> float:
        return math.radians(self.max_steer_deg)

    @property
    def min_turn_radius(self) -> float:
        return self.wheelbase / math.tan(self.max_steer)


@dataclass
class SlotParams:
    """垂直车位参数。``heading_deg`` 为车位坐标系相对世界坐标系的偏航角。"""

    width: float = 2.70
    depth: float = 5.50
    center_x: float = 0.0
    center_y: float = 2.75
    heading_deg: float = 0.0

    @property
    def heading(self) -> float:
        return math.radians(self.heading_deg)


@dataclass
class PlannerConfig:
    """搜索参数。默认值已在本仓库文档给出的场景上验证可用。"""

    step: float = 0.40              # 单次扩展弧长（m）
    xy_resolution: float = 0.25     # 位置离散分辨率（m）
    heading_resolution_deg: float = 5.0
    goal_xy_tolerance: float = 0.35
    goal_heading_tolerance_deg: float = 10.0
    gear_penalty: float = 1.2       # 换挡惩罚，抑制无意义的前后挪动
    steer_penalty: float = 0.05     # 满舵惩罚
    heuristic_weight: float = 1.2
    max_iterations: int = 40000
    neighbors_parked: bool = True
    steer_levels: tuple = (0.0, 0.5, -0.5, 1.0, -1.0)


@dataclass
class PlanResult:
    """规划结果。``poses`` 为 ``[(x, y, yaw, gear)]``，``gear`` 为 ``1`` 前进 / ``-1`` 倒车。"""

    success: bool
    poses: list = field(default_factory=list)
    cost: float = 0.0
    iterations: int = 0
    message: str = ""


# --------------------------------------------------------------------------- #
# 几何工具
# --------------------------------------------------------------------------- #
def wrap_angle(angle: float) -> float:
    """把角度归一化到 ``[-pi, pi]``。"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def clip_halfplane(polygon, a, b, c):
    """Sutherland-Hodgman：保留满足 ``a*x + b*y + c >= 0`` 的部分。"""
    if not polygon:
        return []
    out = []
    n = len(polygon)
    for i in range(n):
        p = polygon[i]
        q = polygon[(i + 1) % n]
        fp = a * p[0] + b * p[1] + c
        fq = a * q[0] + b * q[1] + c
        if fp >= 0.0:
            out.append(p)
        if (fp > 0.0 > fq) or (fp < 0.0 < fq):
            t = fp / (fp - fq)
            out.append((p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1])))
    return out


def polygon_area(polygon) -> float:
    """多边形面积（鞋带公式）。"""
    if len(polygon) < 3:
        return 0.0
    total = 0.0
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        total += x1 * y2 - x2 * y1
    return abs(total) / 2.0


def world_to_slot(x: float, y: float, slot: SlotParams):
    """世界坐标 -> 车位坐标系。"""
    dx, dy = x - slot.center_x, y - slot.center_y
    c, s = math.cos(-slot.heading), math.sin(-slot.heading)
    return dx * c - dy * s, dx * s + dy * c


def slot_to_world(x: float, y: float, slot: SlotParams):
    """车位坐标系 -> 世界坐标。"""
    c, s = math.cos(slot.heading), math.sin(slot.heading)
    return slot.center_x + x * c - y * s, slot.center_y + x * s + y * c


def car_polygon(x: float, y: float, yaw: float, vehicle: VehicleParams):
    """车辆矩形轮廓（世界坐标）。"""
    c, s = math.cos(yaw), math.sin(yaw)
    half_l, half_w = vehicle.length / 2.0, vehicle.width / 2.0
    corners = ((-half_l, -half_w), (half_l, -half_w),
               (half_l, half_w), (-half_l, half_w))
    return [(x + px * c - py * s, y + px * s + py * c) for px, py in corners]


def is_collision(x: float, y: float, yaw: float, vehicle: VehicleParams,
                 slot: SlotParams, neighbors_parked: bool = True) -> bool:
    """车辆矩形是否侵入不可行驶区。"""
    # 把车体轮廓转到车位坐标系，碰撞判断就退化为轴对齐的情形
    world_poly = car_polygon(x, y, yaw, vehicle)
    poly = [world_to_slot(px, py, slot) for px, py in world_poly]

    in_slot = clip_halfplane(poly, 0.0, 1.0, slot.depth / 2.0)
    if polygon_area(in_slot) > EPS:
        # 入口以内的部分必须落在车位宽度内
        if neighbors_parked:
            if polygon_area(clip_halfplane(in_slot, 1.0, 0.0, -slot.width / 2.0)) > EPS:
                return True
            if polygon_area(clip_halfplane(in_slot, -1.0, 0.0, -slot.width / 2.0)) > EPS:
                return True
        # 车位后墙
        if polygon_area(clip_halfplane(poly, 0.0, 1.0, -slot.depth / 2.0)) > EPS:
            return True
    return False


def _pose_after(x, y, yaw, steer_ratio, gear, ds, vehicle):
    """按恒定曲率推进一步（用中点朝向积分以减小离散误差）。"""
    yaw_next = yaw + gear * ds * math.tan(steer_ratio * vehicle.max_steer) / vehicle.wheelbase
    mid = yaw + 0.5 * (yaw_next - yaw)
    return (x + gear * ds * math.cos(mid),
            y + gear * ds * math.sin(mid),
            wrap_angle(yaw_next))


# --------------------------------------------------------------------------- #
# 规划器
# --------------------------------------------------------------------------- #
def plan_parking(start, goal, vehicle=None, slot=None, config=None) -> PlanResult:
    """在车位周围搜索一条可行的泊车轨迹。

    :param start: 起始位姿 ``(x, y, yaw)``，世界坐标
    :param goal: 目标位姿 ``(x, y, yaw)``，世界坐标（通常贴车位中心）
    """
    vehicle = vehicle or VehicleParams()
    slot = slot or SlotParams()
    config = config or PlannerConfig()

    sx, sy, syaw = start
    gx, gy, gyaw = goal
    heading_res = math.radians(config.heading_resolution_deg)
    goal_th_tol = math.radians(config.goal_heading_tolerance_deg)

    def heuristic(x, y, yaw):
        return (math.hypot(x - gx, y - gy)
                + 0.6 * abs(wrap_angle(yaw - gyaw)))

    def state_key(x, y, yaw):
        return (int(round(x / config.xy_resolution)),
                int(round(y / config.xy_resolution)),
                int(round(wrap_angle(yaw) / heading_res)))

    if is_collision(sx, sy, syaw, vehicle, slot, config.neighbors_parked):
        return PlanResult(False, message="起始位姿位于不可行驶区域")
    if is_collision(gx, gy, gyaw, vehicle, slot, config.neighbors_parked):
        return PlanResult(False, message="目标位姿位于不可行驶区域")

    start_node = (sx, sy, syaw, 0.0, 1, None)
    open_heap = [(config.heuristic_weight * heuristic(sx, sy, syaw), 0.0, start_node)]
    best_cost = {state_key(sx, sy, syaw): 0.0}
    closed = set()
    goal_node = None
    iterations = 0

    while open_heap and iterations < config.max_iterations:
        iterations += 1
        _, cost, node = heapq.heappop(open_heap)
        x, y, yaw, _, gear, _parent = node
        key = state_key(x, y, yaw)
        if key in closed:
            continue
        closed.add(key)

        if (math.hypot(x - gx, y - gy) < config.goal_xy_tolerance
                and abs(wrap_angle(yaw - gyaw)) < goal_th_tol):
            goal_node = node
            break

        for next_gear in (1, -1):
            for steer in config.steer_levels:
                nx, ny, nyaw = _pose_after(x, y, yaw, steer, next_gear,
                                           config.step, vehicle)
                if is_collision(nx, ny, nyaw, vehicle, slot, config.neighbors_parked):
                    continue
                nkey = state_key(nx, ny, nyaw)
                if nkey in closed:
                    continue
                penalty = 0.0
                if next_gear != gear:
                    penalty += config.gear_penalty
                if abs(steer) > 0.9:
                    penalty += config.steer_penalty
                new_cost = cost + config.step + penalty
                if new_cost < best_cost.get(nkey, float("inf")):
                    best_cost[nkey] = new_cost
                    heapq.heappush(
                        open_heap,
                        (new_cost + config.heuristic_weight * heuristic(nx, ny, nyaw),
                         new_cost,
                         (nx, ny, nyaw, new_cost, next_gear, node)))
    else:
        if goal_node is None:
            return PlanResult(False, iterations=iterations, message="未找到可行泊车轨迹")

    poses = []
    node = goal_node
    while node is not None:
        x, y, yaw, _, gear, parent = node
        poses.append((x, y, yaw, gear))
        node = parent
    poses.reverse()

    return PlanResult(True, poses, cost=poses_cost(poses), iterations=iterations,
                      message="规划成功")


def poses_cost(poses) -> float:
    """轨迹总里程。"""
    return sum(math.hypot(poses[i][0] - poses[i - 1][0],
                          poses[i][1] - poses[i - 1][1])
               for i in range(1, len(poses)))


def densify(poses, step: float = 0.15):
    """把稀疏的搜索结果插值为连续轨迹点 ``[(x, y, yaw, gear)]``，便于下游控制器跟踪。"""
    if not poses:
        return []
    out = []
    for i in range(len(poses) - 1):
        x1, y1, yaw1, gear = poses[i]
        x2, y2, yaw2 = poses[i + 1][0], poses[i + 1][1], poses[i + 1][2]
        delta_yaw = wrap_angle(yaw2 - yaw1)
        distance = math.hypot(x2 - x1, y2 - y1)
        segments = max(1, int(distance / step))
        for j in range(segments):
            ratio = j / segments
            out.append((x1 + ratio * (x2 - x1),
                        y1 + ratio * (y2 - y1),
                        wrap_angle(yaw1 + ratio * delta_yaw),
                        gear))
    out.append((poses[-1][0], poses[-1][1], poses[-1][2], poses[-1][3]))
    return out


def count_gear_shifts(poses) -> int:
    """换挡次数。"""
    shifts = 0
    for i in range(1, len(poses)):
        if poses[i][3] != poses[i - 1][3]:
            shifts += 1
    return shifts


def max_curvature(poses) -> float:
    """轨迹上的最大曲率 ``kappa = d(yaw)/ds``，用于验证非完整约束是否满足。

    需传入带朝向的搜索节点 ``(x, y, yaw, gear)``；插值后的稠密点不含朝向，
    用折线做圆拟合会把节点拐点误判成极大曲率，因此这里按航向变化率计算。
    """
    peak = 0.0
    for i in range(1, len(poses)):
        x1, y1, yaw1 = poses[i - 1][0], poses[i - 1][1], poses[i - 1][2]
        x2, y2, yaw2 = poses[i][0], poses[i][1], poses[i][2]
        ds = math.hypot(x2 - x1, y2 - y1)
        if ds < EPS:
            continue
        peak = max(peak, abs(wrap_angle(yaw2 - yaw1)) / ds)
    return peak


def goal_pose_from_slot(slot: SlotParams, vehicle: VehicleParams = None):
    """由车位参数推导目标位姿：车心在车位中心，车头朝入口方向（即倒车入库后的姿态）。"""
    x, y = slot_to_world(0.0, 0.0, slot)
    yaw = wrap_angle(slot.heading - math.pi / 2.0)
    return x, y, yaw


def slot_corners_world(slot: SlotParams):
    """车位四角（世界坐标），供 RViz 绘制车位框。"""
    half_w, half_d = slot.width / 2.0, slot.depth / 2.0
    local = [(-half_w, -half_d), (half_w, -half_d),
             (half_w, half_d), (-half_w, half_d)]
    return [slot_to_world(px, py, slot) for px, py in local]
