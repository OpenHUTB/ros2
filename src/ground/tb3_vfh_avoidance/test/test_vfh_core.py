"""vfh_core / sim_world 单元测试。

所有用例只用纯 Python 构造扫描数据（列表），不依赖 ROS 消息，
可在任何装有 Python 的机器上运行：``pytest test``。
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tb3_vfh_avoidance.sim_world import (DiffDriveRobot, World,
                                         normalize_angle)
from tb3_vfh_avoidance.vfh_core import (VfhPlanner, normalize_angle as na,
                                        polar_points)

FAR = 3.5


def make_scan(front=None, left=None, right=None, n=360, angle_min=0.0):
    """构造一帧 TB3 风格扫描：ranges[0] 是正前方，逆时针为正。"""
    ranges = [FAR] * n
    inc = 2.0 * math.pi / n

    def fill(center_deg, half_deg, value):
        center = math.radians(center_deg)
        half = math.radians(half_deg)
        for i in range(n):
            angle = angle_min + i * inc
            if abs(na(angle - center)) <= half:
                ranges[i] = value

    if front is not None:
        fill(0, 15, front)
    if left is not None:
        fill(60, 20, left)
    if right is not None:
        fill(-60, 20, right)
    return ranges, angle_min, inc


# ----------------------------------------------------------------------
# 角度与极坐标工具
# ----------------------------------------------------------------------
def test_normalize_angle_range():
    assert abs(na(math.pi + 0.1) + math.pi - 0.1) < 1e-9
    assert abs(na(0.0)) < 1e-12
    assert abs(na(-math.pi / 2) + math.pi / 2) < 1e-12
    assert -math.pi <= na(10.0 * math.pi) < math.pi


def test_polar_points_filters_invalid():
    ranges = [0.5, float('nan'), float('inf'), 0.0, 4.0, 1.2]
    pts = polar_points(ranges, angle_min=0.0, angle_increment=0.1,
                       range_min=0.1, range_max=3.5)
    # 只保留 0.5 和 1.2 两个有效回波
    assert [round(a, 2) for a, _ in pts] == [0.0, 0.5]
    assert [d for _, d in pts] == [0.5, 1.2]


# ----------------------------------------------------------------------
# TB3 雷达约定（angle_min=0，ranges[0] 是正前方）
# ----------------------------------------------------------------------
def test_forward_obstacle_in_sector_zero():
    """正前方 0.5 m 的障碍必须落在扇区 0，而不是数组中间。"""
    planner = VfhPlanner()
    ranges, angle_min, inc = make_scan(front=0.5)
    hist, sector_min = planner.build_histogram(
        polar_points(ranges, angle_min, inc))
    assert sector_min[0] == 0.5
    assert sector_min[planner.n_sectors // 2] == FAR   # 数组中间是正后方


def test_angle_min_offset_respected():
    """angle_min = -pi 时，同一帧数据的角度归属必须整体平移。"""
    planner = VfhPlanner()
    n = 360
    inc = 2.0 * math.pi / n
    ranges = [FAR] * n
    ranges[0] = 0.6                     # angle_min=-pi 时这是正后方
    ranges[n // 2] = 0.6                # 这是正前方
    hist, sector_min = planner.build_histogram(
        polar_points(ranges, -math.pi, inc))
    assert sector_min[0] == 0.6
    assert sector_min[planner.n_sectors // 2] == 0.6


# ----------------------------------------------------------------------
# 决策行为
# ----------------------------------------------------------------------
def test_open_space_cruise_straight():
    planner = VfhPlanner()
    cmd = planner.compute(*make_scan())
    assert cmd.state == 'cruise'
    assert abs(cmd.heading) < math.radians(5)
    assert cmd.linear > 0.15
    assert abs(cmd.angular) < 0.1


def test_wall_ahead_forces_turn():
    planner = VfhPlanner()
    cmd = planner.compute(*make_scan(front=0.45))
    assert abs(cmd.heading) > math.radians(20)
    assert abs(cmd.angular) > 0.3


def test_full_block_enters_recovery():
    """四面都是 0.45 m 的墙：先报 blocked，随后进入 recovery 原地旋转。"""
    planner = VfhPlanner()
    ranges, angle_min, inc = make_scan()
    ranges = [0.45] * len(ranges)
    states = [planner.compute(ranges, angle_min, inc).state
              for _ in range(12)]
    assert states[0] == 'blocked'
    assert 'recovery' in states
    assert all(s == 'recovery' for s in states[1:])


def test_freer_side_is_chosen():
    """右侧被堵死、左侧开阔时应向左转（正角速度）。"""
    planner = VfhPlanner()
    cmd = planner.compute(*make_scan(front=0.5, right=0.5, left=2.5))
    assert cmd.heading > math.radians(20)


def test_speed_reduced_near_obstacle():
    # 规划器带航向滞后状态，两个场景必须各用独立实例才可比
    open_cmd = VfhPlanner().compute(*make_scan())
    near_cmd = VfhPlanner().compute(*make_scan(front=1.6))
    # 全向 0.22 m/s 直行；前方 1.6 m 有墙时必须减速并偏向谷值方向
    assert near_cmd.linear < open_cmd.linear
    assert abs(near_cmd.heading) > math.radians(10)


def test_velocity_limits():
    planner = VfhPlanner()
    cmd = planner.compute(*make_scan())
    assert cmd.linear <= planner.max_linear + 1e-9
    assert abs(cmd.angular) <= planner.max_angular + 1e-9


def test_reset_clears_state():
    planner = VfhPlanner()
    planner.compute(*make_scan(front=0.3))
    planner.reset()
    assert planner._last_heading is None
    assert planner._recovery_left == 0
    assert planner._stuck_count == 0


# ----------------------------------------------------------------------
# 配置空间阻塞（车身半径参与判决）
# ----------------------------------------------------------------------
def test_lateral_obstacle_blocks_passing_direction():
    """右侧 0.3 m 的障碍应阻塞"贴着它开过去"的方向。"""
    planner = VfhPlanner()
    ranges, angle_min, inc = make_scan(front=2.0, right=0.30)
    hist, sector_min = planner.build_histogram(
        polar_points(ranges, angle_min, inc))
    blocked = planner.blocked_sectors(sector_min)
    right_sector = int((-math.radians(60)) / planner.sector_width) \
        % planner.n_sectors
    assert blocked[right_sector]


def test_distant_wall_does_not_block():
    """2.5 m 外的墙超出 block_range=2.0，不应阻塞任何方向。"""
    planner = VfhPlanner()
    ranges, angle_min, inc = make_scan(front=2.5)
    hist, sector_min = planner.build_histogram(
        polar_points(ranges, angle_min, inc))
    blocked = planner.blocked_sectors(sector_min)
    assert not any(blocked)


def test_body_radius_enters_blocking():
    """同一障碍：车身半径越大，阻塞的方向扇区越宽。"""
    ranges, angle_min, inc = make_scan(front=0.60)
    small = VfhPlanner(robot_radius=0.10, body_margin=0.02)
    large = VfhPlanner(robot_radius=0.28, body_margin=0.02)
    _, sector_min = small.build_histogram(
        polar_points(ranges, angle_min, inc))
    n_small = sum(small.blocked_sectors(sector_min))
    n_large = sum(large.blocked_sectors(sector_min))
    assert n_large > n_small


# ----------------------------------------------------------------------
# sim_world 几何
# ----------------------------------------------------------------------
def test_ray_cast_hits_wall():
    world = World()
    world.add_wall(0.0, 2.0, 4.0, 2.0)          # y=2 的墙
    assert abs(world.ray_cast(2.0, 0.0, math.pi / 2) - 2.0) < 1e-9
    assert world.ray_cast(2.0, 0.0, -math.pi / 2) == 3.5   # 背对墙：未命中
    assert world.ray_cast(0.0, 0.0, 0.0) == 3.5            # 与墙平行：未命中


def test_scan_front_index_zero():
    """机器人朝 +x 时，正前方墙的回波必须在 ranges[0]（TB3 约定）。"""
    world = World()
    world.add_wall(5.0, -1.0, 5.0, 1.0)         # x=5 的墙
    ranges, angle_min, inc = world.scan(2.0, 0.0, 0.0)
    assert angle_min == 0.0
    assert abs(ranges[0] - 3.0) < 1e-6          # 5 - 2 = 3 m
    assert ranges[180] == FAR                    # 正后方没有墙


def test_robot_step_moves_forward():
    robot = DiffDriveRobot(0.0, 0.0, 0.0)
    robot.step(0.2, 0.0, dt=0.5)
    assert abs(robot.x - 0.1) < 1e-9
    assert abs(robot.y) < 1e-9
    robot.step(0.2, math.pi, dt=0.5)            # 原地掉头
    assert abs(normalize_angle(robot.theta)) > 1.0
    assert len(robot.path) == 3                 # 记录了轨迹
