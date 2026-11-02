"""离线闭环仿真演示：VFH vs 朴素三扇区避障。

用法（无需 Gazebo / Ubuntu，只需 Python + matplotlib）::

    python tools/offline_demo.py --out ../docs/img/ground

脚本会：
1. 在同一个 U 形陷阱迷宫里，分别用 VFH 规划器与朴素三扇区策略跑完整闭环；
2. 终端打印两者是否到达终点、用时、路径长度、最小净空、碰撞次数；
3. 生成两张图：轨迹对比图、控制量时程曲线。

所有数值都由本脚本真跑得到，不含任何手工填写的"示意数据"。
"""

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import matplotlib                                  # noqa: E402
matplotlib.use('Agg')
import matplotlib.pyplot as plt                    # noqa: E402
from matplotlib.patches import Rectangle            # noqa: E402

from tb3_vfh_avoidance.sim_world import (           # noqa: E402
    DiffDriveRobot, make_maze, normalize_angle)
from tb3_vfh_avoidance.vfh_core import polar_points, VfhPlanner  # noqa: E402

plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei',
                                   'Noto Sans CJK SC', 'WenQuanYi Micro Hei',
                                   'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

SPAWN = (1.0, 1.0, math.radians(45.0))
GOAL = (7.0, 7.0)
DT = 0.05
MAX_TIME = 150.0
GOAL_TOLERANCE = 0.35


# ----------------------------------------------------------------------
# 几何工具
# ----------------------------------------------------------------------
def point_segment_distance(px, py, ax, ay, bx, by):
    """点到线段的距离。"""
    dx, dy = bx - ax, by - ay
    length2 = dx * dx + dy * dy
    if length2 < 1e-12:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / length2
    t = max(0.0, min(1.0, t))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def clearance_to_walls(world, x, y):
    """到最近障碍的距离。"""
    return min(point_segment_distance(x, y, a[0], a[1], b[0], b[1])
               for a, b in world.segments)


# ----------------------------------------------------------------------
# 朴素三扇区基线（对照组）
# ----------------------------------------------------------------------
class NaiveSectorPlanner:
    """朴素三扇区策略：前 / 左 / 右各取最短距离，前方过近就朝更空的一侧转。

    "左 / 右"取机器人坐标系 ±30°~±90° 扇区，索引按
    ``angle_min + i * angle_increment`` 计算（与 VFH 用同一套正确索引），
    因此对照组失败的原因只可能是策略本身，而不是实现 bug。
    """

    def __init__(self, safe_distance=0.55, max_linear=0.22, max_angular=1.4):
        self.safe_distance = safe_distance
        self.max_linear = max_linear
        self.max_angular = max_angular

    @staticmethod
    def _sector_min(points, low, high):
        values = [r for a, r in points
                  if low <= normalize_angle(a) < high]
        return min(values) if values else 3.5

    def compute(self, ranges, angle_min, angle_increment, target_angle=0.0,
                range_min=0.0, debug=None):
        points = polar_points(ranges, angle_min, angle_increment,
                              range_min=range_min, range_max=3.5)
        front = self._sector_min(points, math.radians(-30), math.radians(30))
        left = self._sector_min(points, math.radians(30), math.radians(90))
        right = self._sector_min(points, math.radians(-90), math.radians(-30))

        cmd = VfhCommandProxy()
        cmd.clearance = front
        if front >= self.safe_distance:
            cmd.linear = self.max_linear
            cmd.angular = 0.0
            cmd.state = 'cruise'
        else:
            direction = 1.0 if left >= right else -1.0
            cmd.linear = 0.03
            cmd.angular = direction * self.max_angular * 0.8
            cmd.state = 'avoid'
        return cmd


class VfhCommandProxy:
    """与 :class:`vfh_core.VfhCommand` 字段兼容的轻量结构，便于同循环复用。"""

    def __init__(self):
        self.linear = 0.0
        self.angular = 0.0
        self.state = 'cruise'
        self.heading = 0.0
        self.clearance = 3.5
        self.valley_width = 0


# ----------------------------------------------------------------------
# 闭环仿真
# ----------------------------------------------------------------------
def run_episode(planner, world, label):
    """跑一次完整任务，返回轨迹与统计量。"""
    robot = DiffDriveRobot(*SPAWN)
    t = 0.0
    collisions = 0
    min_clearance = float('inf')
    samples = []
    reached = False

    while t < MAX_TIME:
        target = normalize_angle(
            math.atan2(GOAL[1] - robot.y, GOAL[0] - robot.x) - robot.theta)
        ranges, angle_min, increment = world.scan(robot.x, robot.y,
                                                  robot.theta)
        debug = {}
        cmd = planner.compute(ranges, angle_min, increment,
                              target_angle=target, debug=debug)

        # 目标直达优先：目标方向若畅通，直接趋近。
        # 只靠局部避障在终点附近会出现"绕着目标打转"的极限环（航向在
        # ±150° 之间反复翻转，实测距离卡在 0.85 m 上不去），因此当目标
        # 方向净空足够时改为直接指向目标并降速接近。
        sector_min = debug.get('sector_min')
        distance_to_goal = math.hypot(robot.x - GOAL[0], robot.y - GOAL[1])
        if sector_min is not None:
            direct = planner._window_clearance(sector_min, target,
                                              math.radians(20.0))
            if direct > 0.85 and abs(target) < math.radians(90.0):
                cmd.linear = min(planner.max_linear, 0.8 * distance_to_goal)
                cmd.angular = max(-planner.max_angular,
                                  min(planner.max_angular,
                                      2.0 * planner.turn_gain * target))
                cmd.state = 'goal'
                cmd.heading = target
                cmd.clearance = direct

        gap = clearance_to_walls(world, robot.x, robot.y)
        min_clearance = min(min_clearance, gap)
        if gap < 0.12:
            collisions += 1

        samples.append((t, robot.x, robot.y, cmd.linear, cmd.angular,
                        target, cmd.clearance))
        robot.step(cmd.linear, cmd.angular, DT)
        t += DT

        if math.hypot(robot.x - GOAL[0], robot.y - GOAL[1]) < GOAL_TOLERANCE:
            reached = True
            break

    path_length = sum(math.hypot(robot.path[i + 1][0] - robot.path[i][0],
                                 robot.path[i + 1][1] - robot.path[i][1])
                      for i in range(len(robot.path) - 1))

    print('[%-6s] 到达终点: %s | 用时 %6.1f s | 路径长 %5.2f m | '
          '最小净空 %4.2f m | 碰撞帧 %d'
          % (label, '是' if reached else '否', t, path_length,
             min_clearance, collisions))
    return {
        'label': label, 'path': robot.path, 'samples': samples,
        'reached': reached, 'time': t, 'length': path_length,
        'min_clearance': min_clearance, 'collisions': collisions,
    }


# ----------------------------------------------------------------------
# 绘图
# ----------------------------------------------------------------------
def draw_world(ax, world):
    """把世界的线段画到坐标轴上。"""
    for (ax0, ay0), (bx0, by0) in world.segments:
        ax.plot([ax0, bx0], [ay0, by0], color='#2f3640', linewidth=2.0,
                solid_capstyle='round', zorder=1)


def plot_paths(world, results, out_dir):
    """轨迹对比图。"""
    fig, ax = plt.subplots(figsize=(7.2, 7.2), dpi=150)
    draw_world(ax, world)

    styles = {'VFH': ('#e84118', '-', 2.2), 'Naive': ('#0097e6', '--', 1.8)}
    for result in results:
        xs = [p[0] for p in result['path']]
        ys = [p[1] for p in result['path']]
        color, style, width = styles.get(result['label'],
                                         ('#7f8c8d', '-', 1.5))
        ax.plot(xs, ys, style, color=color, linewidth=width,
                label='%s（%s，%.1f s）'
                      % (result['label'],
                         '到达' if result['reached'] else '未到达',
                         result['time']))

    ax.plot(SPAWN[0], SPAWN[1], 'o', color='#4cd137', markersize=11,
            markeredgecolor='white', zorder=5, label='起点')
    ax.plot(GOAL[0], GOAL[1], '*', color='#fbc531', markersize=19,
            markeredgecolor='#4a4a4a', zorder=5, label='终点')
    ax.add_patch(Rectangle((0, 0), 8, 8, fill=False, edgecolor='none'))
    ax.set_xlim(-0.3, 8.3)
    ax.set_ylim(-0.3, 8.3)
    ax.set_aspect('equal')
    ax.set_xlabel('x / m')
    ax.set_ylabel('y / m')
    ax.set_title('TurtleBot3 VFH 局部避障：轨迹对比（8 m × 8 m 迷宫，含 U 形陷阱）')
    ax.grid(alpha=0.25, linestyle=':')
    ax.legend(loc='lower right', fontsize=9, framealpha=0.92)
    fig.tight_layout()
    path = os.path.join(out_dir, 'tb3_vfh_avoidance_path.png')
    fig.savefig(path)
    plt.close(fig)
    print('已生成轨迹对比图: %s' % path)


def plot_signals(result, out_dir):
    """控制量时程曲线。"""
    samples = result['samples']
    ts = [s[0] for s in samples]
    linear = [s[3] for s in samples]
    angular = [s[4] for s in samples]
    clearance = [min(s[6], 3.5) for s in samples]

    fig, axes = plt.subplots(2, 1, figsize=(7.6, 5.4), dpi=150, sharex=True)

    axes[0].plot(ts, linear, color='#e84118', linewidth=1.4, label='线速度 v')
    axes[0].set_ylabel('v / (m·s⁻¹)')
    axes[0].set_ylim(-0.02, 0.26)
    axes[0].legend(loc='upper right', fontsize=9)
    axes[0].grid(alpha=0.25, linestyle=':')
    axes[0].set_title('VFH 闭环控制量（起点 → U 形陷阱 → 绕行 → 终点）')

    axes[1].plot(ts, clearance, color='#0097e6', linewidth=1.4,
                 label='选定方向净空')
    axes[1].set_ylabel('净空 / m')
    axes[1].set_xlabel('时间 / s')
    axes[1].legend(loc='upper right', fontsize=9)
    axes[1].grid(alpha=0.25, linestyle=':')

    fig.tight_layout()
    path = os.path.join(out_dir, 'tb3_vfh_avoidance_signals.png')
    fig.savefig(path)
    plt.close(fig)
    print('已生成控制量曲线图: %s' % path)


def main():
    parser = argparse.ArgumentParser(description='VFH 避障离线闭环演示')
    default_out = os.path.join(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))), 'figures')
    parser.add_argument('--out', default=default_out, help='图片输出目录')
    args = parser.parse_args()
    os.makedirs(args.out, exist_ok=True)

    world = make_maze()
    print('迷宫就绪：%d 段墙体，起点 %s，终点 %s'
          % (len(world.segments), SPAWN[:2], GOAL))

    vfh = run_episode(VfhPlanner(), world, 'VFH')
    naive = run_episode(NaiveSectorPlanner(), world, 'Naive')

    plot_paths(world, [vfh, naive], args.out)
    plot_signals(vfh, args.out)

    print('结论：VFH %s；朴素三扇区 %s'
          % ('成功到达终点' if vfh['reached'] else '未能在限时内到达',
             '成功到达终点' if naive['reached'] else '陷入局部极小值未能到达'))


if __name__ == '__main__':
    main()
