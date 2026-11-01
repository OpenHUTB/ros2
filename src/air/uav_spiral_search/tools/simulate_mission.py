#!/usr/bin/env python3
"""Closed-loop mission simulation without AirSim.

用一阶惯性速度模型代替仿真器，验证螺旋搜索状态机、跟踪控制和
目标锁定的完整闭环；可选输出轨迹图（--plot）。

用法：
    python3 tools/simulate_mission.py --target-x 12 --target-y -8 --plot sim.png
"""
import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from uav_spiral_search.spiral_search import Settings, SpiralSearchController


class SimpleDrone:
    """速度指令 -> 一阶惯性速度 -> 位置积分（世界 NED）。"""

    def __init__(self, x, y, z, yaw, tau=0.6):
        self.pos = [x, y, z]
        self.yaw = yaw
        self.tau = tau
        self.vel = [0.0, 0.0, 0.0]

    def step(self, command, dt):
        for i in range(3):
            self.vel[i] += (command[i] - self.vel[i]) / self.tau * dt
            self.pos[i] += self.vel[i] * dt


def camera_sees(drone, target, half_fov, max_range=30.0, min_range=1.0):
    """前视相机模型：返回 (可见, 检测框归一化横坐标)。"""
    dx = target[0] - drone.pos[0]
    dy = target[1] - drone.pos[1]
    dist = math.hypot(dx, dy)
    if not min_range <= dist <= max_range:
        return False, 0.0
    bearing = math.atan2(dy, dx)
    error = math.atan2(math.sin(bearing - drone.yaw),
                       math.cos(bearing - drone.yaw))
    if abs(error) > half_fov:
        return False, 0.0
    # 目标在视场内的水平偏移，映射为归一化横坐标 [-1, 1]。
    return True, max(-1.0, min(1.0, error / half_fov))


def run_simulation(args):
    settings = Settings()
    controller = SpiralSearchController(settings)
    drone = SimpleDrone(args.start_x, args.start_y, args.start_z,
                        math.radians(args.yaw_deg))
    target = (args.target_x, args.target_y)

    log = {'t': [], 'pos': [], 'ref': [], 'state': [], 'confirm': [],
           'dist': []}
    now = 0.0
    next_image = 0.0
    command = (0.0, 0.0, 0.0)
    report = {'state': 'waiting_odom'}
    while now <= args.max_time:
        drone.step(command, args.dt)
        now += args.dt
        # 图像线程 5 Hz：按相机模型给出可见性与检测框横坐标。
        if now + 1e-9 >= next_image:
            visible, cx = camera_sees(drone, target,
                                      settings.camera_half_fov)
            controller.update_detection(visible, now, cx)
            next_image += 0.2
        controller.update_pose(drone.pos[0], drone.pos[1], drone.pos[2],
                               drone.yaw, now)
        command, state, report = controller.step(now)
        if not log['t'] or now - log['t'][-1] >= 0.1:  # 10 Hz 记录
            ref_point = (controller.reference if state == 'searching'
                         else (controller.hover_point[:2]
                               if controller.hover_point else None))
            log['t'].append(now)
            log['pos'].append((drone.pos[0], drone.pos[1]))
            log['ref'].append(ref_point)
            log['state'].append(state)
            log['confirm'].append(controller.confirm_count)
            log['dist'].append(math.hypot(drone.pos[0] - target[0],
                                          drone.pos[1] - target[1]))
        if state in ('locked', 'exhausted'):
            break
    return controller, report, target, log


def print_metrics(report, target):
    print(f'最终状态      : {report["state"]}')
    print(f'搜索用时      : {report["search_time_s"]} s')
    print(f'飞行水平路程  : {report["path_len_m"]} m')
    if report['target_estimate_ned'] is not None:
        ex, ey = report['target_estimate_ned']
        error = math.hypot(ex - target[0], ey - target[1])
        print(f'目标真实位置  : ({target[0]:.2f}, {target[1]:.2f}) m')
        print(f'目标位置估计  : ({ex:.2f}, {ey:.2f}) m')
        print(f'估计误差      : {error:.2f} m')


def save_plot(report, target, log, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.6))
    px = [p[0] for p in log['pos']]
    py = [p[1] for p in log['pos']]
    rx = [r[0] for r in log['ref'] if r]
    ry = [r[1] for r in log['ref'] if r]
    ax1.plot(px, py, lw=1.6, label='actual trajectory')
    ax1.plot(rx, ry, ':', lw=1.0, color='0.5', label='reference')
    ax1.plot(log['pos'][0][0], log['pos'][0][1], 'k^', ms=9, label='start')
    ax1.plot(*target, 'r*', ms=15, label='target (ground truth)')
    if report['target_estimate_ned'] is not None:
        ax1.plot(*report['target_estimate_ned'], 'rx', ms=11,
                 mew=2, label='target estimate')
    ax1.set_xlabel('x / NED m')
    ax1.set_ylabel('y / NED m')
    ax1.set_aspect('equal', adjustable='datalim')
    ax1.set_title(f'mission: {report["state"]} after '
                  f'{report["search_time_s"]} s, {report["path_len_m"]} m')
    ax1.legend(loc='upper left', fontsize=8)
    ax1.grid(alpha=0.3)

    ax2.plot(log['t'], log['dist'], lw=1.2, color='tab:blue',
             label='distance to target')
    ax2.plot(log['t'], log['confirm'], lw=1.2, color='tab:red',
             label='consecutive detections')
    spans = {'climbing': '#ffe08a', 'searching': '#a8d8ff',
             'locked': '#b8f2b8', 'exhausted': '#ffb3b3'}
    start = 0
    for i in range(1, len(log['t']) + 1):
        if i == len(log['t']) or log['state'][i] != log['state'][start]:
            color = spans.get(log['state'][start])
            if color:
                ax2.axvspan(log['t'][max(0, start - 1)], log['t'][i - 1],
                            color=color, alpha=0.35, lw=0,
                            label=log['state'][start])
            start = i
    ax2.set_xlabel('time / s')
    ax2.set_ylabel('value')
    ax2.legend(loc='upper right', fontsize=8)
    ax2.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    print(f'轨迹图已保存  : {path}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--target-x', type=float, default=12.0)
    parser.add_argument('--target-y', type=float, default=-8.0)
    parser.add_argument('--start-x', type=float, default=0.0)
    parser.add_argument('--start-y', type=float, default=0.0)
    parser.add_argument('--start-z', type=float, default=-1.0)
    parser.add_argument('--yaw-deg', type=float, default=0.0)
    parser.add_argument('--dt', type=float, default=0.05)
    parser.add_argument('--max-time', type=float, default=300.0)
    parser.add_argument('--plot', type=str, default='',
                        help='保存轨迹图的路径（需要 matplotlib）')
    args = parser.parse_args()

    _, report, target, log = run_simulation(args)
    print_metrics(report, target)
    if args.plot:
        save_plot(report, target, log, args.plot)


if __name__ == '__main__':
    main()
