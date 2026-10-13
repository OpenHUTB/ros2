#!/usr/bin/env python3
"""编队性能评测：采集 /odom，计算四个指标，输出 CSV、曲线图与可直接粘贴的表格行。

用法（仿真已启动、formation_controller 正在运行时）：
    ros2 run holoocean_examples formation_controller ...      # 终端 A
    python3 evaluate_formation.py --duration 60 --leader auv0 --followers auv1

输出：
    formation_metrics.csv        原始采样（时间、各载具位置、编队误差）
    formation_error.png          编队误差随时间变化曲线（可直接放进章节）
    （终端）                      RMS / 最大误差 / 收敛时间 / 速度波动 + Markdown 表格行
"""

import argparse
import csv
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry


class FormationProbe(Node):

    def __init__(self, leader, followers, offsets, rate_hz=20.0):
        super().__init__('formation_probe')
        self.leader = leader
        self.followers = followers
        self.offsets = offsets
        self.samples = []          # (t, agent, x, y, z, error)
        self.vel = {}              # agent -> [speed...]
        self.t0 = None
        self.pose = {}

        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        for name in [leader] + followers:
            self.create_subscription(
                Odometry, '/holoocean/%s/DynamicsSensorOdom' % name,
                lambda msg, n=name: self._cb(n, msg), qos)

        self.timer = self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(
            '评测探针就绪：leader=%s followers=%s' % (leader, followers))

    def _cb(self, name, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        p = msg.pose.pose.position
        self.pose[name] = (p.x, p.y, p.z, yaw)
        v = msg.twist.twist.linear
        self.vel.setdefault(name, []).append(math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2))

    def _tick(self):
        if self.leader not in self.pose:
            return
        if self.t0 is None:
            self.t0 = time.time()
        t = time.time() - self.t0
        lx, ly, lz, lyaw = self.pose[self.leader]
        c, s = math.cos(lyaw), math.sin(lyaw)
        for name, (dx, dy, dz) in zip(self.followers, self.offsets):
            if name not in self.pose:
                continue
            tx = lx + (c * dx - s * dy)
            ty = ly + (s * dx + c * dy)
            tz = lz + dz
            px, py, pz, _ = self.pose[name]
            err = math.sqrt((tx - px) ** 2 + (ty - py) ** 2 + (tz - pz) ** 2)
            self.samples.append((t, name, px, py, pz, err))


def stats(samples, follower, thresh):
    errs = [(t, e) for t, n, _x, _y, _z, e in samples if n == follower]
    if not errs:
        return None
    vals = [e for _t, e in errs]
    rms = math.sqrt(sum(v * v for v in vals) / len(vals))
    # 收敛时间：误差峰值之后首次回到 ±thresh 以内。
    # 跟随者从期望队形出发，起点就在阈值内，直接取"首个低于阈值"会恒为 0，没有意义。
    i_pk = max(range(len(vals)), key=lambda i: vals[i])
    conv = next((errs[i][0] for i in range(i_pk, len(errs)) if errs[i][1] < thresh), None)
    # 收敛后再看稳态段（最后 30% 采样）
    tail = vals[int(len(vals) * 0.7):]
    steady = sum(tail) / len(tail) if tail else float('nan')
    return dict(rms=rms, mx=max(vals), conv=conv, steady=steady, n=len(vals))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=60.0, help='采集时长（秒）')
    ap.add_argument('--leader', default='auv0')
    ap.add_argument('--followers', default='auv1')
    ap.add_argument('--offsets', default='-3,0,0', help='机体系偏移，多个跟随者用分号分隔')
    ap.add_argument('--thresh', type=float, default=0.3, help='收敛判据（米）')
    ap.add_argument('--outdir', default='.')
    args = ap.parse_args()

    followers = [x for x in args.followers.split(',') if x]
    offs = [[float(v) for v in part.split(',')] for part in args.offsets.split(';')]
    if len(offs) == 1:
        offs = offs * len(followers)

    rclpy.init()
    node = FormationProbe(args.leader, followers, offs)
    end = time.time() + args.duration
    while rclpy.ok() and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    samples = node.samples
    if not samples:
        print('没有采到数据：确认仿真与 formation_controller 正在运行，'
              '且话题 /holoocean/<agent>/DynamicsSensorOdom 存在。')
        return 1

    csv_path = os.path.join(args.outdir, 'formation_metrics.csv')
    with open(csv_path, 'w', newline='', encoding='utf-8') as fh:
        w = csv.writer(fh)
        w.writerow(['t_sec', 'agent', 'x', 'y', 'z', 'formation_error_m'])
        w.writerows(samples)
    print('原始数据：%s（%d 行）' % (csv_path, len(samples)))

    print()
    print('| 指标 | 数值 |')
    print('| --- | --- |')
    for f in followers:
        st = stats(samples, f, args.thresh)
        if not st:
            print('| %s 编队误差 RMS | 无数据 |' % f)
            continue
        conv = ('%.2f s' % st['conv']) if st['conv'] is not None else '未在采集窗口内收敛'
        print('| %s 编队误差 RMS | %.3f m |' % (f, st['rms']))
        print('| %s 最大瞬时误差 | %.3f m |' % (f, st['mx']))
        print('| %s 峰值后收敛时间（回到 ±%.1f m） | %s |' % (f, args.thresh, conv))
        print('| %s 稳态误差（后 30%% 采样均值） | %.3f m |' % (f, st['steady']))
        vs = node.vel.get(f) or []
        if vs:
            mean = sum(vs) / len(vs)
            std = math.sqrt(sum((v - mean) ** 2 for v in vs) / len(vs))
            print('| %s 稳态速度波动（标准差） | %.3f m/s |' % (f, std))

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        plt.figure(figsize=(8, 3.6))
        for f in followers:
            ts = [t for t, n, _x, _y, _z, _e in samples if n == f]
            es = [e for _t, n, _x, _y, _z, e in samples if n == f]
            plt.plot(ts, es, label='%s 编队误差' % f, linewidth=1.4)
        plt.axhline(args.thresh, color='gray', linestyle='--', linewidth=1,
                    label='收敛阈值 %.1f m' % args.thresh)
        plt.xlabel('时间 (s)')
        plt.ylabel('编队误差 (m)')
        plt.title('编队误差收敛曲线')
        plt.grid(alpha=0.3)
        plt.legend()
        plt.tight_layout()
        png = os.path.join(args.outdir, 'formation_error.png')
        plt.savefig(png, dpi=160)
        print()
        print('曲线图：%s' % png)
    except ImportError:
        print()
        print('（未安装 matplotlib，跳过绘图：pip install matplotlib 后重跑即可出图）')

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
