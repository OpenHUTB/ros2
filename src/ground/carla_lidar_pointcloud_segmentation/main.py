#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""carla_lidar_pointcloud_segmentation 统一入口（仓库约定：入口为 main.*）。

用 CARLA 的语义激光雷达当监督信号，训练普通激光雷达点云的逐点语义分割网络，
再用分割结果给轨迹跟踪提供"哪里能走、哪里不能走"的信息。

用法::

    # 1. 离线自检：合成点云跑通"特征 -> 训练 -> 评价"全链路（不需要 CARLA）
    python3 main.py --self-check

    # 2. 在 CARLA 里采集数据（需要 CARLA 服务端）
    python3 main.py --collect --host 192.168.148.1 --town Town03 --frames 300

    # 3. 用采集到的数据训练并评价
    python3 main.py --train --data data/dataset.npz

    # 4. 在线演示：加载权重边跑边分割
    python3 main.py --demo --host 192.168.148.1
"""

import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from carla_lidar_pointcloud_segmentation import labels as L      # noqa: E402
from carla_lidar_pointcloud_segmentation import pipeline as P    # noqa: E402

# 合成数据里用的原始语义标签，直接取实测表的首个标签：
# 这样以后重新体检、调整实测表时，自检数据会自动跟着变，不会悄悄用旧编号。
TAG_ROAD = L.first_tag_of(L.DRIVABLE)
TAG_VEHICLE = L.first_tag_of(L.VEHICLE)
TAG_PEDESTRIAN = L.first_tag_of(L.PEDESTRIAN)
TAG_BUILDING = L.first_tag_of(L.OBSTACLE)


def build_parser():
    parser = argparse.ArgumentParser(
        description='CARLA 激光雷达点云语义分割（语义激光雷达自监督）')

    # 运行模式
    parser.add_argument('--self-check', action='store_true', help='离线自检，不需要 CARLA')
    parser.add_argument('--collect', action='store_true', help='在 CARLA 里采集数据集')
    parser.add_argument('--train', action='store_true', help='用数据集训练并评价')
    parser.add_argument('--demo', action='store_true', help='在线分割演示')
    parser.add_argument('--probe', action='store_true',
                        help='实测语义标签编号，换地图或换 CARLA 版本后先跑这个')

    # CARLA 连接
    parser.add_argument('--host', default='127.0.0.1', help='CARLA 服务端地址')
    parser.add_argument('--port', type=int, default=2000, help='CARLA 服务端端口')
    parser.add_argument('--town', default='Town03', help='地图名')
    parser.add_argument('--vehicle', default='vehicle.tesla.model3', help='自车 blueprint')
    parser.add_argument('--spawn-index', type=int, default=0, help='出生点索引')
    parser.add_argument('--traffic', type=int, default=12, help='NPC 车数量，0 表示不放')
    parser.add_argument('--pedestrians', type=int, default=8, help='行人数量，0 表示不放')
    parser.add_argument('--dt', type=float, default=0.05, help='同步步长（秒）')

    # 激光雷达参数（普通与语义两台必须一致）
    parser.add_argument('--lidar-range', type=float, default=60.0, help='量程（米）')
    parser.add_argument('--channels', type=int, default=32, help='线数')
    parser.add_argument('--points-per-second', type=int, default=200000,
                        help='点频，双雷达各一路；虚拟机里太高会拖到两帧每秒')
    parser.add_argument('--rotation', type=float, default=20.0, help='转速（Hz）')
    parser.add_argument('--lidar-height', type=float, default=2.4, help='雷达安装高度（米）')

    # 数据与模型
    parser.add_argument('--frames', type=int, default=300, help='采集/演示帧数')
    parser.add_argument('--out-dir', default='data', help='数据集输出目录')
    parser.add_argument('--data', default=os.path.join('data', 'dataset.npz'),
                        help='训练用数据集路径')
    parser.add_argument('--checkpoint', default=os.path.join('models', 'seg_mlp.npz'),
                        help='权重路径（models/ 下的 npz 会入库，方便免训复现）')
    parser.add_argument('--epochs', type=int, default=30, help='训练轮数')
    parser.add_argument('--lr', type=float, default=0.01, help='学习率')
    parser.add_argument('--hidden', default='32,32', help='隐层宽度，逗号分隔')
    parser.add_argument('--figure-dir', default='figures', help='训练曲线输出目录')
    parser.add_argument('--no-figures', action='store_true', help='不出图')
    parser.add_argument('--no-weight', action='store_true', help='不加类别权重')

    # 控制与画图
    parser.add_argument('--corridor-width', type=float, default=1.2, help='走廊半宽（米）')
    parser.add_argument('--no-plot', action='store_true', help='演示时不弹三维图')
    return parser


def synthetic_dataset(n_frames=6, seed=0):
    """造一份几何上可分的合成点云，用来验证整条链路。

    每一帧包含四类点：地面（大面积、位置低）、车辆（前方立方体）、
    行人（近处细高柱）、建筑（两侧高墙）。
    """
    rng = np.random.default_rng(seed)
    all_points, all_tags, all_frames = [], [], []
    for frame in range(n_frames):
        n_road = 1200
        x = rng.uniform(5.0, 40.0, n_road)
        y = rng.uniform(-8.0, 8.0, n_road)
        z = rng.normal(-1.70, 0.05, n_road)
        road = np.column_stack([x, y, z])

        n_veh = 300
        x = rng.uniform(10.0, 25.0, n_veh)
        y = rng.uniform(-2.0, 2.0, n_veh)
        z = rng.uniform(-1.0, 0.2, n_veh)
        veh = np.column_stack([x, y, z])

        n_ped = 150
        x = rng.uniform(6.0, 12.0, n_ped)
        y = rng.uniform(5.0, 7.0, n_ped)
        z = rng.uniform(-1.6, 0.1, n_ped)
        ped = np.column_stack([x, y, z])

        n_bld = 400
        x = rng.uniform(5.0, 40.0, n_bld)
        y = np.where(rng.random(n_bld) < 0.5, rng.uniform(-12.0, -9.0, n_bld),
                     rng.uniform(9.0, 12.0, n_bld))
        z = rng.uniform(-1.0, 6.0, n_bld)
        bld = np.column_stack([x, y, z])

        points = np.vstack([road, veh, ped, bld])
        tags = np.concatenate([
            np.full(n_road, TAG_ROAD),
            np.full(n_veh, TAG_VEHICLE),
            np.full(n_ped, TAG_PEDESTRIAN),
            np.full(n_bld, TAG_BUILDING),
        ])
        all_points.append(points)
        all_tags.append(tags)
        all_frames.append(np.full(points.shape[0], frame))
    return (np.vstack(all_points), np.concatenate(all_tags), np.concatenate(all_frames))


def run_train(args, points, tags, frames, source):
    """特征工程 + 训练 + 评价，并把权重和标准化统计量存到磁盘。"""
    x, y, r = P.frame_features(points, tags, frames)
    if x.shape[0] == 0:
        print('没有可用样本，检查标签映射或采集参数。')
        return 3
    hidden = tuple(int(v) for v in args.hidden.split(',') if v.strip())
    result = P.train_eval(x, y, r, hidden=hidden, epochs=args.epochs, lr=args.lr,
                          weighted=not args.no_weight)

    print('=' * 60)
    print('数据来源：%s' % source)
    print('=' * 60)
    print(P.format_report(result))

    out_dir = os.path.dirname(os.path.abspath(args.checkpoint))
    os.makedirs(out_dir, exist_ok=True)
    result['model'].save(args.checkpoint, meta={
        'classes': list(L.CLASS_NAMES),
        'n_features': int(x.shape[1]),
        'hidden': list(hidden),
        'macro_iou': float(result['report']['macro_iou']),
        'accuracy': float(result['report']['accuracy']),
    })
    stats_path = os.path.splitext(args.checkpoint)[0] + '.stats.npz'
    np.savez_compressed(stats_path, mean=result['mean'], std=result['std'])
    metrics_path = os.path.join(out_dir, 'metrics.json')
    with open(metrics_path, 'w', encoding='utf-8') as handle:
        json.dump({
            'source': source,
            'n_train': int(result['n_train']),
            'n_val': int(result['n_val']),
            'n_parameters': int(result['n_parameters']),
            'confusion_matrix': [[int(v) for v in row]
                                 for row in result['report']['confusion_matrix']],
            'iou': result['report']['iou'],
            'precision': result['report']['precision'],
            'recall': result['report']['recall'],
            'f1': result['report']['f1'],
            'macro_iou': result['report']['macro_iou'],
            'macro_f1': result['report']['macro_f1'],
            'accuracy': result['report']['accuracy'],
            'by_range': result['by_range'],
        }, handle, ensure_ascii=False, indent=2)

    print('\n权重已保存：%s（%.1f KB）'
          % (args.checkpoint, os.path.getsize(args.checkpoint) / 1024.0))
    print('标准化统计量：%s' % stats_path)
    print('指标明细：%s' % metrics_path)

    if not getattr(args, 'no_figures', False):
        from carla_lidar_pointcloud_segmentation import figures
        paths = figures.save_all(result['report'], result['by_range'],
                                 getattr(args, 'figure_dir', 'figures'))
        for path in paths:
            print('图已保存：%s（%.1f KB）'
                  % (path, os.path.getsize(path) / 1024.0))
    return 0


def main(argv=None):
    args = build_parser().parse_args(argv)

    if args.self_check:
        points, tags, frames = synthetic_dataset()
        return run_train(args, points, tags, frames, '合成点云（离线自检）')

    if args.train:
        if not os.path.isfile(args.data):
            print('找不到数据集：%s' % args.data)
            print('先跑 --collect 采集，或用 --self-check 验证链路。')
            return 2
        with np.load(args.data) as data:
            points = np.array(data['points'])
            tags = np.array(data['tags'])
            frames = np.array(data['frames'])
        return run_train(args, points, tags, frames, args.data)

    try:
        from carla_lidar_pointcloud_segmentation import app
    except ImportError:
        print('CARLA 相关部分未接入。可以先用 --self-check 验证离线链路：')
        print('  python3 main.py --self-check')
        return 2

    if args.probe:
        from carla_lidar_pointcloud_segmentation import probe
        return probe.run_probe(args)
    if args.collect:
        return app.run_collect(args)
    if args.demo:
        return app.run_demo(args)

    print('没有指定运行模式，试试 --self-check / --probe / --collect / --train / --demo')
    return 2


if __name__ == '__main__':
    sys.exit(main())