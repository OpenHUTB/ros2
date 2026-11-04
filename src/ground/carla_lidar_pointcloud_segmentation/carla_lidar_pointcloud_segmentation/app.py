"""CARLA 采集与在线演示。

两个入口：

* :func:`run_collect` —— 自车开自动驾驶，同时录下"普通点云"和"语义点云"，
  对齐后存成 ``dataset.npz``（``points`` / ``tags`` / ``frames`` 三个数组）；
* :func:`run_demo` —— 加载训练好的权重，边跑边分割，终端打印每帧的
  可行驶比例与前方余量；装了 matplotlib 还能弹出三维散点图看分割效果。

数据落盘的键名与 :mod:`pipeline.frame_features` 完全对应，训练脚本直接吃这个文件。
"""

from __future__ import annotations

import json
import os
import time

import numpy as np

from . import carla_common as cc
from . import control
from . import features as F
from . import labels as L
from . import model as MD


def _log_every(frame, total, started):
    """每 10% 打一行进度，避免刷屏。"""
    if frame == 0 or (frame + 1) % max(1, total // 10) == 0:
        elapsed = time.time() - started
        rate = (frame + 1) / elapsed if elapsed > 0 else 0.0
        print('[采集] %d/%d 帧，%.1f 帧/秒' % (frame + 1, total, rate))


def run_collect(args):
    """采集成对点云并存盘。"""
    client, world = cc.connect(args.host, args.port, args.town, sync=True, dt=args.dt)
    sensors, actors = [], []
    started = time.time()
    try:
        ego = cc.spawn_ego(world, args.vehicle, args.spawn_index, autopilot=True)
        print('[自车] %s 已在出生点 %d' % (ego.type_id, args.spawn_index))

        if args.traffic > 0:
            actors = cc.spawn_traffic(world, client, count=args.traffic)
        actors.append(ego)

        common = dict(lidar_range=args.lidar_range, channels=args.channels,
                      points_per_second=args.points_per_second,
                      rotation_frequency=args.rotation, sensor_tick=0.0)
        raw_sensor, raw_q = cc.attach_lidar(
            world, ego, cc.lidar_blueprint(world, semantic=False, **common), args.lidar_height)
        sem_sensor, sem_q = cc.attach_lidar(
            world, ego, cc.lidar_blueprint(world, semantic=True, **common), args.lidar_height)
        sensors = [raw_sensor, sem_sensor]

        # 先空跑几帧，等传感器稳定
        for _ in range(5):
            world.tick()

        all_points, all_tags, all_frames = [], [], []
        dropped = 0
        for frame in range(args.frames):
            world.tick()
            try:
                raw = raw_q.get(timeout=10.0)
                sem = sem_q.get(timeout=10.0)
            except Exception:      # noqa: BLE001
                print('[采集] 第 %d 帧等不到传感器数据，跳过' % frame)
                continue

            raw_pts = cc.points_from_raw(raw)
            sem_pts, sem_tags = cc.points_from_semantic(sem)
            points, tags, mask = cc.align_pair(raw_pts, sem_pts, sem_tags, tolerance=0.05)
            dropped += int((~mask).sum())
            if not mask.any():
                continue

            all_points.append(points[mask])
            all_tags.append(tags[mask])
            all_frames.append(np.full(int(mask.sum()), frame, dtype=np.int32))
            _log_every(frame, args.frames, started)

        if not all_points:
            print('[采集] 没有采到有效数据')
            return 3

        points = np.vstack(all_points).astype(np.float32)
        tags = np.concatenate(all_tags).astype(np.int32)
        frames = np.concatenate(all_frames).astype(np.int32)

        os.makedirs(args.out_dir, exist_ok=True)
        dataset_path = os.path.join(args.out_dir, 'dataset.npz')
        np.savez_compressed(dataset_path, points=points, tags=tags, frames=frames)

        meta = {
            'town': args.town,
            'vehicle': args.vehicle,
            'frames': int(len(np.unique(frames))),
            'points': int(points.shape[0]),
            'unmatched_points_dropped': int(dropped),
            'lidar': {'range': args.lidar_range, 'channels': args.channels,
                      'points_per_second': args.points_per_second,
                      'rotation_frequency': args.rotation,
                      'height': args.lidar_height},
            'class_distribution': L.class_distribution(tags),
            'size_bytes': int(points.nbytes + tags.nbytes + frames.nbytes),
        }
        with open(os.path.join(args.out_dir, 'dataset_meta.json'), 'w', encoding='utf-8') as handle:
            json.dump(meta, handle, ensure_ascii=False, indent=2)

        print('=' * 60)
        print('采集完成：%s' % dataset_path)
        print('  帧数 %d，点数 %d，配对失败丢弃 %d' % (meta['frames'], meta['points'], dropped))
        print('  类别分布：%s' % meta['class_distribution'])
        print('  文件大小 %.1f MB，未压缩数组 %.1f MB'
              % (os.path.getsize(dataset_path) / 1024 ** 2, meta['size_bytes'] / 1024 ** 2))
        print('  元信息：%s' % os.path.join(args.out_dir, 'dataset_meta.json'))
        print('=' * 60)
        return 0
    finally:
        for sensor in sensors:
            try:
                sensor.stop()
                sensor.destroy()
            except Exception:      # noqa: BLE001
                pass
        for actor in actors:
            try:
                actor.destroy()
            except Exception:      # noqa: BLE001
                pass
        cc.restore_async(world)
        print('[清理] 传感器与车辆已销毁，世界已恢复异步模式')


def load_model(checkpoint, n_features=None):
    """加载权重；缺元信息时按特征维度兜底。"""
    if not os.path.isfile(checkpoint):
        raise FileNotFoundError('找不到权重文件：%s（先跑 --train）' % checkpoint)
    net = MD.MLP.load(checkpoint)
    if n_features is not None and net.n_in != n_features:
        raise ValueError('权重输入维度 %d 与特征维度 %d 不匹配' % (net.n_in, n_features))
    return net


def segment_frame(net, points, mean, std):
    """对单帧点云做推理，返回逐点类别。"""
    feats = F.point_features(points, ground_z=F.estimate_ground_z(points))
    return net.predict(F.standardize_apply(feats, mean, std))


def run_demo(args):
    """在线演示：边采边分割，打印可行驶比例与前方余量；可选三维散点图。"""
    net = load_model(args.checkpoint)
    mean = np.zeros(net.n_in)
    std = np.ones(net.n_in)
    stats_path = os.path.splitext(args.checkpoint)[0] + '.stats.npz'
    if os.path.isfile(stats_path):
        with np.load(stats_path) as data:
            mean, std = data['mean'], data['std']

    client, world = cc.connect(args.host, args.port, args.town, sync=True, dt=args.dt)
    sensors, actors = [], []
    plot = None
    if not args.no_plot:
        try:
            import matplotlib
            matplotlib.use('TkAgg')
            import matplotlib.pyplot as plt
            plot = plt
        except Exception as exc:      # noqa: BLE001
            print('[演示] 画图不可用（%s），改为纯文字输出' % exc)

    try:
        ego = cc.spawn_ego(world, args.vehicle, args.spawn_index, autopilot=True)
        if args.traffic > 0:
            actors = cc.spawn_traffic(world, client, count=args.traffic)
        actors.append(ego)
        common = dict(lidar_range=args.lidar_range, channels=args.channels,
                      points_per_second=args.points_per_second,
                      rotation_frequency=args.rotation, sensor_tick=0.0)
        raw_sensor, raw_q = cc.attach_lidar(
            world, ego, cc.lidar_blueprint(world, semantic=False, **common), args.lidar_height)
        sensors = [raw_sensor]
        for _ in range(5):
            world.tick()

        colors = np.array([[0.2, 0.8, 0.2], [0.9, 0.2, 0.2], [1.0, 0.8, 0.1], [0.4, 0.4, 0.9]])
        for frame in range(args.frames):
            world.tick()
            try:
                data = raw_q.get(timeout=10.0)
            except Exception:      # noqa: BLE001
                continue
            points = cc.points_from_raw(data)
            classes = segment_frame(net, points, mean, std)
            free = control.free_distance(points, classes, half_width=args.corridor_width)
            ratio = control.drivable_ratio(classes, points=points)
            print('[演示] 帧 %3d 点数 %5d 可行驶比例 %.2f 前方余量 %.1f m'
                  % (frame, points.shape[0], ratio, free))

            if plot is not None:
                plot.clf()
                ax = plot.subplot(111, projection='3d') if hasattr(plot, 'subplot') else plot.gca()
                ax.cla()
                for cls_id in range(L.N_CLASSES):
                    mask = classes == cls_id
                    if mask.any():
                        ax.scatter(points[mask, 0], points[mask, 1], points[mask, 2],
                                   s=1, c=[colors[cls_id]], label=L.CLASS_NAMES[cls_id])
                ax.set_xlim(0, 40)
                ax.set_ylim(-15, 15)
                ax.set_zlim(-3, 6)
                ax.set_xlabel('x / m')
                ax.set_ylabel('y / m')
                ax.legend(loc='upper right', markerscale=6)
                plot.pause(0.001)
        if plot is not None:
            plot.close()
        return 0
    finally:
        for sensor in sensors:
            try:
                sensor.stop()
                sensor.destroy()
            except Exception:      # noqa: BLE001
                pass
        for actor in actors:
            try:
                actor.destroy()
            except Exception:      # noqa: BLE001
                pass
        cc.restore_async(world)
        print('[清理] 已完成')