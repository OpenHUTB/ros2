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
import math
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


def _read_matched(raw_q, sem_q, timeout=10.0, max_skew=10):
    """读到同一帧的两份点云。

    两台雷达各自有队列，服务端给出的帧可能差一拍；帧号小的那边多读几帧追上，
    最多容错 ``max_skew`` 次，超过就直接返回当前这对（由坐标配对兜底）。
    """
    raw = raw_q.get(timeout=timeout)
    sem = sem_q.get(timeout=timeout)
    for _ in range(max_skew):
        if raw.frame == sem.frame:
            return raw, sem
        if raw.frame < sem.frame:
            raw = raw_q.get(timeout=timeout)
        else:
            sem = sem_q.get(timeout=timeout)
    return raw, sem


def run_collect(args):
    """采集成对点云并存盘。"""
    client, world = cc.connect(args.host, args.port, args.town, sync=True, dt=args.dt)
    cc.enable_sync_traffic_manager(client)
    sensors, actors = [], []
    started = time.time()
    try:
        ego = cc.spawn_ego(world, args.vehicle, args.spawn_index, autopilot=True)
        print('[自车] %s 已在出生点 %d' % (ego.type_id, args.spawn_index))

        # 先 tick 一次：刚 spawn 的自车位姿还是全零，拿它算"附近"会全部落空
        world.tick()
        if args.traffic > 0:
            actors = cc.spawn_traffic(world, client, count=args.traffic, near=ego)
        actors.append(ego)

        # 行人必须主动放：Town03 默认场景里没有行人，不补的话
        # 数据集里 pedestrian 一直是 0，训练出来的模型永远预测不出行人。
        # 放之前先 tick 一次，让自车位姿下发，否则相对位置会算到世界原点。
        if getattr(args, 'pedestrians', 0) > 0:
            actors.extend(cc.spawn_pedestrians(world, ego, count=args.pedestrians))

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
                raw, sem = _read_matched(raw_q, sem_q, timeout=10.0)
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
            # 存的是原始标签，统计前要先归并成四个类别，否则打印出来是一堆编号
            'class_distribution': L.class_distribution(L.group_labels(tags)),
            'tag_histogram': L.tag_histogram(tags),
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
        cc.teardown(client, world, sensors, actors)


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


def _print_demo_summary(stats, args):
    """把演示过程里的控制指标汇总成一段，方便直接抄进文档。"""
    if not stats['free']:
        return
    free = np.asarray(stats['free'], dtype=float)
    print('=' * 60)
    print('演示汇总：%d 帧' % free.size)
    print('  前方余量：最小 %.1f m，平均 %.1f m'
          % (free.min(), free.mean()))
    print('  触发刹车（刹车量 > 0.1）的帧：%d / %d'
          % (stats['brake_frames'], free.size))
    if stats['lateral']:
        lateral = np.asarray(stats['lateral'], dtype=float)
        print('  横向偏差：平均 %.2f m，最大 %.2f m' % (lateral.mean(), lateral.max()))
    if stats['gap_true']:
        gap = np.asarray(stats['gap_true'], dtype=float)
        print('  与其它车辆的真实最小中心距：%.1f m' % gap.min())
    print('=' * 60)


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
    cc.enable_sync_traffic_manager(client)
    sensors, actors = [], []
    plot = None
    record_fig = None
    if getattr(args, 'record', ''):
        # 录制模式先用无界面后端：虚拟机里没有显示服务，TkAgg 会直接失败。
        # 每帧存一张 PNG，后面用 Pillow 或 ffmpeg 合成 gif。
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        os.makedirs(args.record, exist_ok=True)
        record_fig = plt.figure(figsize=(6.4, 4.8), dpi=80)
        print('[录制] 逐帧写入 %s' % args.record)
    elif not args.no_plot:
        try:
            import matplotlib
            matplotlib.use('TkAgg')
            import matplotlib.pyplot as plt
            plot = plt
        except Exception as exc:      # noqa: BLE001
            print('[演示] 画图不可用（%s），改为纯文字输出' % exc)

    try:
        ego = cc.spawn_ego(world, args.vehicle, args.spawn_index, autopilot=True)
        # 先 tick 一次：刚 spawn 的自车位姿还是全零，拿它算"附近"会全部落空
        world.tick()
        if args.traffic > 0:
            actors = cc.spawn_traffic(world, client, count=args.traffic, near=ego)
        actors.append(ego)

        # 行人必须主动放：Town03 默认场景里没有行人，不补的话
        # 数据集里 pedestrian 一直是 0，训练出来的模型永远预测不出行人。
        # 放之前先 tick 一次，让自车位姿下发，否则相对位置会算到世界原点。
        if getattr(args, 'pedestrians', 0) > 0:
            actors.extend(cc.spawn_pedestrians(world, ego, count=args.pedestrians))
        common = dict(lidar_range=args.lidar_range, channels=args.channels,
                      points_per_second=args.points_per_second,
                      rotation_frequency=args.rotation, sensor_tick=0.0)
        raw_sensor, raw_q = cc.attach_lidar(
            world, ego, cc.lidar_blueprint(world, semantic=False, **common), args.lidar_height)
        sensors = [raw_sensor]
        for _ in range(5):
            world.tick()

        colors = np.array([[0.2, 0.8, 0.2], [0.9, 0.2, 0.2], [1.0, 0.8, 0.1], [0.4, 0.4, 0.9]])
        carla_map = world.get_map()
        if args.control:
            ego.set_autopilot(False)
            # 延迟导入：只有装了 CARLA 才会走到这里。--self-check 在没有 carla
            # 库的机器上也要能跑，所以不能在模块顶部 import carla。
            import carla as carla_module
        stats = {'free': [], 'lateral': [], 'brake_frames': 0, 'gap_true': []}
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
            factor = control.speed_factor(free, comfort=args.comfort_distance,
                                          stop=args.stop_distance)

            pose = ego.get_transform()
            command = None
            if args.control:
                # 参考轨迹取当前车道往前 30 m 的中心线，按 2 m 采样
                path = []
                waypoint = carla_map.get_waypoint(pose.location, project_to_road=True)
                travelled = 0.0
                while waypoint is not None and travelled < 30.0:
                    location = waypoint.transform.location
                    path.append((location.x, location.y))
                    next_waypoints = waypoint.next(2.0)
                    if not next_waypoints:
                        break
                    waypoint = next_waypoints[0]
                    travelled += 2.0

                delta = control.pure_pursuit(
                    (pose.location.x, pose.location.y, math.radians(pose.rotation.yaw)),
                    path, lookahead=args.lookahead)
                steer = control.normalized_steer(delta)

                # 速度控制：远了给油门，近了踩刹车；目标速度按速度系数缩放
                speed = math.sqrt(ego.get_velocity().x ** 2 + ego.get_velocity().y ** 2)
                target = args.max_speed * factor
                error = target - speed
                # 末尾补 0.0：误差正好为 0 时 np.clip 会保留 -0.0，
                # 打印出来是 "刹车 -0.00"，看着像 bug，也容易让人以为控制量越界。
                throttle = float(np.clip(error / max(args.max_speed, 1e-3), 0.0, 1.0)) + 0.0
                brake = float(np.clip(-error / max(args.max_speed, 1e-3), 0.0, 1.0)) + 0.0
                command = carla_module.VehicleControl(throttle=throttle, brake=brake,
                                                      steer=steer)
                ego.apply_control(command)
                if brake > 0.1:
                    stats['brake_frames'] += 1

                # 横向偏差：自车到车道中心线的距离，越小说明循迹越好
                lateral = pose.location.distance(
                    carla_map.get_waypoint(pose.location, project_to_road=True)
                    .transform.location)
                stats['lateral'].append(float(lateral))

                # 真实最小间距：命令行里能直接看出有没有真的撞上
                nearest = min(
                    (pose.location.distance(other.get_location())
                     for other in world.get_actors().filter('vehicle.*')
                     if other.id != ego.id),
                    default=float('nan'))
                if nearest == nearest:
                    stats['gap_true'].append(float(nearest))
            stats['free'].append(float(free))

            print('[演示] 帧 %3d 点数 %5d 可行驶比例 %.2f 前方余量 %5.1f m 速度系数 %.2f%s'
                  % (frame, points.shape[0], ratio, free, factor,
                     '' if command is None else
                     ' 油门 %.2f 刹车 %.2f 转向 %+.2f' % (command.throttle, command.brake,
                                                         command.steer)))

            if record_fig is not None:
                # 录制：每帧清空重画，标题里带上本帧的关键指标，
                # 合成 gif 后能直接看出"哪里是路面、哪里在减速"。
                record_fig.clf()
                record_ax = record_fig.add_subplot(111, projection='3d')
                for cls_id in range(L.N_CLASSES):
                    mask = classes == cls_id
                    if mask.any():
                        record_ax.scatter(points[mask, 0], points[mask, 1], points[mask, 2],
                                          s=2, c=[colors[cls_id]],
                                          label=L.CLASS_NAMES[cls_id], depthshade=False)
                record_ax.set_xlim(0, 40)
                record_ax.set_ylim(-15, 15)
                record_ax.set_zlim(-3, 6)
                record_ax.set_xlabel('x / m', fontsize=8)
                record_ax.set_ylabel('y / m', fontsize=8)
                record_ax.set_zlabel('z / m', fontsize=8)
                record_ax.tick_params(labelsize=7)
                record_ax.view_init(elev=18, azim=-68)
                record_ax.set_title('frame %d   free %.1f m   factor %.2f   brake %.2f'
                                    % (frame, free, factor,
                                       0.0 if command is None else command.brake),
                                    fontsize=9)
                record_ax.legend(loc='upper right', markerscale=4, fontsize=7)
                record_fig.savefig(os.path.join(args.record, 'frame_%04d.png' % frame))
                continue

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
        if record_fig is not None:
            import matplotlib.pyplot as plt
            plt.close(record_fig)
        _print_demo_summary(stats, args)
        return 0
    finally:
        cc.teardown(client, world, sensors, actors)