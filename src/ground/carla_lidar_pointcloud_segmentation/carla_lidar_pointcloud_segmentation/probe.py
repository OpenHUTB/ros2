#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""标签体检：实测语义激光雷达的 ``object_tag`` 编号。

为什么要有这个模式
------------------
CARLA 文档说 ``object_tag`` 等于 ``carla.CityObjectLabel``（路面 7、车辆 11、
行人 4），但不同打包版本的编号并不一致。本项目实测的 CARLA 0.9.16 里，
路面是 1、车辆是 14、行人是 12。按文档的编号写映射表，会把建筑当成行人，
训练集从一开始就是错的。

所以映射表按实测写，并且留一个能随时重跑的体检模式：换地图、换 CARLA 版本、
换打包方式之后，先跑 ``--probe`` 核对这张表。

怎么做
------
1. 在自车前方 8 / 12 / 16 m 放三辆已知的宝马，在侧前方放三个已知行人；
2. 挂语义激光雷达读一帧；
3. 按 ``object_idx``（射线撞到的 actor 编号）筛出只属于这些已知目标的点；
4. 打印这些点的标签，同时打印整帧的标签直方图。

**为什么按 object_idx 筛，而不是按距离筛**：按距离筛会被遮挡骗——目标在建筑后面
时统计为空，看不出是"没这个标签"还是"没看见"；位置稍有偏差还会把旁边的东西算进来。
``object_idx`` 直接给出"这个点打在谁身上"，被挡住就诚实地报"本帧没有它的点"。

**为什么一次放多个、且用自车朝向算位置**：单个位置可能正好在建筑或护栏背后，
射线打不到，体检就会误报"没有这个目标"；而 ``waypoint.next(9.0)`` 在实测里
给出的位置离自车有 19 m，距离语义并不可靠。用自车朝向直接算坐标，一次铺几个，
只要有一个可见，标签就能定下来。
"""

import math
import queue
import time

import numpy as np

from . import carla_common as cc
from . import labels as L


def _tag_ranking(tags, mask, limit=5):
    """某一区域出现最多的若干标签，返回 ``[(tag, 点数), ...]``。"""
    if not mask.any():
        return []
    counts = np.bincount(tags[mask].astype(np.int64))
    pairs = [(int(tag), int(n)) for tag, n in enumerate(counts) if n > 0]
    pairs.sort(key=lambda item: -item[1])
    return pairs[:limit]


def _forward_offset(ego, forward, right):
    """按自车朝向算一个相对位置，不依赖 waypoint 的距离语义。"""
    transform = ego.get_transform()
    heading = transform.get_forward_vector()
    side = transform.get_right_vector()
    return cc.carla.Location(
        x=transform.location.x + heading.x * forward + side.x * right,
        y=transform.location.y + heading.y * forward + side.y * right,
        z=transform.location.z + 0.4)


def _spawn_reference_actors(world, ego):
    """在自车前后放几个"已知答案"，总有能被雷达看到的。

    为什么一次放多个：单个位置可能正好在建筑或护栏后面，射线打不到，
    体检就会误报"没有这个目标"。多铺几个，只要有一个可见，标签就能定下来。
    位置用自车朝向直接算，不走 waypoint —— ``next(9.0)`` 在实测里给出的
    位置离自车有 19 m，距离语义不可靠。
    """
    library = world.get_blueprint_library()
    car_bp = library.filter('vehicle.bmw.grandtourer')[0]
    walker_bp = library.filter('walker.pedestrian.*')[0]
    rotation = ego.get_transform().rotation
    actors = {}

    for forward in (8.0, 12.0, 16.0):
        car = world.try_spawn_actor(
            car_bp, cc.carla.Transform(_forward_offset(ego, forward, 0.0), rotation))
        if car is not None:
            actors['参考车辆%.0fm' % forward] = car

    for forward, right in ((6.0, 3.0), (10.0, -3.0), (14.0, 3.5)):
        walker = world.try_spawn_actor(
            walker_bp, cc.carla.Transform(_forward_offset(ego, forward, right), rotation))
        if walker is not None:
            actors['参考行人%.0fm%+.1fm' % (forward, right)] = walker

    if not actors:
        print('[体检] 参考目标一个都没放下去，出生点太挤，换 --spawn-index 重试')
    return actors


def run_probe(args):
    """实测标签编号，打印对照表。"""
    client, world = cc.connect(args.host, args.port, args.town, sync=True, dt=args.dt)
    cc.enable_sync_traffic_manager(client)
    sensors = []
    ego = None
    others = {}
    try:
        ego = cc.spawn_ego(world, spawn_index=args.spawn_index, autopilot=False)

        # 必须先 tick 一次再读自车位姿：CARLA 刚 spawn 的 actor 在下一帧之前
        # 读到的 transform 是全零，照它算相对位置会把目标扔到世界原点附近。
        for _ in range(3):
            world.tick()

        others = _spawn_reference_actors(world, ego)

        # 让刚生成的车和人先进入场景，否则这一帧里可能还没有它们
        for _ in range(10):
            world.tick()

        # 诊断：先确认目标真的落在预期位置上，再谈标签
        ego_transform = ego.get_transform()
        print('[诊断] 自车位置 (%.1f, %.1f, %.1f)，朝向 %.1f 度'
              % (ego_transform.location.x, ego_transform.location.y,
                 ego_transform.location.z, ego_transform.rotation.yaw))
        for name, actor in others.items():
            location = actor.get_location()
            print('[诊断] %s 位置 (%.1f, %.1f, %.1f)，距自车 %.1f m，'
                  '离地高度 %.1f m'
                  % (name, location.x, location.y, location.z,
                     location.distance(ego_transform.location),
                     location.z - ego_transform.location.z))

        blueprint = cc.lidar_blueprint(world, semantic=True, lidar_range=args.lidar_range,
                                       channels=args.channels,
                                       points_per_second=args.points_per_second,
                                       rotation_frequency=args.rotation)
        sensor, data_queue = cc.attach_lidar(world, ego, blueprint, height=args.lidar_height)
        sensors.append(sensor)

        # 空跑三帧，丢掉不满一圈的残帧
        for _ in range(3):
            world.tick()
            try:
                data_queue.get(timeout=5.0)
            except queue.Empty:
                pass

        frame = None
        started = time.time()
        while time.time() - started < 10.0:
            world.tick()
            try:
                frame = data_queue.get(timeout=1.0)
                break
            except queue.Empty:
                continue
        if frame is None:
            print('[体检] 10 秒内没收到语义点云')
            return 1

        print('[诊断] 点云帧号 %s，当前世界帧号 %s'
              % (getattr(frame, 'frame', '?'), world.get_snapshot().frame))
        points, tags, indices = cc.semantic_arrays(frame)

        print('=' * 60)
        print('标签体检：帧号 %s，点数 %d'
              % (getattr(frame, 'frame', '?'), len(points)))
        print('=' * 60)

        counts = np.bincount(tags.astype(np.int64), minlength=32)
        distance = np.hypot(points[:, 0], points[:, 1])
        print('\n%-6s %8s %10s %10s %16s %s'
              % ('标签', '点数', '最小距离', '平均距离', '高度范围', '实测含义'))
        print('-' * 88)
        for tag in np.argsort(-counts):
            if counts[tag] == 0:
                continue
            mask = tags == tag
            z = points[mask, 2]
            print('%-6d %8d %10.1f %10.1f %9.1f ~ %-6.1f %s'
                  % (tag, counts[tag], distance[mask].min(), distance[mask].mean(),
                     z.min(), z.max(), L.tag_description(int(tag))))

        # object_idx 直接记录射线撞到的 actor 编号，按它筛点比按距离筛可靠：
        # 目标被建筑挡住时会诚实报空，而不是悄悄统计旁边的东西。
        distinct = len(np.unique(indices[indices > 0]))
        print('\n这一帧里不同 actor 的个数：%d' % distinct)

        def report(name, actor, expect):
            mask = indices == actor.id
            if not mask.any():
                print('%s（%s，id %d）：本帧没有它的点，被遮挡或不在视野内'
                      % (name, actor.type_id, actor.id))
                return
            print('%s（%s，id %d）：%d 个点，标签 %s（应为%s）'
                  % (name, actor.type_id, actor.id, int(mask.sum()),
                     _tag_ranking(tags, mask), expect))

        report('自车', ego, '车辆')
        for name, actor in others.items():
            report(name, actor, '车辆' if 'vehicle' in actor.type_id else '行人')

        print('\n对照：官方 CityObjectLabel 里 Roads=7、Vehicles=11、Pedestrians=4。')
        print('本表实测值见 carla_lidar_pointcloud_segmentation/labels.py 的 MEASURED_TAG_TABLE。')
        return 0
    finally:
        for sensor in sensors:
            try:
                sensor.stop()
                sensor.destroy()
            except Exception:      # noqa: BLE001
                pass
        for actor in list(others.values()) + ([ego] if ego is not None else []):
            try:
                actor.destroy()
            except Exception:      # noqa: BLE001
                pass
        try:
            cc.restore_async(world)
        except Exception:      # noqa: BLE001
            pass
        print('[清理] 传感器与车辆已销毁，世界已恢复异步模式')