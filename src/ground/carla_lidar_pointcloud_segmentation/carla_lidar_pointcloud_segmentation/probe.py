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
1. 生成自车，正前方 10 m 放一辆已知的 BMW，右侧 6 m 放一个已知行人；
2. 挂语义激光雷达读一帧；
3. 把点云从传感器坐标系转到世界坐标系；
4. 打印标签直方图，以及"自车附近 / 前方车辆附近 / 行人附近"分别是什么标签。

前两个是已知答案，标签对得上，映射表才可信。
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


def _spawn_reference_actors(world, ego):
    """放两个"已知答案"：正前方一辆车，右侧一个行人。"""
    library = world.get_blueprint_library()
    transform = ego.get_transform()
    yaw = math.radians(transform.rotation.yaw)
    actors = {}

    ahead = transform.location
    ahead_transform = world.get_map().get_spawn_points()[0]
    ahead_transform.location.x = ahead.x + 10.0 * math.cos(yaw)
    ahead_transform.location.y = ahead.y + 10.0 * math.sin(yaw)
    ahead_transform.location.z = ahead.z
    ahead_transform.rotation = transform.rotation
    car = world.try_spawn_actor(library.filter('vehicle.bmw.grandtourer')[0], ahead_transform)
    if car is not None:
        actors['前方车辆'] = car

    walker_transform = world.get_map().get_spawn_points()[0]
    walker_transform.location.x = ahead.x
    walker_transform.location.y = ahead.y + 6.0
    walker_transform.location.z = ahead.z + 1.0
    walker_transform.rotation = transform.rotation
    walker = world.try_spawn_actor(library.filter('walker.pedestrian.*')[0], walker_transform)
    if walker is not None:
        actors['右侧行人'] = walker
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

        others = _spawn_reference_actors(world, ego)
        if not others:
            print('[体检] 参考目标没生成成功，出生点可能太挤，换 --spawn-index')
            return 1

        # 让刚生成的车和人先进入场景，否则这一帧里可能还没有它们
        for _ in range(10):
            world.tick()

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

        points, tags = cc.points_from_semantic(frame)
        sensor_transform = sensor.get_transform()
        world_points = cc.to_world(points, sensor_transform)

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

        origin = np.array([sensor_transform.location.x, sensor_transform.location.y])
        near_ego = np.linalg.norm(world_points[:, :2] - origin, axis=1) < 3.0
        print('\n自车附近（3 m 内）：%s（应为车辆）' % _tag_ranking(tags, near_ego))

        for name, actor in others.items():
            centre = actor.get_location()
            target = np.array([centre.x, centre.y])
            radius = 3.0 if 'vehicle' in actor.type_id else 1.2
            mask = np.linalg.norm(world_points[:, :2] - target, axis=1) < radius
            expect = '车辆' if 'vehicle' in actor.type_id else '行人'
            print('%s（%s，半径 %.1f m）：%s（应为%s）'
                  % (name, actor.type_id, radius, _tag_ranking(tags, mask), expect))

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