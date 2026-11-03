#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""诊断脚本：检查「车辆能不能正常开动」以及「哪些出生点可用」。

为什么需要它：如果自车在某个出生点被墙/其他车辆挡住，就会出现
"油门已经给到 1.0，但车速始终接近 0" 的现象，跟车数据就会完全失真。
本脚本会对若干个出生点逐个试验：原地全油门加速 5 秒，看能跑多远。

用法：
    python3 diagnose_drive.py --host 192.168.148.1 --town Town03
    python3 diagnose_drive.py --host 192.168.148.1 --indices 0,1,2,3,4,5,6,7
    python3 diagnose_drive.py --host 192.168.148.1 --clean      # 先清掉地图里残留的车辆
"""

import argparse
import math
import sys

import carla


def speed_of(actor):
    v = actor.get_velocity()
    return math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)


def clean_world(world):
    """销毁地图里所有车辆/行人/传感器（用于清掉上一次运行残留的 actor）。"""
    actors = (world.get_actors().filter('vehicle.*')
              + world.get_actors().filter('walker.*')
              + world.get_actors().filter('sensor.*'))
    print('发现残留 actor %d 个，正在清理...' % len(actors))
    for actor in actors:
        try:
            actor.destroy()
        except Exception:
            pass
    print('清理完成，剩余 vehicle.*：%d 个'
          % len(world.get_actors().filter('vehicle.*')))


def main():
    parser = argparse.ArgumentParser(description='CARLA 车辆可行驶性诊断')
    parser.add_argument('--host', default='192.168.148.1', help='Windows 宿主机 IP')
    parser.add_argument('--port', type=int, default=2000)
    parser.add_argument('--town', default=None, help='如 Town03')
    parser.add_argument('--indices', default='0,1,2,3,4,5',
                        help='要测试的出生点序号，逗号分隔')
    parser.add_argument('--steps', type=int, default=100, help='每个点全油门推进的帧数（100 帧 = 5 秒）')
    parser.add_argument('--throttle', type=float, default=1.0)
    parser.add_argument('--clean', action='store_true',
                        help='测试前先清掉地图里所有残留车辆/行人/传感器')
    args = parser.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(60.0)
    if args.town:
        print('正在加载地图 %s（首次较慢）...' % args.town)
        client.load_world(args.town)

    world = client.get_world()
    print('地图：%s' % world.get_map().name)

    original_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    if args.clean:
        clean_world(world)

    print('地图中当前车辆数：%d' % len(world.get_actors().filter('vehicle.*')))

    spawn_points = world.get_map().get_spawn_points()
    print('出生点总数：%d，本次测试：%s'
          % (len(spawn_points), args.indices))
    print('每个点原地全油门 %.1f 秒（throttle=%.2f）\n' % (args.steps * 0.05, args.throttle))

    results = []
    try:
        for text in args.indices.split(','):
            text = text.strip()
            if not text:
                continue
            index = int(text)
            if index >= len(spawn_points):
                print('#%-3d 超出范围，跳过' % index)
                continue

            spawn_point = spawn_points[index]
            blueprint = world.get_blueprint_library().find('vehicle.tesla.model3')
            vehicle = world.try_spawn_actor(blueprint, spawn_point)
            if vehicle is None:
                print('#%-3d 位置(%7.1f, %7.1f) -> 出生点被占用，跳过'
                      % (index, spawn_point.location.x, spawn_point.location.y))
                continue

            start = vehicle.get_location()
            try:
                for _ in range(args.steps):
                    vehicle.apply_control(
                        carla.VehicleControl(throttle=args.throttle, steer=0.0, brake=0.0))
                    world.tick()
                displacement = vehicle.get_location().distance(start)
                final_speed = speed_of(vehicle)
                verdict = '✅ 可以正常行驶' if displacement > 15.0 else '❌ 开不动（被挡住/出生点异常）'
                print('#%-3d 位置(%7.1f, %7.1f) yaw=%6.1f -> %.1f 秒位移 %6.1f m，'
                      '末速度 %5.2f m/s  %s'
                      % (index, spawn_point.location.x, spawn_point.location.y,
                         spawn_point.rotation.yaw, args.steps * 0.05,
                         displacement, final_speed, verdict))
                results.append((index, displacement, final_speed))
            finally:
                vehicle.destroy()
    finally:
        world.apply_settings(original_settings)
        print('\n[清理] 世界设置已恢复（异步模式）')

    if results:
        best = max(results, key=lambda item: item[1])
        print('\n结论：可正常行驶的出生点 %d 个；推荐出生点 #%d（位移 %.1f m，末速 %.2f m/s）'
              % (len([r for r in results if r[1] > 15.0]), best[0], best[1], best[2]))
        print('       把推荐序号告诉我，我把它固化到模块里作为默认出生点。')
    else:
        print('\n结论：没有可用出生点，请检查 CARLA 地图与版本。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
