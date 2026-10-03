#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OpenHUTB（CarlaAir / hutb）场景道路探路工具

用于"沿道路飞行"任务的前置勘察，模拟器（hutb）运行时执行：

  1. 连接 CARLA 端口（默认 2000），打印客户端/服务端版本与地图名；
  2. 列出地图里的若干条道路（road_id、位置），并采样道路中心线；
  3. 连接 AirSim 端口（默认 41451），打印无人机当前 NED 位置；
  4. 把同一条道路的采样点用几种候选坐标变换换算到 AirSim 坐标，
     打印对照表；加 --fly 时把无人机飞到指定候选的起点，便于在模拟器
     窗口里肉眼确认"是否飞在那条道路上空"。

用法（在虚拟机里，先 source 好 ROS 与 catkin 工作空间）：

  python3 road_probe.py --carla-host 192.168.237.1 --airsim-host 192.168.237.1
  python3 road_probe.py --carla-host 192.168.237.1 --fly 1 --altitude 15

注意：CARLA 的 Python 客户端版本必须与服务端一致，例如服务端是 0.9.16 就
     pip3 install carla==0.9.16
"""

import argparse
import sys


def log(msg):
    print(msg, flush=True)


def next_waypoint(wp, distance):
    """CARLA 不同版本 next() 返回单个或列表，这里统一成单个"""
    nxt = wp.next(distance)
    if isinstance(nxt, list):
        return nxt[0] if nxt else None
    return nxt


def connect_carla(host, port):
    try:
        import carla
    except ImportError:
        log("× 没有安装 carla 客户端。请先安装与服务端一致的版本，例如：")
        log("    pip3 install carla==0.9.16")
        return None

    client = carla.Client(host, port)
    client.set_timeout(15.0)
    world = client.get_world()
    cmap = world.get_map()
    try:
        version = client.get_server_version()
    except Exception:
        version = "(无法获取，旧版本客户端)"
    log("√ 已连接 CARLA %s:%d  服务端版本 %s  地图 %s"
        % (host, port, version, cmap.name))
    return cmap


def sample_roads(cmap, step=5.0, max_len=400.0, want=5):
    """取前 want 条道路的中心线采样点：{road_id: [(x, y, z), ...]}"""
    roads = {}
    for wp_a, _wp_b in cmap.get_topology():
        rid = wp_a.road_id
        if rid in roads:
            continue
        pts = []
        wp = wp_a
        travelled = 0.0
        while wp is not None and travelled < max_len:
            loc = wp.transform.location
            pts.append((loc.x, loc.y, loc.z))
            wp = next_waypoint(wp, step)
            travelled += step
        if len(pts) >= 3:
            roads[rid] = pts
        if len(roads) >= want:
            break
    return roads


def connect_airsim(host, port, vehicle):
    try:
        import airsim
    except ImportError:
        log("× 没有安装 airsim 客户端：pip3 install msgpack-rpc-python airsim")
        return None

    client = airsim.MultirotorClient(ip=host, port=port)
    client.confirmConnection()
    pos = client.getMultirotorState(vehicle_name=vehicle).kinematics_estimated.position
    log("√ 已连接 AirSim %s:%d  载具 %s" % (host, port, vehicle))
    log("  当前 NED 位置 (x=北, y=东, z=下): (%.2f, %.2f, %.2f)"
        % (pos.x_val, pos.y_val, pos.z_val))
    return client


def candidates(carla_xyz):
    """CARLA 世界坐标 -> AirSim NED 的几种候选变换（含符号与轴交换）"""
    cx, cy, cz = carla_xyz
    return [
        ("1  x->x, y->y, z 取负", (cx, cy, -cz)),
        ("2  x<->y 交换, z 取负", (cy, cx, -cz)),
        ("3  x->x, y 取负, z 取负", (cx, -cy, -cz)),
        ("4  旋转 90: (x, y) -> (-y, x)", (-cy, cx, -cz)),
        ("5  旋转 -90: (x, y) -> (y, -x)", (cy, -cx, -cz)),
    ]


def fly_to(client, ned, altitude, vehicle, speed=5.0):
    """起飞并飞到指定 NED 位置，上方 altitude 米（NED 的 z 向下为负）"""
    x, y, _z = ned
    target_z = -abs(altitude)
    log("-> 起飞并飞往 NED (%.1f, %.1f, %.1f) ..." % (x, y, target_z))
    client.enableApiControl(True, vehicle_name=vehicle)
    client.armDisarm(True, vehicle_name=vehicle)
    client.takeoffAsync(vehicle_name=vehicle).join()
    client.moveToPositionAsync(x, y, target_z, speed, vehicle_name=vehicle).join()
    pos = client.getMultirotorState(vehicle_name=vehicle).kinematics_estimated.position
    log("   到达后实际 NED: (%.2f, %.2f, %.2f)"
        % (pos.x_val, pos.y_val, pos.z_val))
    log("   请在模拟器窗口里确认：无人机是否停在那条道路的上空？")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--carla-host", default="192.168.237.1")
    ap.add_argument("--carla-port", type=int, default=2000)
    ap.add_argument("--airsim-host", default="192.168.237.1")
    ap.add_argument("--airsim-port", type=int, default=41451)
    ap.add_argument("--vehicle", default="Drone1")
    ap.add_argument("--step", type=float, default=5.0, help="道路采样间隔（米）")
    ap.add_argument("--fly", type=int, default=0,
                    help="把无人机飞到第 N 个候选变换对应的起点（1~5，0=只打印）")
    ap.add_argument("--altitude", type=float, default=15.0, help="飞行高度（米）")
    args = ap.parse_args()

    log("=" * 62)
    log("OpenHUTB 场景道路探路")
    log("=" * 62)

    cmap = connect_carla(args.carla_host, args.carla_port)
    if cmap is None:
        return 1

    roads = sample_roads(cmap, step=args.step)
    if not roads:
        log("× 没有取到道路，检查地图是否载入完成")
        return 1

    log("")
    log("地图里的道路（各取前 400 米中心线）：")
    for rid, pts in roads.items():
        x0, y0, z0 = pts[0]
        x1, y1, z1 = pts[-1]
        log("  road_id=%-4d 采样点 %3d 个  起点 (%.1f, %.1f, %.1f)  "
            "终点 (%.1f, %.1f, %.1f)"
            % (rid, len(pts), x0, y0, z0, x1, y1, z1))

    first_road = next(iter(roads))
    start = roads[first_road][0]
    log("")
    log("第一条道路 road_id=%d 的起点（CARLA 世界坐标）: (%.1f, %.1f, %.1f)"
        % (first_road, start[0], start[1], start[2]))
    log("换算成 AirSim NED 的候选：")
    for i, (label, ned) in enumerate(candidates(start), start=1):
        log("  %d) %-32s -> NED (%.1f, %.1f, %.1f)"
            % (i, label, ned[0], ned[1], ned[2]))

    client = connect_airsim(args.airsim_host, args.airsim_port, args.vehicle)
    if client is None:
        log("（只做了 CARLA 侧的勘察）")
        return 0

    if args.fly:
        label, ned = candidates(start)[args.fly - 1]
        log("")
        log("按候选 %s 试飞：" % label)
        fly_to(client, ned, args.altitude, args.vehicle)
    else:
        log("")
        log("提示：加 --fly 1..5 可让无人机按某个候选换算飞过去，"
            "在模拟器窗口里肉眼核对哪一组变换是对的。")

    return 0


if __name__ == "__main__":
    sys.exit(main())
