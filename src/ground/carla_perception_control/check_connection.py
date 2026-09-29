#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CARLA 连通性诊断：分步定位"连不上/超时"出在哪一环。

用法:
    python3.10 check_connection.py [host] [port] [town]

依次检查并报告每步耗时：
    1. carla 模块是否可导入
    2. TCP 端口是否可达（不需要 carla）
    3. carla.Client 能否建立连接（get_world，瞬时）
    4. 服务端当前地图与同步模式
    5. load_world 能否完成（重操作，实测本机 7 s 以上）
    6. 世界内 actor 数量（残留传感器/车辆会拖慢后续操作）
"""
import math
import socket
import sys
import time


def step(n, desc):
    print(f"\n[{n}] {desc}")


def main():
    host = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 2000
    town = sys.argv[3] if len(sys.argv) > 3 else "Town05"

    print("=" * 64)
    print(f"  CARLA 连通性诊断  host={host} port={port} town={town}")
    print("=" * 64)

    # 1) 模块
    step(1, "导入 carla 模块")
    try:
        import carla
        print("    OK：carla 模块可导入")
    except ImportError as e:
        print(f"    失败：{e}")
        print("    → 需安装 CARLA 0.9.16 客户端 wheel（cp310/cp311/cp312）")
        return 2

    # 2) TCP 可达性（不依赖 carla，能区分"网络不通"与"服务端卡住"）
    step(2, f"TCP 连接 {host}:{port}")
    t0 = time.time()
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(5.0)
    try:
        s.connect((host, port))
        print(f"    OK：TCP 可达（{time.time() - t0:.2f}s）")
    except Exception as e:
        print(f"    失败：{e}")
        print("    → 网络层不通。依次检查：")
        print("      1. 宿主机 CARLA 服务端是否已启动")
        print(f"      2. 宿主机能否 ping 通（虚拟机执行: ping {host}）")
        print("      3. 宿主机防火墙是否放行 2000 端口（Windows Defender 防火墙）")
        print("      4. host 是否填了宿主机的 VMware 网卡地址（VMnet8）")
        return 3
    finally:
        s.close()

    # 3) carla 客户端瞬时连接
    step(3, "carla.Client 建立连接并 get_world（瞬时操作）")
    t0 = time.time()
    try:
        client = carla.Client(host, port)
        client.set_timeout(30.0)
        world = client.get_world()
        dt = time.time() - t0
        print(f"    OK：get_world 成功（{dt:.2f}s）")
        if dt > 10:
            print(f"    ⚠ 偏慢（{dt:.1f}s）：服务端负载较高，建议加大超时")
    except Exception as e:
        print(f"    失败：{e}")
        print("    → 端口开着但 CARLA 未正常响应，可能服务端仍在初始化或已卡死，"
              "建议重启服务端")
        return 4

    # 4) 当前状态
    step(4, "服务端当前状态")
    try:
        cur_map = world.get_map().name
        st = world.get_settings()
        print(f"    当前地图  : {cur_map}")
        print(f"    同步模式  : {st.synchronous_mode}"
              f"（fixed_delta_seconds={st.fixed_delta_seconds}）")
    except Exception as e:
        print(f"    读取失败：{e}")

    # 5) load_world（重操作，最易超时）
    step(5, f"load_world('{town}')（重操作，实测本机需 7 s 以上）")
    already = cur_map == town or cur_map.endswith("/" + town)
    if already:
        print(f"    跳过：服务端已在该地图（本模块会自动跳过，不再重载）")
    else:
        t0 = time.time()
        try:
            client.set_timeout(120.0)
            client.load_world(town)
            print(f"    OK：加载完成（{time.time() - t0:.1f}s）")
        except Exception as e:
            print(f"    失败（{time.time() - t0:.1f}s）：{e}")
            print("    → 这是最耗时的一步。可改用服务端**已加载**的地图"
                  "（--town 传当前地图名），即可跳过重载")

    # 6) actor 统计
    step(6, "世界内 actor 统计（残留会拖慢后续操作）")
    try:
        actors = world.get_actors()
        by = {}
        for a in actors:
            k = a.type_id.split(".")[0]
            by[k] = by.get(k, 0) + 1
        print(f"    总数 {len(actors)}：{by}")
        if len(actors) > 60:
            print("    ⚠ actor 偏多（早期版本传感器被 GC 回收后会残留），"
                  "建议重启服务端或 load_world 清理")
    except Exception as e:
        print(f"    统计失败：{e}")

    print("\n" + "=" * 64)
    print("  诊断完成：上面各步均 OK 即可运行 main.py --mode run")
    print("=" * 64)
    return 0


if __name__ == "__main__":
    sys.exit(main())
