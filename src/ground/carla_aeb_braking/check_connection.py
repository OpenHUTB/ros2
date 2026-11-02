#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""快速验证 CARLA 服务端连通性。

用法：
    python3 check_connection.py [host] [port]

输出服务器版本与当前地图名；连不上时给出常见排查建议。
"""
import sys

TIMEOUT = 10.0


def main():
    host = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 2000
    try:
        import carla
    except ImportError:
        print("未安装 carla 客户端：pip3 install <CARLA>/PythonAPI/carla/dist/carla-0.9.16-cp310-*.whl")
        return 1
    client = carla.Client(host, port)
    client.set_timeout(TIMEOUT)
    try:
        print("已连接，服务端版本:", client.get_server_version())
    except RuntimeError as exc:
        print(f"连接 {host}:{port} 失败：{exc}")
        print("排查：1) CARLA 服务端是否已启动（默认端口 2000）")
        print("      2) 跨机器时 host 填 CARLA 所在机器的 IP（虚拟机 NAT 网络填宿主机 IP）")
        print("      3) 参考 docs/set_up_and_connect_to_carla.md 的常见问题")
        return 1
    world = client.get_world()
    print("当前地图:", world.get_map().name)
    print("actor 数:", len(world.get_actors()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
