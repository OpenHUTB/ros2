#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""多机连接自检（客户机侧运行，无需 roscore）。

把“编队起不来”拆成几段分别判定：

  1. 读参数：默认读 config/params.yaml 的 airsim 段（与主节点用的是同一套参数）；
  2. 列载具：问模拟器要载具列表，确认 settings.json 里的三架都真的起来了；
  3. 逐个 RPC：每架发一次 getMultirotorState，确认 RPC 应答而不是只有端口通；
  4. 逐架起降（可选，--no-fly 关闭）：依次起飞、悬停、降落，验证能真正控制。

退出码：全部通过为 0，任何一项不通过为 1。
"""

import argparse
import os
import sys
import time

PARAMS_DEFAULT = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), os.pardir, "config", "params.yaml")


def load_params(path):
    path = os.path.abspath(path)
    if not os.path.isfile(path):
        return {}, "参数文件不存在：%s" % path
    try:
        import yaml
    except ImportError:
        return {}, "没有装 PyYAML，无法读取 %s" % path
    with open(path, "r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh) or {}
    return data.get("airsim") or {}, None


def main(argv=None):
    ap = argparse.ArgumentParser(description="编队实验的多机连接自检")
    ap.add_argument("--params", default=PARAMS_DEFAULT)
    ap.add_argument("--ip", default=None)
    ap.add_argument("--timeout", type=float, default=None)
    ap.add_argument("--no-fly", action="store_true",
                    help="只列载具并发 RPC，不做起降")
    args = ap.parse_args(argv)

    params, warning = load_params(args.params)
    ip = args.ip or params.get("ip") or "192.168.237.1"
    timeout = args.timeout or float(params.get("timeout") or 30.0)
    leader = params.get("leader") or "SimpleFlight"
    followers = list(params.get("followers") or ["SimpleFlight2", "SimpleFlight3"])
    expected = [leader] + followers

    print("=" * 62)
    print("编队实验 · 多机连接自检")
    print("=" * 62)
    print("[1/4] 目标 %s    RPC 超时 %.1f 秒    期望载具 %s" % (ip, timeout, expected))
    if warning:
        print("      提示：%s" % warning)

    try:
        import airsim
    except Exception as exc:                                    # noqa: BLE001
        print("[2/4] 无法导入 airsim 客户端：%s" % exc)
        print("      → 客户机安装顺序：pip3 install numpy → msgpack-rpc-python → airsim")
        return 1

    client = airsim.MultirotorClient(ip=ip, timeout_value=timeout)
    client.confirmConnection()
    print("      airsim 客户端版本：%s" % getattr(airsim, "__version__", "未知"))

    names = client.listVehicles()
    names = getattr(names, "vehicle_names", names)
    names = [str(n) for n in names]
    print("[2/4] 模拟器载具列表：%s" % names)
    missing = [n for n in expected if n not in names]
    if missing:
        print("      → 缺少 %s：检查宿主 settings.json 的 Vehicles 段，"
              "改完必须重启模拟器" % missing)
        return 1
    print("      全部载具就位")

    print("[3/4] 逐架 RPC")
    for name in expected:
        state = client.getMultirotorState(vehicle_name=name)
        pos = state.kinematics_estimated.position
        print("      %-15s NED (%.2f, %.2f, %.2f)"
              % (name, pos.x_val, pos.y_val, pos.z_val))

    if args.no_fly:
        print("[4/4] 跳过起降（--no-fly）")
        print("=" * 62)
        return 0

    print("[4/4] 逐架起飞、悬停、降落（依次执行，不会相撞）")
    failed = []
    for name in expected:
        try:
            client.enableApiControl(True, vehicle_name=name)
            client.armDisarm(True, vehicle_name=name)
            client.takeoffAsync(timeout_sec=20, vehicle_name=name).join()
            time.sleep(2.0)
            client.landAsync(timeout_sec=30, vehicle_name=name).join()
            client.armDisarm(False, vehicle_name=name)
            client.enableApiControl(False, vehicle_name=name)
            print("      %-15s 起飞并降落正常" % name)
        except Exception as exc:                                # noqa: BLE001
            print("      %-15s 失败：%s" % (name, exc))
            failed.append(name)
    print("-" * 62)
    if failed:
        print("自检未通过：%s" % failed)
        print("=" * 62)
        return 1
    print("多机自检通过，可以运行 roslaunch uav_formation_flight main.launch")
    print("=" * 62)
    return 0


if __name__ == "__main__":
    sys.exit(main())
