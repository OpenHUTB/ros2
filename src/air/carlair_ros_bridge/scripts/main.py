#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""模块入口与环境自检（main.* 入口约定）.

检查四项：airsim 包可用性、仿真器连通性、位姿读取、相机取帧、激光雷达点云。
全部通过返回 0，任一失败返回 1，便于 CI/launch 判定。

用法:
    rosrun carlair_ros_bridge main.py
    rosrun carlair_ros_bridge main.py _sim/host:=192.168.94.1
    roslaunch carlair_ros_bridge main.launch
"""
from __future__ import annotations

import os
import sys

import rospy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sim_client import self_check  # noqa: E402


def main():
    # 自检节点不依赖 ROS master 也能跑（方便直接 python3 main.py 排查环境）
    if not rospy.core.is_initialized():
        try:
            rospy.init_node("uav_bridge_selfcheck", anonymous=True, disable_signals=True)
        except Exception:  # noqa: BLE001 - 没有 roscore 时退化为纯 Python 自检
            pass

    host = rospy.get_param("sim/host", os.environ.get("AIRSIM_HOST", "127.0.0.1"))
    port = int(rospy.get_param("sim/airsim_port", 41451))
    vehicle = rospy.get_param("sim/vehicle_name", "")

    print("==> carlair_ros_bridge 环境自检")
    print("    目标仿真器: %s:%d (vehicle='%s')" % (host, port, vehicle))

    results = self_check(host, port, vehicle)
    passed = 0
    for name, ok, detail in results:
        flag = "通过" if ok else "失败"
        print("    [%s] %s%s" % (flag, name, ("  -- " + detail) if detail else ""))
        passed += 1 if ok else 0

    total = len(results)
    print("自检结果: %d/%d 通过" % (passed, total))
    if passed != total:
        print("排查建议:")
        print("    1) Windows 侧是否已启动: ./CarlaAir.sh Town10HD （等待 'Ready! Both servers are running.'）")
        print("    2) 宿主机地址是否正确（VMware NAT 下通常为 192.168.94.1，可用 ping 验证）")
        print("    3) sensors 配置是否已放到 Windows 的 ~/Documents/AirSim/settings.json")
        print("    4) 相机/雷达名称须与 settings.json 中的键一致（front_rgb / front_depth / lidar1）")
        sys.exit(1)
    print("环境自检全部通过")
    sys.exit(0)


if __name__ == "__main__":
    main()
