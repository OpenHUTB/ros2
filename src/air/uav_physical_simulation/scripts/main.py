#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""无人机物理仿真与键盘运动控制 —— 课程约定主入口。

对应课程任务（1）：对无人机进行物理仿真，通过键盘对无人机进行运动控制。

运行方式
--------
A. ROS 模式（推荐，满足「必须支持 launch 启动」要求）::

       # 终端 1：桥接（连仿真器 + 自动起飞悬停）
       roslaunch carlair_ros_bridge main.launch publish_image:=false

       # 终端 2：键盘控制
       roslaunch uav_physical_simulation main.launch

B. 独立模式（不经 ROS，直接调 AirSim RPC，用于快速验证）::

       python3 main.py --host 192.168.8.1

控制链路::

    按键 → key_to_cmd → geometry_msgs/Twist → /uav/cmd_vel
         → carlair_ros_bridge（坐标换算 + 执行）→ CarlaAir/AirSim

按键说明（机体系 x 前 / y 左 / z 上）::

    W / S      前进 / 后退      linear.x = ±horiz_speed
    A / D      左移 / 右移      linear.y = ±horiz_speed
    R / F      上升 / 下降      linear.z = ±vert_speed
    Q / E      右转 / 左转      angular.z = ±yaw_rate
    松开       悬停             全零
    ESC        退出
"""

from __future__ import annotations

import argparse
import os
import sys

# 允许从源码目录直接运行（catkin 的 devel/setup.bash 通常已处理包路径）
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from keyboard_control import key_to_cmd, publish_loop  # noqa: E402


# --------------------------------------------------------------------------
# 参数
# --------------------------------------------------------------------------

def build_parser():
    p = argparse.ArgumentParser(
        description="无人机物理仿真与键盘运动控制（课程任务 1）")
    p.add_argument("--horiz", type=float, default=None,
                   help="水平速度上限 (m/s)，默认 2.0")
    p.add_argument("--vert", type=float, default=None,
                   help="垂直速度上限 (m/s)，默认 1.0")
    p.add_argument("--yaw", type=float, default=None,
                   help="偏航角速度上限 (rad/s)，默认 1.0")
    p.add_argument("--dt", type=float, default=None,
                   help="控制周期 (s)，默认 0.1（须与桥接 cmd_duration 一致）")
    p.add_argument("--cmd_topic", type=str, default=None,
                   help="速度指令话题，默认 /uav/cmd_vel")
    p.add_argument("--host", type=str, default="127.0.0.1",
                   help="独立模式：仿真器 RPC 主机地址")
    p.add_argument("--port", type=int, default=41451,
                   help="独立模式：仿真器 RPC 端口（AirSim 默认 41451）")
    p.add_argument("--standalone", action="store_true",
                   help="强制走独立模式（不经 ROS）")
    return p


def resolve_params(args):
    """命令行 > ROS 参数 > 内置默认值。"""
    cfg = {
        "horiz": 2.0, "vert": 1.0, "yaw": 1.0,
        "dt": 0.1, "cmd_topic": "/uav/cmd_vel",
    }

    # ROS 参数服务器（若可用）
    try:
        import rospy
        if rospy.get_node_uri() is not None or rospy.core.is_initialized():
            for key, default in list(cfg.items()):
                cfg[key] = rospy.get_param("~" + key
                                           if key != "cmd_topic" else "~cmd_topic",
                                           default)
    except Exception:
        pass

    for key in ("horiz", "vert", "yaw", "dt", "cmd_topic"):
        val = getattr(args, key)
        if val is not None:
            cfg[key] = val
    return cfg


# --------------------------------------------------------------------------
# ROS 模式
# --------------------------------------------------------------------------

def run_ros_mode(cfg):
    import rospy
    from geometry_msgs.msg import Twist

    rospy.init_node("uav_keyboard_control", anonymous=False)

    pub = rospy.Publisher(cfg["cmd_topic"], Twist, queue_size=10)
    rospy.sleep(0.5)                     # 等 publisher 与桥接建立连接

    def log(msg):
        rospy.loginfo(msg)

    log("[就绪] 无人机键盘运动控制已启动")
    log("[就绪] 指令话题：{}  控制周期：{:.2f}s".format(cfg["cmd_topic"], cfg["dt"]))
    log("[就绪] 速度上限：水平 {:.2f} m/s，垂直 {:.2f} m/s，偏航 {:.2f} rad/s"
        .format(cfg["horiz"], cfg["vert"], cfg["yaw"]))
    log("[就绪] 控制键：W 前进 / S 后退 / A 左 / D 右 / "
        "R 升 / F 降 / Q 右转 / E 左转 / ESC 退出")
    log("[提示] 请确保桥接已启动且无人机已自动起飞悬停；松开按键即悬停。")

    publish_loop(
        pub=pub, twist_cls=Twist,
        horiz=cfg["horiz"], vert=cfg["vert"], yaw=cfg["yaw"],
        dt=cfg["dt"], stop_flag=rospy.is_shutdown, log=log,
    )
    log("[完成] 已退出并发布零速度指令（悬停）")


# --------------------------------------------------------------------------
# 独立模式（直接调 AirSim RPC，不经 ROS）
# --------------------------------------------------------------------------

def run_standalone_mode(cfg, args):
    """独立模式：直接连接仿真器，用 AirSim API 下发速度指令。

    注意：独立模式下必须自行处理使能 API 控制、解锁、起飞悬停这些
    通常由桥接完成的步骤。
    """
    try:
        import airsim
    except ImportError:
        print("[错误] 未安装 airsim 客户端。请执行：pip3 install airsim",
              file=sys.stderr)
        print("       或改用 ROS 模式（默认）：roslaunch uav_physical_simulation main.launch",
              file=sys.stderr)
        return 2

    client = airsim.MultirotorClient(ip=args.host, port=args.port)
    client.confirmConnection()
    print("[就绪] 已连接仿真器 {}:{}".format(args.host, args.port))

    client.enableApiControl(True)
    client.armDisarm(True)
    print("[就绪] API 控制已使能，正在自动起飞 …")
    client.takeoffAsync().join()
    client.hoverAsync().join()
    print("[就绪] 自动起飞完成，进入悬停")
    print("[就绪] 控制键：W 前进 / S 后退 / A 左 / D 右 / "
          "R 升 / F 降 / Q 右转 / E 左转 / ESC 退出")

    from keyboard_control import RawKeyReader, normalize_key, VALID_KEYS
    import time

    yaw_accum = 0.0
    try:
        with RawKeyReader() as reader:
            while True:
                t0 = time.time()
                held = set()
                ch = reader.get_key(timeout=cfg["dt"] * 0.5)
                if ch:
                    if ch == "\x1b":
                        break
                    k = normalize_key(ch)
                    if k in VALID_KEYS:
                        held = {k}

                vx, vy, vz, wz = key_to_cmd(
                    held, cfg["horiz"], cfg["vert"], cfg["yaw"])
                yaw_accum += wz * cfg["dt"]

                if vx or vy or vz or wz:
                    print("[控制] 按键={:<1} → vx={:+.2f} vy={:+.2f} vz={:+.2f} wz={:+.2f}"
                          .format("".join(sorted(held)) or "-", vx, vy, vz, wz))

                # AirSim：速度指令必须在机身系下给绝对速度，并显式带 yaw
                client.moveByVelocityZAsync(
                    vx, vy, -abs(vz) if vz else client.getMultirotorState()
                    .kinematics_estimated.position.z_val,
                    cfg["dt"], airsim.DroneYawMode(False, yaw_accum),
                )
                time.sleep(max(0.0, cfg["dt"] - (time.time() - t0)))
    except KeyboardInterrupt:
        print("\n[退出] 收到 Ctrl-C")
    finally:
        client.hoverAsync().join()
        client.armDisarm(False)
        client.enableApiControl(False)
        print("[完成] 已悬停、上锁并释放 API 控制")
    return 0


# --------------------------------------------------------------------------

def main():
    args = build_parser().parse_args()
    cfg = resolve_params(args)

    ros_available = False
    if not args.standalone:
        try:
            import rospy  # noqa: F401
            ros_available = True
        except ImportError:
            ros_available = False

    if ros_available:
        run_ros_mode(cfg)
        return 0
    return run_standalone_mode(cfg, args)


if __name__ == "__main__":
    sys.exit(main())
