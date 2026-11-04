#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""uav_mujoco_sim 统一入口：argparse 解析各子命令并分发到对应层。

仿真侧需 Python ≥3.10（MuJoCo 3.12 下限），ROS 侧 rclpy 仅有 3.8 绑定，
故子命令延迟导入，两套依赖互不干扰。
"""
from __future__ import annotations

import argparse

SCENES = ["open_ground", "obstacle_field", "corridor", "gate_track", "urban_block"]


# --------------------------------------------------------------------------- #
def _add_scene_args(p):
    p.add_argument("--scene", default="open_ground", choices=SCENES,
                   help="场景（默认 open_ground）")
    p.add_argument("--seed", type=int, default=0, help="场景随机种子")


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        prog="main.py",
        description="uav_mujoco_sim —— MuJoCo 四旋翼仿真与 ROS2 桥接底座",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--server", action="store_true", help="启动 MuJoCo 仿真服务端")
    g.add_argument("--bridge", action="store_true", help="启动 ROS2 桥接节点")
    g.add_argument("--keyboard", action="store_true", help="启动键盘遥控节点（ROS）")
    g.add_argument("--demo", action="store_true", help="独立演示：仿真+键盘，不用 ROS")
    g.add_argument("--selftest", action="store_true", help="运行内置自检")
    g.add_argument("--record", metavar="OUT.npz", help="录制一次飞行到 npz")
    g.add_argument("--replay", metavar="IN.npz", help="回放录制的飞行数据")

    _add_scene_args(ap)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=41460)
    ap.add_argument("--dt", type=float, default=0.01)
    ap.add_argument("--substeps", type=int, default=4)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--headless", action="store_true", help="不渲染视觉传感器")
    ap.add_argument("--quality", choices=["fast", "full"], default="fast",
                    help="fast=关阴影，感知用（快 6 倍）；full=全开，录演示动图用")
    ap.add_argument("--imu-noise", action="store_true")
    ap.add_argument("--lidar-azimuth", type=int, default=72)
    ap.add_argument("--lidar-max-range", type=float, default=15.0)
    ap.add_argument("--seconds", type=float, default=15.0,
                    help="录制/演示时长（秒）")
    ap.add_argument("--viewer", action="store_true",
                    help="演示时打开 MuJoCo 交互窗口（需要显示器）")
    ap.add_argument("--verbose", "-v", action="store_true")
    return ap


# --------------------------------------------------------------------------- #
def cmd_server(a) -> int:
    from uav_mujoco_sim.core.sensors import CameraConfig, LidarConfig
    from uav_mujoco_sim.sim.server import serve, DEFAULT_PORT

    cam = CameraConfig(width=a.width, height=a.height, quality=a.quality)
    lid = LidarConfig(n_azimuth=a.lidar_azimuth, max_range=a.lidar_max_range)
    serve(a.host, a.port if a.port else DEFAULT_PORT, scene=a.scene, seed=a.seed,
          dt=a.dt, substeps=a.substeps, camera=cam, lidar=lid,
          imu_noise=a.imu_noise, headless=a.headless)
    return 0


def cmd_bridge(a) -> int:
    from uav_mujoco_sim.ros.bridge_node import main as bridge_main
    return bridge_main()


def cmd_keyboard(a) -> int:
    from uav_mujoco_sim.ros.keyboard_node import main as kb_main
    return kb_main()


def cmd_demo(a) -> int:
    """独立演示：本进程直接跑仿真 + 终端键盘，不经过 ROS。"""
    from uav_mujoco_sim.demo import run_demo
    return run_demo(scene=a.scene, seed=a.seed, dt=a.dt, substeps=a.substeps,
                    seconds=a.seconds, viewer=a.viewer, quality=a.quality,
                    verbose=a.verbose)


def cmd_selftest(a) -> int:
    from uav_mujoco_sim.selftest import run_selftest
    return run_selftest(scene=a.scene, seed=a.seed, verbose=a.verbose)


def cmd_record(a) -> int:
    from uav_mujoco_sim.demo import record_flight
    return record_flight(a.record, scene=a.scene, seed=a.seed, dt=a.dt,
                         substeps=a.substeps, seconds=a.seconds, verbose=True)


def cmd_replay(a) -> int:
    from uav_mujoco_sim.demo import replay_flight
    return replay_flight(a.replay)


# --------------------------------------------------------------------------- #
def main(argv=None) -> int:
    a = build_parser().parse_args(argv)
    if a.server:
        return cmd_server(a)
    if a.bridge:
        return cmd_bridge(a)
    if a.keyboard:
        return cmd_keyboard(a)
    if a.demo:
        return cmd_demo(a)
    if a.selftest:
        return cmd_selftest(a)
    if a.record:
        return cmd_record(a)
    if a.replay:
        return cmd_replay(a)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
