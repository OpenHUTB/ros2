#!/usr/bin/env python3
"""humanoid_h1_walk 统一入口（课程要求入口以 main 开头）。

    python main.py --demo              # 无头跑一遍固定动作序列，打印各阶段指标
    python main.py --demo --robot h1_2 # 换机型
    python main.py --gui               # 打开 MuJoCo 窗口实时观看
    python main.py --test              # 无头自检（两机型，判定不摔倒且真的走了）
    python main.py --compare           # H1 / H1_2 步态指标对比
    python main.py --gif               # 录制动图
    python main.py --ros               # 用 ros2 launch 启动完整 ROS 2 链路

ROS 2 链路（``--ros``）包含三个节点：

    sim_node         MuJoCo 物理，发布 /joint_states、/odom、/humanoid/obs
    policy_node      TorchScript 策略，/humanoid/obs → /humanoid/joint_targets
    keyboard_teleop  方向键 → /cmd_vel（可关）
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.join(HERE, "scripts")
sys.path.insert(0, HERE)


def _run_script(name: str, extra: list[str]) -> int:
    return subprocess.call([sys.executable, os.path.join(SCRIPTS, name), *extra])


def demo(robot: str) -> int:
    """无头跑一遍固定动作序列，按阶段打印平均速度与骨盆高度。"""
    from humanoid_h1_walk import ROUTINE, Walker, load_specs, routine_command

    walker = Walker(robot, load_specs())
    print(f"机型 {robot}：{walker.spec.description}")
    print(f"控制周期 {walker.control_dt * 1e3:.0f} ms（{1 / walker.control_dt:.0f} Hz），"
          f"观测 {walker.spec.num_obs} 维，动作 {walker.spec.num_actions} 维\n")

    total = sum(seconds for _, seconds, _, _ in ROUTINE)
    label = None
    vx_sum, samples, height_min = 0.0, 0, 9.9
    while walker.data.time < total:
        cmd, new_label = routine_command(walker.data.time)
        if new_label != label:
            if label is not None:
                print(f"  {label:<12} 平均 vx {vx_sum / max(samples, 1):+.2f} m/s，"
                      f"最低骨盆 {height_min:.2f} m")
            label, vx_sum, samples, height_min = new_label, 0.0, 0, 9.9
        walker.step(cmd)
        vx_sum += float(walker.base_lin_vel_body()[0])
        samples += 1
        height_min = min(height_min, walker.base_height)

    print(f"  {label:<12} 平均 vx {vx_sum / max(samples, 1):+.2f} m/s，最低骨盆 {height_min:.2f} m")
    print(f"\n结束位置 x={walker.data.qpos[0]:+.2f} m, y={walker.data.qpos[1]:+.2f} m，"
          f"偏航 {walker.yaw:+.2f} rad，末态骨盆高度 {walker.base_height:.2f} m")
    return 0


def gui(robot: str, routine: bool) -> int:
    """打开 MuJoCo 窗口实时观看（方向键调速/转向）。"""
    import mujoco.viewer
    import numpy as np

    from humanoid_h1_walk import Walker, load_specs, routine_command

    walker = Walker(robot, load_specs())
    cmd = np.zeros(3, dtype=np.float32)
    KEY = {265: (0, +0.1), 264: (0, -0.1), 263: (2, +0.2), 262: (2, -0.2)}  # 上下左右

    def on_key(key: int) -> None:
        if routine or key not in KEY:
            return
        index, delta = KEY[key]
        cmd[index] = float(np.clip(cmd[index] + delta, -0.5 if index == 0 else -1.0,
                                   1.0))
        print(f"指令 vx {cmd[0]:+.1f} m/s, wz {cmd[2]:+.1f} rad/s")

    print(f"机型 {robot}。" + ("播放固定动作序列。" if routine else "方向键调速/转向，关窗口退出。"))
    with mujoco.viewer.launch_passive(walker.model, walker.data, key_callback=on_key) as viewer:
        while viewer.is_running():
            tick = time.perf_counter()
            if routine:
                cmd[:], _ = routine_command(walker.data.time)
            with viewer.lock():
                walker.step(cmd)
            viewer.sync()
            # Windows 的 sleep 精度约 1~15 ms，按控制周期补齐即可
            time.sleep(max(0.0, walker.control_dt - (time.perf_counter() - tick)))
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="humanoid_h1_walk 统一入口", formatter_class=argparse.RawDescriptionHelpFormatter
    )
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--demo", action="store_true", help="无头跑固定动作序列")
    mode.add_argument("--gui", action="store_true", help="打开 MuJoCo 窗口观看")
    mode.add_argument("--test", action="store_true", help="无头自检")
    mode.add_argument("--compare", action="store_true", help="机型步态指标对比")
    mode.add_argument("--gif", action="store_true", help="录制动图")
    mode.add_argument("--ros", action="store_true", help="ros2 launch 启动 ROS 2 链路")
    parser.add_argument("--robot", default="h1", choices=["h1", "h1_2"])
    parser.add_argument("--routine", action="store_true", help="--gui 下播放固定序列而不是键盘控制")
    args, rest = parser.parse_known_args()
    rest = [a for a in rest if a != "--"]

    if args.demo:
        return demo(args.robot)
    if args.gui:
        return gui(args.robot, args.routine)
    if args.test:
        return _run_script("smoke_walk.py", rest)
    if args.compare:
        return _run_script("compare_gaits.py", rest)
    if args.gif:
        return _run_script("record_gif.py", ["--robot", args.robot, *rest])
    if args.ros:
        return subprocess.call(
            ["ros2", "launch", "humanoid_h1_walk", "main.launch.py", f"robot:={args.robot}", *rest]
        )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
