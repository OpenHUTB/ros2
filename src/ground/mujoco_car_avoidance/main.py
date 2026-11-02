#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""mujoco_car_avoidance 统一入口。

用法（满足仓库"入口以 main 开头、必须支持 launch"的约定）：
    python3 main.py --demo                  # 打开视窗，自动避障演示
    python3 main.py --interactive           # 打开视窗，键盘 WASD 手动驾驶（避障仍然生效）
    python3 main.py --headless              # 无窗口，跑完整闭环并导出曲线/CSV（推荐在虚拟机上先跑这个）
    python3 main.py --test                  # 自检：纯逻辑单测 + 场景冒烟测试
    python3 main.py --launch                # 通过 ros2 launch 一键启动 ROS 2 节点
    bash main.sh --headless                 # 同上，脚本入口

退出码约定（遵循仓库 #140/#143 的修复口径：失败必须返回非 0）：
    0 成功；1 运行失败或自检未通过；2 参数错误
"""

import argparse
import shutil
import subprocess
import sys
import time
from pathlib import Path

# 允许"在功能包根目录直接 python3 main.py"与"colcon 安装后 ros2 run"两种方式
sys.path.insert(0, str(Path(__file__).resolve().parent))

from mujoco_car_avoidance import __version__, config          # noqa: E402
from mujoco_car_avoidance.car_env import CarEnv, MujocoUnavailable, describe_environment  # noqa: E402
from mujoco_car_avoidance.controller import RampParams      # noqa: E402
from mujoco_car_avoidance.decision import DecisionParams  # noqa: E402
from mujoco_car_avoidance.runner import AvoidanceRunner     # noqa: E402

PACKAGE_ROOT = Path(__file__).resolve().parent
VERSION_LINE = "mujoco_car_avoidance {}".format(__version__)


# ----------------------------------------------------------------------------
# 命令行
# ----------------------------------------------------------------------------
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="main.py",
        description="MuJoCo 地面两轮小车：射线感知 + 分级减速避障",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--demo", action="store_true", help="视窗 + 自动避障演示")
    mode.add_argument("--interactive", action="store_true", help="视窗 + 键盘手动驾驶")
    mode.add_argument("--headless", action="store_true", help="无窗口运行并导出结果")
    mode.add_argument("--test", action="store_true", help="运行自检")
    mode.add_argument("--launch", action="store_true", help="调用 ros2 launch 启动节点")

    parser.add_argument("--scene", default=None, help="场景 XML 路径（默认使用包内 models/）")
    parser.add_argument("--duration", type=float, default=config.SIM_DURATION,
                        help="仿真时长上限 (s)")
    parser.add_argument("--control-hz", type=float, default=config.CONTROL_HZ,
                        help="决策/控制频率 (Hz)")
    parser.add_argument("--speed", type=float, default=DecisionParams.cruise_speed,
                        help="巡航速度 (m/s)")
    parser.add_argument("--slow-start", type=float, default=DecisionParams.slow_start_m,
                        help="开始减速的距离 (m)")
    parser.add_argument("--stop-target", type=float, default=DecisionParams.stop_target_m,
                        help="期望停车位置：障碍物前该距离处速度归零 (m)")
    parser.add_argument("--emergency", type=float, default=DecisionParams.emergency_m,
                        help="急刹距离 (m)")
    parser.add_argument("--stale-timeout", type=float, default=DecisionParams.stale_timeout_s,
                        help="感知失效超时 (s)")
    parser.add_argument("--acc-max", type=float, default=RampParams.acc_max,
                        help="最大加速度 (m/s^2)")
    parser.add_argument("--brake-max", type=float, default=RampParams.brake_max,
                        help="最大减速度 (m/s^2)")
    parser.add_argument("--settle", type=float, default=config.SETTLE_SECONDS,
                        help="起步静置时间 (s)：先零轮速推进，让接触建立后再进入闭环")
    parser.add_argument("--heading-gain", type=float, default=config.HEADING_HOLD_GAIN,
                        help="直行航向保持增益，0 表示关闭")
    parser.add_argument("--save-dir", default=str(config.DEFAULT_OUTPUT_DIR),
                        help="结果导出目录")
    parser.add_argument("--no-plot", action="store_true", help="不导出曲线图")
    parser.add_argument("--version", action="version", version=VERSION_LINE,
                        help="打印版本号后退出（用于确认虚拟机里是不是最新代码）")
    parser.add_argument("--logic-only", action="store_true",
                        help="配合 --test：只跑不依赖 MuJoCo 的纯逻辑测试")
    parser.add_argument("--verbose", action="store_true", help="打印逐帧日志")
    return parser


def build_params(args) -> DecisionParams:
    return DecisionParams(
        cruise_speed=args.speed,
        slow_start_m=args.slow_start,
        stop_target_m=args.stop_target,
        emergency_m=args.emergency,
        stale_timeout_s=args.stale_timeout,
    )


def print_step(info) -> None:
    flags = ""
    if info.held:
        flags += "  [沿用上次测距]"
    if info.collision:
        flags += "  [碰撞]"
    print("  [t={:5.1f}s] {:<10} d={:<8} v={:+.2f} m/s  cmd={:+.2f}  x={:6.2f} m  "
          "车头间距={:.2f} m{}".format(
              info.t, info.state.value,
              "N/A" if info.d_sensor is None else "{:.2f} m".format(info.d_sensor),
              info.v_actual, info.v_cmd, info.x, info.clearance, flags))


# ----------------------------------------------------------------------------
# 各模式
# ----------------------------------------------------------------------------
def run_headless(args) -> int:
    env, runner = make_runner(args)
    print("[就绪] {}".format(describe_environment()))
    runner.settle()
    print("[静置] 零轮速推进 {:.2f}s，接触已建立".format(args.settle))
    print("[运行] 无窗口模式，仿真时长上限 {:.1f}s，控制频率 {:.1f}Hz".format(
        args.duration, args.control_hz))

    def on_frame(info):
        if args.verbose:
            print_step(info)
        return True

    recorder = runner.run(args.duration, on_frame=on_frame)
    return report(args, env, runner, recorder)


def run_viewer(args, interactive: bool) -> int:
    env, runner = make_runner(args)
    teleop = None
    manual_source = None

    if interactive:
        from mujoco_car_avoidance.teleop import KeyboardTeleop

        teleop = KeyboardTeleop(forward_speed=min(args.speed, 1.2), turn_rate=1.0)
        teleop.open()
        if not teleop.available:
            print("[警告] 当前终端不支持非阻塞键盘读取，交互模式将按无按键处理；"
                  "请改用 --demo 或 --headless")
        manual_source = teleop.poll
        print("[提示] W/S 前进后退，A/D 转向，空格停车，Q 退出；避障限速始终生效")

    print("[就绪] {}".format(describe_environment()))
    runner.settle()
    print("[静置] 零轮速推进 {:.2f}s，接触已建立".format(args.settle))
    print("[运行] 视窗模式，按 Ctrl+C 或关闭窗口结束")

    recorder = runner.recorder
    wall_start = time.perf_counter()
    sim_start = env.time
    try:
        with env.launch_viewer() as viewer:
            next_deadline = 0.0
            last_log = -1.0

            while viewer.is_running() and (env.time - sim_start) < args.duration:
                manual = manual_source() if manual_source is not None else None
                info = runner.step_once(manual)
                viewer.sync()

                if info.t - last_log >= 1.0:
                    last_log = info.t
                    print_step(info)

                if teleop is not None and teleop.quit_requested:
                    break

                # 按墙钟节奏推进，使演示接近实时
                next_deadline += runner.dt
                remaining = next_deadline - (time.perf_counter() - wall_start)
                if remaining > 0:
                    time.sleep(remaining)
    except KeyboardInterrupt:
        print("\n[中断] 用户终止")
    except Exception as exc:
        print("[错误] 无法打开仿真视窗（{}）。无图形界面的环境请改用 --headless。".format(exc))
        return 1
    finally:
        if teleop is not None:
            teleop.close()

    wall_elapsed = max(time.perf_counter() - wall_start, 1e-9)
    runner.realtime_factor = (env.time - sim_start) / wall_elapsed
    return report(args, env, runner, recorder)


def run_test(args) -> int:
    import unittest

    print("[自检] 1/2 纯逻辑测试（不需要 MuJoCo）")
    test_dir = PACKAGE_ROOT / "test"
    loader = unittest.TestLoader()
    suite = loader.discover(str(test_dir), pattern="test_*.py")
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    ok = result.wasSuccessful()

    if args.logic_only:
        print("[自检] 已跳过场景冒烟测试（--logic-only）")
        return 0 if ok else 1

    print("[自检] 2/2 场景冒烟测试（需要 MuJoCo）")
    try:
        ok = scene_smoke_test(args) and ok
    except MujocoUnavailable as exc:
        print("[自检失败] {}".format(exc))
        return 1
    print("[自检] 结果：{}".format("通过" if ok else "未通过"))
    return 0 if ok else 1


def scene_smoke_test(args) -> bool:
    """最小场景级验证：射线不打自车、车能动、闭环能停住且不碰撞。"""
    env, runner = make_runner(args)
    runner.settle()
    checks = []

    # 1) 静止时射线不应把自车几何体当成障碍物
    distance = env.sensed_distance()
    ok = distance is not None and distance > 3.0
    checks.append(("静止时本车道最近距离 > 3 m（射线未打到自车）", ok,
                   "实测 {}".format("无命中" if distance is None else "{:.2f} m".format(distance))))

    # 2) 给正速度指令，车应当向前运动
    start_x = env.state().x
    env.set_wheel_speeds(10.0, 10.0)
    env.step(int(1.0 / env.timestep))   # 1 s
    moved = env.state().x - start_x
    checks.append(("给定轮速后 1 s 内前进 > 0.1 m", moved > 0.1,
                   "实测 {:.3f} m".format(moved)))

    # 3) 完整闭环：车必须真的开到障碍物附近并停住
    #    注意这里刻意断言"推进量"，而不是只断言"没碰撞"——
    #    早期版本只要求 c_停车 > 0，结果"车原地不动"这种失败也能通过（假 PASS）。
    env, runner = make_runner(args)
    runner.settle()
    recorder = runner.run(args.duration)
    summary = recorder.summary()
    advanced = env.state().x
    stop_clearance = summary["c_停车(m)"]
    # 若本次没有检测到"停稳时刻"（例如还没真正起步），退化为用最小间距判定
    effective = stop_clearance if stop_clearance is not None else summary["c_最小(m)"]
    stopped_close = effective is not None and 0.8 <= effective <= 3.5
    ok = (not summary["是否碰撞"]) and advanced >= 3.0 and stopped_close
    checks.append((
        "闭环：推进 >= 3 m 且停在障碍物前 0.8~3.5 m",
        ok,
        "推进={:.2f} m，{}={} m（c_停车={} m，c_最小={} m），碰撞={}".format(
            advanced,
            "c_停车" if stop_clearance is not None else "c_最小",
            None if effective is None else round(effective, 2),
            None if stop_clearance is None else round(stop_clearance, 2),
            None if summary["c_最小(m)"] is None else round(summary["c_最小(m)"], 2),
            summary["是否碰撞"])))

    print("\n  场景冒烟测试明细：")
    for name, passed, detail in checks:
        print("    [{}] {} —— {}".format("PASS" if passed else "FAIL", name, detail))
    return all(passed for _, passed, _ in checks)


def run_launch(args) -> int:
    if shutil.which("ros2") is None:
        print("[错误] 未找到 ros2 命令。请先 source ROS 2 环境：\n"
              "  source /opt/ros/humble/setup.bash\n"
              "并确认已 colcon build 本功能包。")
        return 1
    command = ["ros2", "launch", "mujoco_car_avoidance", "main.launch.py"]
    print("[启动] {}".format(" ".join(command)))
    # 用 subprocess.run 而不是 os.system：真实退出码能透出到外部（仓库 #140/#143 的修复口径）
    completed = subprocess.run(command, check=False)
    return int(completed.returncode)


# ----------------------------------------------------------------------------
# 公共
# ----------------------------------------------------------------------------
def make_runner(args):
    scene_path = config.resolve_scene_path(args.scene)
    if not Path(scene_path).is_file():
        raise MujocoUnavailable("找不到场景文件：{}".format(scene_path))

    env = CarEnv(scene_path,
                 timestep=config.TIMESTEP,
                 control_hz=args.control_hz)
    runner = AvoidanceRunner(
        env,
        params=build_params(args),
        ramp_params=RampParams(acc_max=args.acc_max, brake_max=args.brake_max),
        control_hz=args.control_hz,
        settle_seconds=args.settle,
        heading_hold_gain=args.heading_gain,
    )
    print("[场景] {}".format(scene_path))
    return env, runner


def report(args, env, runner, recorder) -> int:
    save_dir = Path(args.save_dir).expanduser()
    csv_path = save_dir / "run_log.csv"
    png_path = save_dir / "run_curves.png"

    recorder.write_csv(csv_path)
    plotted = False if args.no_plot else recorder.plot(png_path)

    print("\n[指标] 性能评价")
    print(recorder.format_summary())
    print("  {:<22}{:.2f}".format("实时率(sim/wall)", runner.realtime_factor))
    if env.state().upright < 0.8:
        print("  [警告] 车体姿态异常（up_z={:.2f}），请检查轮胎摩擦与减速度设置".format(
            env.state().upright))
    print("\n[导出] {}".format(csv_path))
    if plotted:
        print("[导出] {}".format(png_path))
    elif not args.no_plot:
        print("[导出] 未生成曲线图（缺少 matplotlib，可 pip install matplotlib）")

    summary = recorder.summary()
    advanced = env.state().x
    stop_clearance = summary["c_停车(m)"]
    effective = stop_clearance if stop_clearance is not None else summary["c_最小(m)"]
    if summary["是否碰撞"]:
        print("[结果] 失败：发生了碰撞")
        return 1
    if advanced < 3.0:
        print("[结果] 失败：车辆几乎没有前进（推进 {:.2f} m），请检查传感器与决策链路".format(advanced))
        return 1
    if (effective is None) or (effective <= 0.0):
        print("[结果] 失败：未能停在障碍物之前")
        return 1
    if stop_clearance is None:
        print("[结果] 成功：最小间距 {:.2f} m（未检测到停稳时刻，已用最小间距判定）".format(effective))
    else:
        print("[结果] 成功：停在障碍物前 {:.2f} m 处".format(stop_clearance))
    return 0


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    # 每次运行都打印版本：排查问题时第一眼就能确认虚拟机里是不是最新代码
    print("[版本] {}".format(VERSION_LINE))

    try:
        if args.test:
            return run_test(args)
        if args.launch:
            return run_launch(args)
        if args.headless:
            return run_headless(args)
        if args.demo:
            return run_viewer(args, interactive=False)
        if args.interactive:
            return run_viewer(args, interactive=True)
    except MujocoUnavailable as exc:
        print("[错误] {}".format(exc))
        return 1
    except ValueError as exc:
        print("[参数错误] {}".format(exc))
        return 2

    build_parser().print_help()
    print("\n请选择一个模式：--demo / --interactive / --headless / --test / --launch")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
