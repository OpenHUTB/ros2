#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""passability_check 统一入口：车辆几何通过性核算。

一次性算清三件事：
  1. 这台车的接近角、离去角、纵向通过角（由轴距、前/后悬、离地高度决定）
  2. 这条剖面（地库坡道 / 山路过顶 / 减速带）最陡的下凹、上凸折角
  3. 把车按真实位姿沿剖面走一遍，看车头、车底、车尾到底会不会蹭，蹭多深

用法：
    python3 main.py                       # 默认车型跑全部场景（报告 + 剖面图）
    python3 main.py --vehicle suv         # 换车型
    python3 main.py --vehicle all --matrix  # 所有车型 × 所有场景的对比表
    python3 main.py --scenario garage_ramp --vehicle suv
    python3 main.py --json out.json --csv out.csv
    python3 main.py --test                # 自检（不依赖 ROS / 仿真器）
    python3 main.py --list                # 列出场景
    python3 main.py --launch              # ros2 launch 一键启动
    python3 main.py --ros                 # 直接起 ROS 2 节点

退出码：
    0 运行成功（核算结论是 FAILED 也算运行成功）
    1 自检未通过或运行异常
    2 参数错误
    3 加了 --strict 且核算结论为 FAILED
"""

import argparse
import io
import json
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

from passability_check import (  # noqa: E402
    __version__,
    VehicleError,
    ScenarioError,
    load_scenario,
    list_scenarios,
    get_vehicle,
    PRESET_VEHICLES,
    evaluate,
    format_report,
    render_profile,
    scenario_matrix_csv,
    to_json_payload,
    all_passed,
    verdict,
    VERDICT_FAILED,
    VERDICT_PASS_WITH_NOTE,
)


def _force_utf8():
    """Windows 控制台默认 cp936，报告里有 ° 和中文字符，统一转 UTF-8。"""
    for stream_name in ("stdout", "stderr"):
        stream = getattr(sys, stream_name, None)
        reconfigure = getattr(stream, "reconfigure", None)
        if reconfigure is not None:
            try:
                reconfigure(encoding="utf-8", errors="replace")
            except (ValueError, OSError):
                pass


def _write_text(path, text):
    """按 UTF-8 写文本，自动建目录。"""
    directory = os.path.dirname(os.path.abspath(path))
    if directory and not os.path.isdir(directory):
        os.makedirs(directory)
    with io.open(path, "w", encoding="utf-8", newline="\n") as handle:
        handle.write(text)
    print("已写入 %s" % path)


def _select_vehicles(name):
    if name == "all":
        return [PRESET_VEHICLES[key] for key in sorted(PRESET_VEHICLES)]
    return [get_vehicle(name)]


def _select_scenarios(args):
    if args.scenario:
        return [load_scenario(args.scenario, directory=args.scenario_dir)]
    if args.scenario_dir:
        names = sorted(os.listdir(args.scenario_dir))
        return [load_scenario(name, directory=args.scenario_dir)
                for name in names if name.endswith(".json")]
    paths = list_scenarios()
    return [load_scenario(path) for path in paths]


def cmd_list(args):
    print("场景目录：%s" % (args.scenario_dir or os.path.dirname(list_scenarios()[0])))
    for path in list_scenarios(args.scenario_dir):
        scenario = load_scenario(path)
        print("  %-16s %s" % (scenario.name, scenario.title))
    print("车型：%s" % ", ".join(sorted(PRESET_VEHICLES)))
    return 0


def cmd_run(args):
    scenarios = _select_scenarios(args)
    vehicles = _select_vehicles(args.vehicle)
    if not scenarios:
        print("没有可用的场景", file=sys.stderr)
        return 2

    payloads = []
    csv_rows = []
    failed = []
    noted = []
    for scenario in scenarios:
        for vehicle in vehicles:
            report, assertions = evaluate(vehicle, scenario,
                                          samples=args.samples, steps=args.steps)
            payload = to_json_payload(vehicle, scenario, report, assertions)
            payload["figure"] = render_profile(scenario.terrain, vehicle, report["worst"]["pose"],
                                               worst=report["worst"],
                                               width=args.width, height=args.height)
            payloads.append(payload)
            csv_rows.append({
                "scenario": scenario.name,
                "vehicle": vehicle.name,
                "approach_deg": "%.2f" % vehicle.approach_deg,
                "departure_deg": "%.2f" % vehicle.departure_deg,
                "breakover_deg": "%.2f" % vehicle.breakover_deg,
                "min_clearance_m": "%.3f" % vehicle.min_clearance,
                "max_depth_m": "%.3f" % report["max_depth"],
                "min_margin_m": "%.3f" % report["min_margin"],
                "result": payload["result"],
            })
            # 结论只看数值判据：角度判据不通过时是 PASS_WITH_NOTE，不算未通过
            if verdict(assertions) == VERDICT_FAILED:
                failed.append((scenario.name, vehicle.name))
            elif verdict(assertions) == VERDICT_PASS_WITH_NOTE:
                noted.append((scenario.name, vehicle.name))

            if not args.quiet:
                header = "通过性核算 v%s（纯几何，不依赖仿真器）" % __version__
                print(format_report(vehicle, scenario, report, assertions, header))
                if not args.no_figure:
                    print()
                    print(payload["figure"])
                print()

    if args.matrix or args.quiet:
        print("对比表（结论：PASSED = 全部断言通过；"
              "PASS_WITH_NOTE = 数值复核能过、角度判据偏保守；FAILED = 数值判据不通过）")
        print(scenario_matrix_csv(csv_rows))
        print()

    if args.json:
        _write_text(args.json, json.dumps(payloads, ensure_ascii=False, indent=2))
    if args.csv:
        _write_text(args.csv, scenario_matrix_csv(csv_rows))

    if noted:
        print("共 %d 组能过但需注意（角度判据偏保守，见备注）：%s"
              % (len(noted), "，".join("%s/%s" % item for item in noted)))
    if failed:
        print("共 %d 组核算未通过：%s"
              % (len(failed), "，".join("%s/%s" % item for item in failed)))
        if args.strict:
            return 3
    elif not noted:
        print("共 %d 组核算全部通过" % len(payloads))
    return 0


def cmd_self_test(args):
    """自检：只跑纯逻辑，不需要 ROS、仿真器、显示器。"""
    from passability_check import VehicleSpec, Terrain, seat, scan_profile

    passed = []
    failed = []

    def check(name, condition, detail=""):
        if condition:
            passed.append(name)
        else:
            failed.append("%s %s" % (name, detail))

    # 1. 角度
    sedan = get_vehicle("sedan")
    check("接近角 9.04°", abs(sedan.approach_deg - 9.0395) < 1e-3, "%.4f" % sedan.approach_deg)
    check("离去角 11.89°", abs(sedan.departure_deg - 11.8887) < 1e-3,
          "%.4f" % sedan.departure_deg)
    check("纵向通过角 11.85°", abs(sedan.breakover_deg - 11.8412) < 1e-3,
          "%.4f" % sedan.breakover_deg)
    check("车长 4.53 m", abs(sedan.length - 4.53) < 1e-9, "%.3f" % sedan.length)

    # 2. 几何：弦长 == 轴距，坡度正确
    terrain = Terrain([[0.0, 0.0], [3.0, 0.0], [9.0, 1.2], [12.0, 1.2]])
    pose = seat(terrain, sedan, 5.0)
    check("弦长等于轴距", abs(pose["chord"] - sedan.wheelbase) < 1e-6, "%.6f" % pose["chord"])
    check("坡道坡度 11.31°", abs(abs(terrain.slope_deg(6.0)) - 11.3099) < 1e-3,
          "%.4f" % terrain.slope_deg(6.0))
    check("下凹折角 11.31°", abs(terrain.max_concave_deg() - 11.3099) < 1e-3)
    check("上凸折角 11.31°", abs(terrain.max_convex_deg() - 11.3099) < 1e-3)

    # 3. 数值扫描：平地不托底，轿车在地库坡底托底
    flat = Terrain([[0.0, 0.0], [20.0, 0.0]])
    flat_scan = scan_profile(flat, sedan, steps=120)
    check("平地最大侵入 0", flat_scan["max_depth"] <= 1e-9, flat_scan["max_depth"])
    slope_scan = scan_profile(terrain, sedan, steps=240)
    check("地库坡道轿车托底", slope_scan["max_depth"] > 0.01, slope_scan["max_depth"])
    suv_scan = scan_profile(terrain, get_vehicle("suv"), steps=240)
    check("地库坡道 SUV 不托底", suv_scan["max_depth"] <= 1e-9, suv_scan["max_depth"])

    # 4. 参数校验
    try:
        VehicleSpec(wheelbase=-1.0)
    except VehicleError:
        check("负轴距被拒绝", True)
    else:
        check("负轴距被拒绝", False, "没有抛异常")
    try:
        VehicleSpec(mid_ratio=1.5)
    except VehicleError:
        check("mid_ratio 越界被拒绝", True)
    else:
        check("mid_ratio 越界被拒绝", False, "没有抛异常")

    # 5. 场景文件
    scenarios = [load_scenario(path) for path in list_scenarios()]
    check("场景文件 >= 3 个", len(scenarios) >= 3, len(scenarios))
    for scenario in scenarios:
        report, assertions = evaluate(sedan, scenario, samples=31, steps=160)
        check("场景 %s 有 5 条断言" % scenario.name, len(assertions) == 5, len(assertions))
        check("场景 %s 有结论" % scenario.name, report["steps"] > 0)

    print("自检：%d 项通过 / %d 项失败" % (len(passed), len(failed)))
    for item in failed:
        print("  [FAIL] %s" % item)
    if failed:
        return 1
    print("自检全部通过")
    return 0


def cmd_launch(args):
    exe = shutil.which("ros2")
    if exe is None:
        print("没有找到 ros2 命令。先 source ROS 2 环境，再走 launch：\n"
              "  source /opt/ros/humble/setup.bash\n"
              "  ros2 launch passability_check main.launch.py", file=sys.stderr)
        return 2
    command = [exe, "launch", "passability_check", "main.launch.py"]
    if args.vehicle:
        command.append("vehicle:=%s" % args.vehicle)
    if args.scenario:
        command.append("scenario:=%s" % args.scenario)
    print("执行：%s" % " ".join(command))
    return subprocess.call(command)


def cmd_ros(args):
    try:
        from passability_check.node import main as node_main
    except ImportError as error:
        print("没有找到 rclpy（%s）。本模块的核算部分不需要 ROS，可直接跑：\n"
              "  python3 main.py --vehicle suv --scenario garage_ramp" % error, file=sys.stderr)
        return 2
    return node_main()


def build_parser():
    parser = argparse.ArgumentParser(
        description="车辆几何通过性核算：接近角 / 离去角 / 纵向通过角 / 托底",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--vehicle", default="sedan",
                        help="车型：sedan / suv / all")
    parser.add_argument("--scenario", default=None,
                        help="场景名或 .json 路径；留空则跑全部场景")
    parser.add_argument("--scenario-dir", default=None, help="场景目录")
    parser.add_argument("--samples", type=int, default=41, help="车身下缘采样点数")
    parser.add_argument("--steps", type=int, default=480, help="沿剖面扫描的位姿档数")
    parser.add_argument("--width", type=int, default=92, help="字符图宽度")
    parser.add_argument("--height", type=int, default=20, help="字符图高度")
    parser.add_argument("--json", default=None, help="把结果写成 JSON")
    parser.add_argument("--csv", default=None, help="把对比表写成 CSV")
    parser.add_argument("--matrix", action="store_true", help="额外打印对比表")
    parser.add_argument("--no-figure", action="store_true", help="不画字符剖面图")
    parser.add_argument("--quiet", action="store_true", help="只打印对比表")
    parser.add_argument("--strict", action="store_true", help="结论 FAILED 时返回退出码 3")
    parser.add_argument("--test", action="store_true", help="跑自检")
    parser.add_argument("--list", action="store_true", help="列出场景与车型")
    parser.add_argument("--launch", action="store_true", help="ros2 launch 一键启动")
    parser.add_argument("--ros", action="store_true", help="直接起 ROS 2 节点")
    parser.add_argument("--version", action="store_true", help="打印版本")
    return parser


def main(argv=None):
    _force_utf8()
    parser = build_parser()
    args = parser.parse_args(argv)

    if args.version:
        print("passability_check %s" % __version__)
        return 0
    if args.list:
        return cmd_list(args)
    if args.test:
        return cmd_self_test(args)
    if args.launch:
        return cmd_launch(args)
    if args.ros:
        return cmd_ros(args)

    try:
        return cmd_run(args)
    except (VehicleError, ScenarioError) as error:
        print("参数错误：%s" % error, file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
