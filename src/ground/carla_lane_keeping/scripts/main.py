#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""carla_lane_keeping 模块统一入口。

按照仓库约定，每个模块的入口以 `main` 开头。本入口负责：
  1. 解析模式参数（detect / control / both）
  2. 把配置加载进 ROS 参数服务器
  3. 以子进程方式拉起对应节点，统一管理生命周期

用法::

    # 只跑检测，在控制台看横向偏差
    rosrun carla_lane_keeping main.py --mode detect

    # 检测 + 车道保持闭环
    rosrun carla_lane_keeping main.py --mode both

    # 用 roslaunch 一键启动（推荐）
    roslaunch carla_lane_keeping main.launch
"""

import argparse
import os
import signal
import subprocess
import sys

import rospy

HERE = os.path.dirname(os.path.abspath(__file__))


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="车道线检测与车道保持模块入口",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--mode",
        choices=("detect", "control", "both"),
        default="detect",
        help="detect=只检测；control=只控制；both=检测+控制闭环",
    )
    parser.add_argument(
        "--image-topic",
        default="/carla/ego_vehicle/rgb_front/image",
        help="前视相机图像话题",
    )
    parser.add_argument(
        "--params",
        default=os.path.join(HERE, "..", "config", "lane_params.yaml"),
        help="YAML 参数文件路径",
    )
    parser.add_argument("--publish-debug", action="store_true", default=True,
                        help="是否发布鸟瞰掩码调试图像")
    return parser.parse_args(argv)


def build_env():
    """构造子进程环境：确保能 import 同目录下的 lane_detection.py。"""
    env = dict(os.environ)
    scripts_dir = HERE
    existing = env.get("PYTHONPATH", "")
    env["PYTHONPATH"] = scripts_dir + (os.pathsep + existing if existing else "")
    return env


def main(argv=None):
    args = parse_args(argv)

    # 入口本身不需要 master 也能解析参数，但 ROS 参数写入需要
    if not rospy.core.is_initialized():
        rospy.init_node("carla_lane_keeping_main", anonymous=True)

    # 把命令行参数落到 ROS 参数服务器，供子节点读取
    rospy.set_param("~image_topic", args.image_topic)
    rospy.set_param("~publish_debug", bool(args.publish_debug))

    # 载入 YAML 参数（若存在）。
    # 用 rosparam 命令行加载而不是自己解析 YAML：rosparam 是 ROS 自带工具，
    # 能正确处理嵌套命名空间与类型推断，不引入额外 Python 依赖。
    params_path = os.path.abspath(args.params)
    if os.path.isfile(params_path):
        rospy.loginfo("[main] 从 %s 载入参数", params_path)
        try:
            subprocess.check_call(
                ["rosparam", "load", params_path, "/lane_detector"],
                env=build_env(),
            )
        except Exception as exc:  # noqa: BLE001
            rospy.logwarn("[main] 参数加载失败（将使用代码默认值）：%s", exc)
    else:
        rospy.logwarn("[main] 未找到参数文件 %s，使用代码默认值", params_path)

    targets = []
    if args.mode in ("detect", "both"):
        targets.append("lane_detector.py")
    if args.mode in ("control", "both"):
        targets.append("lane_keeping_controller.py")

    env = build_env()
    procs = []
    for script in targets:
        path = os.path.join(HERE, script)
        rospy.loginfo("[main] 启动 %s", script)
        procs.append(subprocess.Popen([sys.executable, path], env=env))

    def shutdown(_signum=None, _frame=None):
        for proc in procs:
            if proc.poll() is None:
                proc.terminate()
        for proc in procs:
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    rospy.loginfo("[main] 已启动 %d 个节点，Ctrl+C 退出", len(procs))
    try:
        # 任一子进程退出则整体退出
        while not rospy.is_shutdown():
            for proc in procs:
                if proc.poll() is not None:
                    rospy.logwarn("[main] 子进程退出（码 %s），整体结束", proc.returncode)
                    shutdown()
                    return proc.returncode or 0
            rospy.sleep(0.2)
    except rospy.ROSInterruptException:
        pass
    finally:
        shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
