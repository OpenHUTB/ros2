#!/usr/bin/env python3
"""水下 ROV AprilTag 视觉定位与定点巡航仿真 - 模块主入口。

用法:
  python3 main.py                 # 启动完整仿真（Gazebo + 相机识别窗口 + RViz）
  python3 main.py --no-rviz       # 不启动 RViz
  python3 main.py --headless      # 无图形界面（仅 gzserver，用于无显示器环境）
"""
import argparse
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description="ROV AprilTag 视觉定位与定点巡航仿真")
    parser.add_argument("--no-rviz", action="store_true", help="不启动 RViz 可视化")
    parser.add_argument("--no-camera-gui", action="store_true", help="不弹出相机识别窗口")
    parser.add_argument("--headless", action="store_true", help="不启动 Gazebo 图形界面")
    args = parser.parse_args()

    launch_args = []
    if args.no_rviz or args.headless:
        launch_args.append("rviz:=false")
    if args.no_camera_gui or args.headless:
        launch_args.append("show_camera_gui:=false")
    if args.headless:
        launch_args.append("gui:=false")

    cmd = ["ros2", "launch", "rov_apriltag_cruise", "apriltag_cruise.launch.py"] + launch_args
    print("[INFO] 启动命令:", " ".join(cmd))
    return subprocess.call(cmd)


if __name__ == "__main__":
    sys.exit(main())
