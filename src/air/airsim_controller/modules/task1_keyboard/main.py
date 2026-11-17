"""任务1 入口：键盘遥控 AirSim 无人机。

直接运行：
    python -m modules.task1_keyboard.main            # 在仓库根目录执行
    python modules/task1_keyboard/main.py
ROS 启动：
    roslaunch airsim_controller task1_keyboard.launch
"""

import argparse
import os
import sys

# 允许直接脚本运行时把仓库根目录加入 sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from common.drone_client import DroneClient          # noqa: E402
from modules.task1_keyboard.teleop_keyboard import KeyboardTeleop  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description="任务1：键盘遥控无人机")
    parser.add_argument("--speed", type=float, default=1.0, help="线速度上限 m/s")
    parser.add_argument("--yaw-rate", type=float, default=30.0, help="偏航角速度 deg/s")
    parser.add_argument("--interval", type=float, default=0.1)
    parser.add_argument("--vehicle", type=str, default="")
    args = parser.parse_args()

    client = DroneClient(interval=args.interval, vehicle_name=args.vehicle)
    teleop = KeyboardTeleop(client, speed=args.speed, yaw_rate=args.yaw_rate)
    try:
        teleop.run()
    except KeyboardInterrupt:
        client.destroy()


if __name__ == "__main__":
    main()
