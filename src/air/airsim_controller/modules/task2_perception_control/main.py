"""任务2 入口：传感器感知 + 运动控制。

用法：
    python -m modules.task2_perception_control.main --mode avoid   # 深度相机神经网络避障
    python -m modules.task2_perception_control.main --mode track   # 神经网络轨迹跟踪
ROS：
    roslaunch airsim_controller task2_avoid.launch
    roslaunch airsim_controller task2_track.launch
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from common.drone_client import DroneClient            # noqa: E402
from modules.task2_perception_control.avoider import DepthAvoider  # noqa: E402
from modules.task2_perception_control.tracker import (         # noqa: E402
    NeuralTracker, circle_trajectory, square_trajectory)


def main():
    p = argparse.ArgumentParser(description="任务2：感知与轨迹控制")
    p.add_argument("--mode", choices=["avoid", "track"], default="avoid")
    p.add_argument("--seconds", type=float, default=30.0)
    p.add_argument("--v-fwd", type=float, default=1.5)
    p.add_argument("--v-max", type=float, default=2.0)
    p.add_argument("--traj", choices=["circle", "square"], default="circle")
    p.add_argument("--interval", type=float, default=0.1)
    args = p.parse_args()

    client = DroneClient(interval=args.interval)
    if args.mode == "avoid":
        DepthAvoider(client, v_fwd=args.v_fwd).run(seconds=args.seconds)
    else:
        wps = circle_trajectory() if args.traj == "circle" else square_trajectory()
        NeuralTracker(client, waypoints=wps, v_max=args.v_max).run(seconds=args.seconds)


if __name__ == "__main__":
    main()
