# -*- coding: utf-8 -*-
"""交互查看器：单独一个进程开 MuJoCo 窗口，内部自带同样的一套仿真与控制。

与 ``hopper_slip_node`` 相互独立——两者各自推进自己的物理，不共享状态。
用于在没有 ROS 话题对接需求时直接看弹跳效果（需要有图形环境）。

本进程不是 ROS 节点，因此不读 ROS 参数；它通过命令行参数跟随 launch 的取值：

    hopper_viewer --target-height 0.62 --control-hz 100
"""

import argparse

import mujoco.viewer

import mujoco_hopper_slip_control.config as config
from mujoco_hopper_slip_control.sim import HopperSim


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description="MuJoCo 单足弹跳查看器")
    ap.add_argument("--target-height", type=float, default=config.TARGET_HEIGHT,
                    help="躯干顶点目标高度 (m)")
    ap.add_argument("--target-speed", type=float, default=config.TARGET_SPEED,
                    help="前进速度目标 (m/s)")
    a = ap.parse_args(argv)

    sim = HopperSim(target_height=a.target_height, target_speed=a.target_speed)
    print(f"窗口已打开（target_height={a.target_height}）；关闭窗口结束。")
    try:
        with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
            while viewer.is_running():
                sim.step()
                viewer.sync()
    except KeyboardInterrupt:
        pass
    finally:
        sim.close()


if __name__ == "__main__":
    main()
