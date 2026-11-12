# -*- coding: utf-8 -*-
"""交互查看器：单独进程开 MuJoCo 窗口，自带同一套仿真与规划。

本进程不是 ROS 节点，因此不读 ROS 参数；通过命令行参数跟随 launch 的取值：

    passage_viewer --width 0.86 --variant mppi_footprint
"""

import argparse

import mujoco.viewer

import humanoid_passage_width_sweep.config as config
from humanoid_passage_width_sweep.sim import PassageSim


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description="MuJoCo 窄通道通行查看器")
    ap.add_argument("--width", type=float, default=config.DEFAULT_WIDTH,
                    help="窄门净宽 (m)")
    ap.add_argument("--variant", type=str, default="mppi_footprint",
                    help="规划器变体：mppi_footprint / mppi_point / reactive_footprint / reactive_point")
    a = ap.parse_args(argv)

    sim = PassageSim(width=a.width, variant=a.variant)
    print(f"窗口已打开（width={a.width}, variant={a.variant}）；关闭窗口结束。")
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
