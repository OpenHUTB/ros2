#!/usr/bin/env python3
"""模块统一入口。

    python3 scripts/main.py            # 无头跑 10 s 仿真，打印指标并出曲线图（默认）
    python3 scripts/main.py --gui      # 起 ROS 节点 + MuJoCo viewer（需要 rospy 和图形界面）
    python3 scripts/main.py --render   # 重新生成演示动图（需要离屏渲染，Linux 上 MUJOCO_GL=egl）

其余参数会原样透传给被转调的脚本，例如 --render --out /tmp/impedance_img。
"""
import os
import runpy
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def main(argv):
    args = [a for a in argv[1:] if a not in ("--gui", "--render")]
    if "--gui" in argv:
        target = "mujoco_arm_impedance_node.py"
    elif "--render" in argv:
        target = "render_demo.py"
    else:
        target = "headless_test.py"

    script = os.path.join(HERE, target)
    sys.argv = [script] + args
    runpy.run_path(script, run_name="__main__")


if __name__ == "__main__":
    main(sys.argv)
