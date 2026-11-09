#!/usr/bin/env python3
"""
main.py —— 统一入口。
子命令：
  collect   采集专家巡航数据（纯 Python，无需 ROS）
  train     训练 MLP（纯 Python）
  test      闭环评估，打印 MAE/成功率（纯 Python）
  sim       启动 ROS 仿真节点
  teleop    启动 ROS 键盘控制节点
  cruise    启动 ROS MLP 巡航节点
  all       collect -> train -> test 全链路

为什么把 collect/train/test 做成纯 Python：
  无 GPU 虚拟机上，离线链路在终端直接跑，不依赖 ROS daemon，
  便于教师快速复现与排错；ROS launch 只做"实时仿真+在线巡航"。
"""
import argparse
import os
import sys

# 兼容两种启动方式：
#   python3 main.py collect          （源码目录直接跑）
#   python3 -m mjc_rov_current_teleop.main collect  （colcon install 后）
_PKG_DIR = os.path.dirname(os.path.abspath(__file__))
if _PKG_DIR not in sys.path:
    sys.path.insert(0, _PKG_DIR)

from mjc_rov_current_teleop import cruise_net  # noqa: E402


DEFAULT_PARAMS = dict(
    buoyancy_z=10.0, damping_lin=2.5, damping_ang=1.5,
    stab_roll=4.0, stab_pitch=4.0,
    current_amp_x=0.15, current_amp_y=0.10,
    current_w_x=0.30, current_w_y=0.23,
    max_force_x=5.0, max_force_y=5.0, max_torque_yaw=1.0,
)


def cmd_collect(args):
    cruise_net.collect_data(DEFAULT_PARAMS, steps=args.steps)


def cmd_train(args):
    cruise_net.train(DEFAULT_PARAMS, epochs=args.epochs)


def cmd_test(_args=None):
    r_net = cruise_net.evaluate(DEFAULT_PARAMS, use_net=True)
    print("---- 对比：纯脚本专家(无NN) ----")
    r_expert = cruise_net.evaluate(DEFAULT_PARAMS, use_net=False)
    print(f"[指标] MLP:  MAE={r_net['mae_position_m']}m 成功率={r_net['all_waypoints_hit']}")
    print(f"[指标] 专家: MAE={r_expert['mae_position_m']}m 成功率={r_expert['all_waypoints_hit']}")


def cmd_sim(_args):
    from mjc_rov_current_teleop.sim_node import main as m
    m()


def cmd_teleop(_args):
    from mjc_rov_current_teleop.keyboard_node import main as m
    m()


def cmd_cruise(_args):
    from mjc_rov_current_teleop.cruise_node import main as m
    m()


def build_parser():
    p = argparse.ArgumentParser(
        prog="mjc_rov_current_teleop",
        description="动态洋流 ROV MuJoCo 仿真 + MLP 巡航")
    sub = p.add_subparsers(dest="cmd")

    c = sub.add_parser("collect", help="采集专家数据")
    c.add_argument("--steps", type=int, default=4000)
    c.set_defaults(func=cmd_collect)

    t = sub.add_parser("train", help="训练 MLP")
    t.add_argument("--epochs", type=int, default=200)
    t.set_defaults(func=cmd_train)

    e = sub.add_parser("test", help="闭环评估")
    e.set_defaults(func=cmd_test)

    s = sub.add_parser("sim", help="ROS 仿真节点")
    s.set_defaults(func=cmd_sim)

    k = sub.add_parser("teleop", help="ROS 键盘节点")
    k.set_defaults(func=cmd_teleop)

    cr = sub.add_parser("cruise", help="ROS MLP 巡航节点")
    cr.set_defaults(func=cmd_cruise)

    a = sub.add_parser("all", help="collect->train->test 全链路")
    a.set_defaults(func=cmd_test)
    return p


def main():
    parser = build_parser()
    args = parser.parse_args()
    if not args.cmd:
        # 不带参数：全链路 collect -> train -> test
        print("== [1/3] 采集专家数据 ==")
        cruise_net.collect_data(DEFAULT_PARAMS, steps=4000)
        print("== [2/3] 训练 MLP ==")
        cruise_net.train(DEFAULT_PARAMS, epochs=200)
        print("== [3/3] 闭环评估 ==")
        cmd_test()
        return
    if args.cmd == "all":
        cruise_net.collect_data(DEFAULT_PARAMS, steps=4000)
        cruise_net.train(DEFAULT_PARAMS, epochs=200)
        cmd_test()
        return
    args.func(args)


if __name__ == "__main__":
    main()
