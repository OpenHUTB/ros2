#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""分层任务优先级全身 IK 演示。

两个场景：

1. **姿态保持**：7 自由度手臂做 3 维位置任务（4 维零空间），让冗余的
   关节回到参考位形。对比单任务解，看末端精度是否被牺牲。
2. **限位规避**：肘关节已经贴在上限附近，让零空间把它拉回安全区，
   末端位置保持不动。

运行::

    python3 demo_whole_body.py
    python3 demo_whole_body.py --outdir demo_out
"""

from __future__ import annotations

import argparse
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from ik_solver import solve_position_ik  # noqa: E402
from kinematics import make_humanoid_arm  # noqa: E402
from linalg import norm  # noqa: E402
from manipulability import joint_limit_cost  # noqa: E402
from simple_plot import plot_series  # noqa: E402
from whole_body_ik import (LimitAvoidanceTask, PositionTask,  # noqa: E402
                           PostureTask, solve_whole_body)

BLUE = (40, 110, 220)
RED = (215, 60, 55)
GREEN = (35, 165, 90)
ORANGE = (235, 150, 30)
PURPLE = (140, 80, 190)
GRAY = (150, 150, 150)


def hr(title=""):
    line = "=" * 74
    if title:
        print(line)
        print(title)
        print(line)
    else:
        print(line)


# ---------------------------------------------------------------------------
def limit_margin(chain, q):
    """最小归一化限位裕度：1 = 区间中点，0 = 正好贴边。"""
    best = 1.0
    for qi, joint in zip(q, chain.joints):
        if joint.limits is None:
            continue
        lo, hi = joint.limits
        mid, half = 0.5 * (lo + hi), 0.5 * (hi - lo)
        best = min(best, 1.0 - abs(qi - mid) / half)
    return best


def joint_margin(chain, q, index):
    """单个关节的归一化限位裕度；无限位时返回 1。"""
    joint = chain.joints[index]
    if joint.limits is None:
        return 1.0
    lo, hi = joint.limits
    mid, half = 0.5 * (lo + hi), 0.5 * (hi - lo)
    return 1.0 - abs(q[index] - mid) / half


def tip_error(chain, q, target):
    p, _r = chain.fk(q)
    return norm([target[i] - p[i] for i in range(3)])


def fmt_list(values, digits=3):
    return "[" + ", ".join("%+.*f" % (digits, v) for v in values) + "]"


# ---------------------------------------------------------------------------
def demo_posture(outdir):
    """场景一：末端定位 + 冗余关节回到参考位形。"""
    print()
    hr("一、姿态保持：末端到位，多余的自由度别乱摆")
    arm = make_humanoid_arm()
    q0 = [0.3, -0.4, 0.2, 1.0, 0.1, 0.2, 0.0]
    p0, _r = arm.fk(q0)
    target = [p0[0] + 0.08, p0[1] - 0.06, p0[2] + 0.05]
    q_ref = [0.0, -0.3, 0.1, 1.2, 0.0, 0.2, 0.0]

    print("  机械臂：%r" % (arm,))
    print("  任务：末端移动到 [%.3f, %.3f, %.3f]" % tuple(target))
    print("  参考位形：%s" % fmt_list(q_ref))
    print("  自由度 7 > 位置任务 3，零空间 4 维。")

    single = solve_position_ik(arm, target, q0=q0, tol=1e-6, max_iter=300)
    whole = solve_whole_body(
        arm, [PositionTask(target), PostureTask(q_ref, weight=0.8)],
        q0=q0, tol=1e-6, max_iter=400)

    print()
    print("  %-18s %-14s %-14s" % ("指标", "单任务", "全身 IK"))
    print("  " + "-" * 50)
    print("  %-18s %.2e    %.2e" % ("末端误差 (m)",
                                    tip_error(arm, single.q, target),
                                    tip_error(arm, whole.q, target)))
    print("  %-18s %.4f        %.4f" % ("|q - q_ref|",
                                        norm([a - b for a, b in zip(single.q, q_ref)]),
                                        norm([a - b for a, b in zip(whole.q, q_ref)])))
    print("  %-18s %d            %d" % ("迭代次数", single.iterations, whole.iterations))
    print()
    print("  单任务解 q = %s" % fmt_list(single.q))
    print("  全身 IK q  = %s" % fmt_list(whole.q))
    print("  参考位形 q = %s" % fmt_list(q_ref))
    print("  → 末端精度不变，离参考位形近了 %.0f%%（零空间里完成的）。"
          % (100.0 * (1.0 - norm([a - b for a, b in zip(whole.q, q_ref)])
                      / norm([a - b for a, b in zip(single.q, q_ref)]))))

    plot_series(
        os.path.join(outdir, "wb_primary_error.png"),
        [("single task", list(range(len(single.history))), single.history, GRAY),
         ("whole-body", list(range(len(whole.history))), whole.history, BLUE)],
        title="TIP ERROR PER ITERATION",
        xlabel="iteration", ylabel="tip error (m)",
    )
    plot_series(
        os.path.join(outdir, "wb_posture_compare.png"),
        [("single task", list(range(arm.dof)),
          [abs(a - b) for a, b in zip(single.q, q_ref)], GRAY),
         ("whole-body", list(range(arm.dof)),
          [abs(a - b) for a, b in zip(whole.q, q_ref)], GREEN)],
        title="POSTURE DEVIATION PER JOINT",
        xlabel="joint index", ylabel="deviation (rad)",
    )
    return arm, q0, target


# ---------------------------------------------------------------------------
def demo_limit(outdir):
    """场景二：肘关节贴限位时，用零空间把它拉回来。"""
    print()
    hr("二、限位规避：肘关节贴边时，末端别动、关节退回来")
    arm = make_humanoid_arm()
    # elbow 限位 (0, 2.6)，这里给到 2.45：已经进入限位裕度 25% 的警戒区
    q0 = [0.1, -0.2, 0.05, 2.45, 0.0, 0.1, 0.0]
    p0, _r = arm.fk(q0)
    target = [p0[0] + 0.04, p0[1] + 0.02, p0[2] + 0.03]

    print("  起始肘角 = %.3f rad（限位上限 %.3f）" % (q0[3], 2.6))
    print("  任务：末端移动 %.3f m，同时尽量远离限位" % norm(
        [target[i] - p0[i] for i in range(3)]))

    single = solve_position_ik(arm, target, q0=q0, tol=1e-6, max_iter=300)
    task = LimitAvoidanceTask(margin=0.25, gain=3.0, max_step=0.08)
    whole = solve_whole_body(
        arm, [PositionTask(target), task], q0=q0, tol=1e-6, max_iter=400)

    c_single, _ = joint_limit_cost(single.q, arm.joints, margin=0.25)
    c_whole, _ = joint_limit_cost(whole.q, arm.joints, margin=0.25)

    print()
    print("  %-18s %-14s %-14s" % ("指标", "单任务", "全身 IK"))
    print("  " + "-" * 50)
    print("  %-18s %.2e    %.2e" % ("末端误差 (m)",
                                    tip_error(arm, single.q, target),
                                    tip_error(arm, whole.q, target)))
    print("  %-18s %.3f        %.3f" % ("最小限位裕度",
                                        limit_margin(arm, single.q),
                                        limit_margin(arm, whole.q)))
    print("  %-18s %.3f        %.3f" % ("限位代价", c_single, c_whole))
    print("  %-18s %.3f        %.3f" % ("肘角 q3", single.q[3], whole.q[3]))
    print()
    print("  单任务解 q = %s" % fmt_list(single.q))
    print("  全身 IK q  = %s" % fmt_list(whole.q))
    print("  → 末端精度基本不变，肘角从 %.2f 退到 %.2f，限位裕度 %.3f → %.3f。"
          % (single.q[3], whole.q[3],
             limit_margin(arm, single.q), limit_margin(arm, whole.q)))

    plot_series(
        os.path.join(outdir, "wb_limit_margin.png"),
        [("single task", list(range(arm.dof)),
          [joint_margin(arm, single.q, i) for i in range(arm.dof)], GRAY),
         ("whole-body", list(range(arm.dof)),
          [joint_margin(arm, whole.q, i) for i in range(arm.dof)], GREEN)],
        title="LIMIT MARGIN PER JOINT",
        xlabel="joint index", ylabel="margin (1 = mid, 0 = at limit)",
    )
    return whole


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="分层任务优先级全身 IK 演示")
    ap.add_argument("--outdir", default=os.path.join(HERE, "demo_out"))
    args, _unknown = ap.parse_known_args()

    if not os.path.isdir(args.outdir):
        os.makedirs(args.outdir)

    print()
    hr("分层任务优先级全身逆运动学（task-priority whole-body IK）")
    print("  主任务：末端到位；次级任务：姿态保持 / 限位规避（只在零空间里动）")

    demo_posture(args.outdir)
    demo_limit(args.outdir)

    print()
    hr("完成")
    print("  曲线目录: %s" % os.path.abspath(args.outdir))
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
