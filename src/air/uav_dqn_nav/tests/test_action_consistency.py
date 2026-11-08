#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""动作空间一致性测试：DQN 离散动作表（35 个）自洽性与边界行为。

纯 numpy，无 ROS/torch/gymnasium 依赖，可在任何 Python 3.8+ 环境运行：
    python3 tests/test_action_consistency.py

也支持 pytest / colcon 收集：
    python3 -m pytest tests/test_action_consistency.py -q
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))
from policy import (ACTION_NAMES, ACTION_VELOCITIES, N_ACTIONS,
                    MAX_HORIZ_SPEED, MAX_VERT_SPEED, act_index_to_velocity)

PASS = 0
FAIL = 0


def check(name, cond, extra=""):
    global PASS, FAIL
    if cond:
        PASS += 1
        print("  [PASS]", name)
    else:
        FAIL += 1
        print("  [FAIL]", name, extra)


def run_checks():
    """执行全部检查，返回失败项数。

    原来的检查直接写在模块顶层，并在最后调用 sys.exit()。这样 pytest/colcon
    在导入这个模块时就会收到 SystemExit，测试收集直接失败（SystemExit: 0），
    根本不会执行任何用例。把检查收进函数后，pytest 可以正常收集下面的
    test_action_consistency。
    """
    global PASS, FAIL
    PASS = FAIL = 0

    print("== 动作空间一致性（35 个动作） ==")
    check("动作数=35", N_ACTIONS == 35)
    check("动作名数=动作数", len(ACTION_NAMES) == N_ACTIONS)
    check("速度表长度=动作数", len(ACTION_VELOCITIES) == N_ACTIONS)

    print("== 方向精度（16 方向，22.5° 间隔） ==")
    # 巡航 16 方向：单位向量长度≈1，角度间隔 22.5°
    for i in range(16):
        vx, vy = ACTION_VELOCITIES[i][0], ACTION_VELOCITIES[i][1]
        mag = math.hypot(vx, vy)
        check("巡航方向%d 速度幅值=2.1" % i, abs(mag - 0.7 * MAX_HORIZ_SPEED) < 1e-4,
              "mag=%.4f" % mag)
    # 相邻方向夹角：cos 关系，22.5° 的 cos = cos(π/8)
    if N_ACTIONS >= 16:
        d0 = ACTION_VELOCITIES[0][:2]
        d1 = ACTION_VELOCITIES[1][:2]
        cos_ang = (d0[0] * d1[0] + d0[1] * d1[1]) / (math.hypot(*d0) * math.hypot(*d1))
        check("相邻巡航方向夹角≈22.5°",
              abs(cos_ang - math.cos(math.pi / 8)) < 1e-4, "cos=%.5f" % cos_ang)

    print("== 双档速度与升降/悬停 ==")
    check("巡航=0.7×水平上限", abs(ACTION_VELOCITIES[0][0] - 0.7 * MAX_HORIZ_SPEED) < 1e-4)
    check("慢速=0.35×水平上限", abs(ACTION_VELOCITIES[16][0] - 0.35 * MAX_HORIZ_SPEED) < 1e-4)
    check("上升=+垂直上限", abs(ACTION_VELOCITIES[32][2] - MAX_VERT_SPEED) < 1e-4)
    check("下降=-垂直上限", abs(ACTION_VELOCITIES[33][2] + MAX_VERT_SPEED) < 1e-4)
    check("悬停=全零", all(abs(x) < 1e-6 for x in ACTION_VELOCITIES[34]))

    print("== 映射函数与越界裁剪 ==")
    v34 = ACTION_VELOCITIES[34]
    check("act_index_to_velocity(34)==悬停", act_index_to_velocity(34) == v34)
    check("act_index_to_velocity(99) 裁剪到悬停", act_index_to_velocity(99) == v34)
    check("act_index_to_velocity(-5) 裁剪到动作0", act_index_to_velocity(-5) == ACTION_VELOCITIES[0])
    check("act_index_to_velocity(0)==巡航东", act_index_to_velocity(0) == ACTION_VELOCITIES[0])
    check("act_index_to_velocity(16)==慢速东", act_index_to_velocity(16) == ACTION_VELOCITIES[16])

    print("\n结果: %d 通过, %d 失败" % (PASS, FAIL))
    return FAIL


def test_action_consistency():
    """pytest 入口：失败项数必须为 0。"""
    assert run_checks() == 0, f"{FAIL} 项动作空间检查未通过"


def main():
    """独立运行入口：返回进程退出码。"""
    return 1 if run_checks() else 0


if __name__ == "__main__":
    sys.exit(main())
