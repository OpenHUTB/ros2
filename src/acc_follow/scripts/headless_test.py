#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
headless_test.py —— ACC 自适应巡航无界面自动验证

场景：前车以 15 m/s 巡航 -> 减速停车（拥堵）-> 停止 7 s -> 重新起步回巡航。
本车由 ACC 控制器驱动，全程自动跟车。

验证项：
    [1] 全程不碰撞：最小车距 > 0.5 m
    [2] 跟停：前车停稳后本车停稳（v < 0.1 m/s）
    [3] 停稳车距：d0 = 3 m，允许 [2.4, 4.5] m
    [4] 不超速：全程 v <= v_cruise（15 m/s）
    [5] 重新起步：前车起步后本车恢复巡航（最终 v >= 14.5 m/s）
    [6] 稳态跟车车距：接近 d_des = d0 + t_gap*v（允许 [15, 22] m）

用法：python3 headless_test.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from vehicle_model import VehicleModel, LeadVehicle
from acc_controller import ACCController

V_CRUISE = 15.0
D0 = 3.0
T_GAP = 1.0
DT = 0.05
CAR_LEN = 4.5      # 车长（m），车距统一按保险杠间距计算

# 场景时间节点（s）
T_STOP_BEGIN = 8.0
T_STOP_END = 13.0
T_GO_BEGIN = 20.0
SIM_TIME = 34.0


def run_scenario():
    """运行 ACC 跟车场景，返回指标字典。"""
    host = VehicleModel(x=0.0, v=V_CRUISE, v_max=V_CRUISE,
                        a_acc=2.5, a_brk=4.0, dt=DT)
    lead = LeadVehicle(x=50.0, v=V_CRUISE, v_cruise=V_CRUISE,
                       a_acc=2.0, a_brk=3.0, dt=DT)
    lead.set_phase_times(T_STOP_BEGIN, T_STOP_END, T_GO_BEGIN)

    ctrl = ACCController(v_cruise=V_CRUISE, d0=D0, t_gap=T_GAP)

    min_gap = 1e9
    max_v = 0.0
    stop_gap = None        # 本车首次停稳时的车距
    v_host_at_stop = None  # 前车停稳后本车是否停稳

    lead_stopped = False
    steps = int(SIM_TIME / DT)
    for _ in range(steps):
        lead_v = lead.step()
        d_center = lead.x - host.x
        d = d_center - CAR_LEN          # 保险杠间距
        min_gap = min(min_gap, d)
        max_v = max(max_v, host.v)

        v_cmd, d_des = ctrl.compute(d_center, lead_v, host.v)
        host.step(v_cmd)

        # 前车停稳 -> 记录本车停稳状态与车距
        if lead_v < 0.05 and lead.t >= T_STOP_END and not lead_stopped:
            lead_stopped = True
        if lead_stopped and host.v < 0.05 and stop_gap is None:
            stop_gap = d

    metrics = {
        "min_gap": min_gap,
        "max_v": max_v,
        "stop_gap": stop_gap,
        "final_v": host.v,
        "final_gap": (lead.x - host.x) - CAR_LEN,
    }
    return metrics


def main():
    print("ACC 自适应巡航无界面验证")
    print("场景：前车 15 m/s 巡航 -> 8s 减速停车 -> 停 7s -> 起步回巡航")
    print("参数：v_cruise=%.1f m/s, d0=%.1f m, t_gap=%.1f s, dt=%.2f s"
          % (V_CRUISE, D0, T_GAP, DT))
    print("-" * 60)

    m = run_scenario()

    checks = [
        ("[1] 全程不碰撞 (min_gap > 2.0m)", m["min_gap"] > 2.0,
         "min_gap=%.2f m" % m["min_gap"]),
        ("[2] 跟停 (前车停稳后本车 v<0.1)", m["stop_gap"] is not None,
         "stop_gap=%.2f m" % (m["stop_gap"] if m["stop_gap"] is not None else -1)),
        ("[3] 停稳车距 (2.0~5.0m)",
         m["stop_gap"] is not None and 2.0 <= m["stop_gap"] <= 5.0,
         "stop_gap=%.2f m" % (m["stop_gap"] if m["stop_gap"] is not None else -1)),
        ("[4] 全程不超速 (v<=15.0)", m["max_v"] <= V_CRUISE + 0.01,
         "max_v=%.2f m/s" % m["max_v"]),
        ("[5] 重新起步恢复巡航 (v>=14.5)", m["final_v"] >= 14.5,
         "final_v=%.2f m/s" % m["final_v"]),
        ("[6] 稳态跟车车距 (14~22m)", 14.0 <= m["final_gap"] <= 22.0,
         "final_gap=%.2f m" % m["final_gap"]),
    ]

    ok = True
    for name, passed, detail in checks:
        flag = "PASS" if passed else "FAIL"
        if not passed:
            ok = False
        print("%-34s %s   (%s)" % (name, flag, detail))

    print("-" * 60)
    print("结果: %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
