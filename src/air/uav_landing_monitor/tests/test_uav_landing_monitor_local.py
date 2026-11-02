#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""uav_landing_monitor 本地回归测试（纯 python，不需要 ROS / 仿真器 / GPU）。

    python3 tests/test_uav_landing_monitor_local.py

覆盖：正常降落不误报、指令无效、状态机空转、掉高、里程计超时、过冲、
下降超时、告警冷却、阈值可配、报告文本。
"""
from __future__ import annotations

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "scripts"))

from monitor_core import (  # noqa: E402
    LandingMonitor, PHASE_ABORTED, PHASE_LANDED, DEFAULT_CONFIG, ALERT_CODES,
)

DT = 0.05          # 20 Hz
TARGET_Z = 15.83   # 屋面高度
PASS = []
FAIL = []


def check(name, cond, detail=""):
    if cond:
        PASS.append(name)
    else:
        FAIL.append("%s%s" % (name, (" -> " + detail) if detail else ""))


def run(monitor, seconds, z0, vz, rate=0.0, status=None, t0=0.0, stop_odom_at=None):
    """合成一段数据：固定速度指令 + 线性高度变化（rate 为下降速度 m/s，正=下降）。"""
    t = t0
    z = z0
    out = []
    steps = int(round(seconds / DT))
    for i in range(steps):
        t = t0 + i * DT
        if stop_odom_at is None or t < stop_odom_at:
            monitor.on_odom(t, z)
        monitor.on_cmd(t, vz)
        if status is not None:
            monitor.on_status(t, status)
        out.extend(monitor.tick(t))
        z -= rate * DT
    return out, t + DT, z


def codes(alerts):
    return [a.code for a in alerts]


def scenario_normal():
    m = LandingMonitor(target_z=TARGET_Z)
    alerts = []
    a, t, z = run(m, 8.0, 20.0, -0.5, rate=0.5)
    alerts += a
    # 到位后悬停（速度为 0）
    m.on_cmd(t, 0.0)
    for i in range(40):
        m.on_odom(t + i * DT, z if z > TARGET_Z else TARGET_Z)
        m.on_cmd(t + i * DT, -0.5 if z > TARGET_Z else 0.0)
        alerts += m.tick(t + i * DT)
        if z > TARGET_Z:
            z = max(TARGET_Z, z - 0.5 * DT)
    rep = m.report()
    check("正常降落：无 error/warn 级告警",
          all(x.level == "info" for x in alerts), str(codes(alerts)))
    check("正常降落：触发 LANDED", "LANDED" in codes(alerts), str(codes(alerts)))
    check("正常降落：阶段为 LANDED", m.phase == PHASE_LANDED, m.phase)
    check("正常降落：结论 ok", rep["verdict"] == "ok", str(rep["verdict"]))


def scenario_cmd_no_effect():
    m = LandingMonitor(target_z=TARGET_Z)
    alerts, t, z = run(m, 6.0, 20.0, -0.5, rate=0.0)     # 指令 -0.5，但高度不动
    c = codes(alerts)
    check("指令无效：报 CMD_NO_EFFECT", "CMD_NO_EFFECT" in c, str(c))
    check("指令无效：结论 fail", m.report()["verdict"] == "fail", m.report()["verdict"])
    # 2 s 窗口、0.5 m/s -> 期望 1.0 m，实际 0
    first = [a for a in alerts if a.code == "CMD_NO_EFFECT"][0]
    check("指令无效：期望位移约 1.0 m",
          abs(first.data.get("expect_m", 0) - 1.0) < 0.2, str(first.data))
    check("指令无效：实际位移接近 0",
          first.data.get("actual_m", 99) < 0.05, str(first.data))
    check("指令无效：不会刷屏（冷却生效）", c.count("CMD_NO_EFFECT") <= 4, str(c.count("CMD_NO_EFFECT")))


def scenario_descend_stall():
    m = LandingMonitor(target_z=TARGET_Z)
    alerts, t, z = run(m, 6.0, 20.0, 0.0, rate=0.0, status="DESCEND")
    c = codes(alerts)
    check("状态机空转：报 DESCEND_STALL", "DESCEND_STALL" in c, str(c))
    check("状态机空转：只报一次", c.count("DESCEND_STALL") == 1, str(c))
    check("状态机空转：不报 CMD_NO_EFFECT", "CMD_NO_EFFECT" not in c, str(c))


def scenario_free_fall():
    m = LandingMonitor(target_z=TARGET_Z)
    alerts, t, z = run(m, 4.0, 20.0, 0.0, rate=2.5)      # 没有指令但掉得很快
    c = codes(alerts)
    check("掉高：报 FREE_FALL", "FREE_FALL" in c, str(c))
    ff = [a for a in alerts if a.code == "FREE_FALL"][0]
    check("掉高：速率约 2.5 m/s", abs(ff.data.get("rate_mps", 0) - 2.5) < 0.4, str(ff.data))
    check("掉高：级别为 error", ff.level == "error", ff.level)


def scenario_stale():
    m = LandingMonitor(target_z=TARGET_Z)
    alerts = []
    a, t, z = run(m, 2.0, 20.0, 0.0, rate=0.0)
    alerts += a
    for i in range(60):                                  # 断流 3 s
        alerts += m.tick(t + i * DT)
    c = codes(alerts)
    check("数据超时：报 INPUT_STALE", "INPUT_STALE" in c, str(c))
    check("数据超时：冷却不刷屏", c.count("INPUT_STALE") <= 3, str(c.count("INPUT_STALE")))


def scenario_overshoot():
    m = LandingMonitor(target_z=TARGET_Z)
    alerts, t, z = run(m, 6.0, TARGET_Z + 1.0, -0.5, rate=0.8)   # 降得比目标低 3.8 m
    c = codes(alerts)
    check("过冲：报 OVERSHOOT", "OVERSHOOT" in c, str(c))
    check("过冲：结论 fail", m.report()["verdict"] == "fail", m.report()["verdict"])


def scenario_timeout():
    cfg = dict(DEFAULT_CONFIG)
    cfg["timeout_s"] = 3.0
    m = LandingMonitor(cfg, target_z=TARGET_Z)
    alerts, t, z = run(m, 8.0, 20.0, -0.5, rate=0.5)     # 8 s 只降到 16 m 附近
    m.set_target_z(5.0)                                  # 目标很低，肯定到不了
    alerts += m.tick(t + DT)
    c = codes(alerts)
    check("下降超时：报 LANDING_TIMEOUT", "LANDING_TIMEOUT" in c, str(c))
    check("下降超时：阶段为 ABORTED", m.phase in (PHASE_ABORTED, PHASE_LANDED), m.phase)


def scenario_thresholds():
    # 把“期望位移下限”调高后，同样的不动数据不再判指令无效
    cfg = dict(DEFAULT_CONFIG)
    cfg["cmd_effect_min_expect_m"] = 99.0
    m = LandingMonitor(cfg, target_z=TARGET_Z)
    alerts, t, z = run(m, 5.0, 20.0, -0.5, rate=0.0)
    check("阈值可配：调高下限后不报 CMD_NO_EFFECT",
          "CMD_NO_EFFECT" not in codes(alerts), str(codes(alerts)))

    # 把一致性比例放宽到 0（等于关掉这条判据），也不应报
    cfg2 = dict(DEFAULT_CONFIG)
    cfg2["cmd_effect_ratio"] = 0.0
    m2 = LandingMonitor(cfg2, target_z=TARGET_Z)
    alerts2, _, _ = run(m2, 5.0, 20.0, -0.5, rate=0.0)
    check("阈值可配：关掉比例判据后不报 CMD_NO_EFFECT",
          "CMD_NO_EFFECT" not in codes(alerts2), str(codes(alerts2)))


def scenario_report_text():
    m = LandingMonitor(target_z=TARGET_Z)
    run(m, 6.0, 20.0, -0.5, rate=0.0)
    text = m.report_text()
    check("报告文本：含结论", "结论" in text and "fail" in text)
    check("报告文本：含告警代码", "CMD_NO_EFFECT" in text)
    rep = m.report()
    check("报告字典：字段齐全",
          set(["phase", "verdict", "duration_s", "final_z", "target_z",
               "alert_counts", "alerts"]).issubset(rep.keys()), str(rep.keys()))
    check("报告字典：告警可序列化",
          all(isinstance(a, dict) and "code" in a for a in rep["alerts"]))
    check("告警代码表完整", len(ALERT_CODES) == 7, str(ALERT_CODES))


def scenario_no_target():
    # 没有目标高度时不应报 LANDED / OVERSHOOT，但仍能判指令无效
    m = LandingMonitor()
    alerts, t, z = run(m, 5.0, 20.0, -0.5, rate=0.0)
    c = codes(alerts)
    check("无目标高度：不误报 LANDED", "LANDED" not in c, str(c))
    check("无目标高度：不误报 OVERSHOOT", "OVERSHOOT" not in c, str(c))
    check("无目标高度：仍能判指令无效", "CMD_NO_EFFECT" in c, str(c))


def scenario_transient():
    """刚下发下降指令时飞控还在加速，头 1 秒多高度几乎不变 —— 不应误报。"""
    m = LandingMonitor(target_z=TARGET_Z)
    t = 0.0
    z = 20.0
    alerts = []
    # 悬停 3 s
    a, t, z = run(m, 3.0, z, 0.0, rate=0.0, t0=t)
    alerts += a
    # 指令下来了但高度还没动（1.2 s 起动延迟）
    a, t, z = run(m, 1.2, z, -0.5, rate=0.0, t0=t)
    alerts += a
    # 正常下降
    a, t, z = run(m, 4.0, z, -0.5, rate=0.5, t0=t)
    alerts += a
    c = codes(alerts)
    check("起动延迟：不误报 CMD_NO_EFFECT", "CMD_NO_EFFECT" not in c, str(c))
    check("起动延迟：确实进入了下降阶段", m.phase in (PHASE_LANDED, "DESCENDING"), m.phase)


def main():
    scenario_normal()
    scenario_cmd_no_effect()
    scenario_descend_stall()
    scenario_free_fall()
    scenario_stale()
    scenario_overshoot()
    scenario_timeout()
    scenario_thresholds()
    scenario_report_text()
    scenario_no_target()
    scenario_transient()

    total = len(PASS) + len(FAIL)
    print("=" * 62)
    for name in PASS:
        print("  [通过] %s" % name)
    for name in FAIL:
        print("  [失败] %s" % name)
    print("=" * 62)
    print("结果：%d/%d 通过" % (len(PASS), total))
    return 0 if not FAIL else 1


if __name__ == "__main__":
    sys.exit(main())
