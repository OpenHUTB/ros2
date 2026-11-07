#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""uav_landing_monitor 本地回归测试（纯 python，不需要 ROS / 仿真器 / GPU）。

    python3 tests/test_uav_landing_monitor_local.py

除了基础场景，这里专门覆盖评审提出的几类"以前没测到"的情况：
指令暂停后恢复 / 变速、低高度启动、第二次降落、到位置但上游不松手、
NED 换算、未知参数、线程并发、掉高不被起动宽限期吃掉。
"""
from __future__ import annotations

import os
import sys
import tempfile
import threading

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "scripts"))
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "tools"))

from monitor_core import (  # noqa: E402
    ALERT_CODES, DEFAULT_CONFIG, LEVEL_OF, PHASE_ABORTED, PHASE_IDLE, PHASE_LANDED,
    LandingMonitor, parse_bool, to_enu,
)

DT = 0.05
TARGET_Z = 15.83
PASS = []
FAIL = []


def check(name, cond, detail=""):
    if cond:
        PASS.append(name)
    else:
        FAIL.append("%s%s" % (name, (" -> " + detail) if detail else ""))


def fly(monitor, seconds, z, vz, rate=0.0, t0=0.0, status=None):
    """合成一段数据：固定速度指令 + 线性高度变化（rate 为下降速度，正=下降）。"""
    t = t0
    out = []
    for i in range(int(round(seconds / DT))):
        t = t0 + i * DT
        monitor.on_odom(t, z)
        monitor.on_cmd(t, vz)
        if status is not None:
            monitor.on_status(t, status)
        out.extend(monitor.tick(t))
        z -= rate * DT
    return out, t + DT, z


def codes(alerts):
    return [a.code for a in alerts]


def counts(alerts):
    out = {}
    for a in alerts:
        out[a.code] = out.get(a.code, 0) + 1
    return out


def land_normally(monitor, z=20.0, t0=0.0, hold=2.0):
    """从 z 降到目标并稳住，返回 (alerts, 结束时刻, 高度)。"""
    alerts = []
    dz = z - TARGET_Z
    dur = max(dz / 0.5, 0.2)
    a, t, zz = fly(monitor, dur, z, -0.5, rate=0.5, t0=t0)
    alerts += a
    a, t, zz = fly(monitor, hold, TARGET_Z, 0.0, rate=0.0, t0=t, status="HOVER")
    alerts += a
    return alerts, t, zz


# --------------------------------------------------------------------- 场景
def scenario_normal():
    m = LandingMonitor(target_z=TARGET_Z)
    alerts, t, z = land_normally(m)
    rep = m.report()
    c = codes(alerts)
    check("正常降落：有 Z_REACHED", "Z_REACHED" in c, str(c))
    check("正常降落：有 LANDED", "LANDED" in c, str(c))
    check("正常降落：无 error/warn", all(x.level == "info" for x in alerts), str(c))
    check("正常降落：阶段 LANDED", m.phase == PHASE_LANDED, m.phase)
    check("正常降落：结论 ok", rep["verdict"] == "ok", rep["verdict"])


def scenario_cmd_no_effect():
    m = LandingMonitor(target_z=TARGET_Z)
    alerts = []
    a, t, z = fly(m, 1.0, 20.0, 0.0, rate=0.0)          # 先悬停
    alerts += a
    a, t, z = fly(m, 8.0, z, -0.5, rate=0.0, t0=t)      # 有指令但不掉高度
    alerts += a
    c = codes(alerts)
    check("指令无效：报 CMD_NO_EFFECT", "CMD_NO_EFFECT" in c, str(c))
    check("指令无效：结论 fail", m.report()["verdict"] == "fail", m.report()["verdict"])
    first = [a for a in alerts if a.code == "CMD_NO_EFFECT"][0]
    check("指令无效：期望位移按指令积分（约 0.9 m）",
          abs(first.data.get("expect_m", 0) - 0.9) < 0.25, str(first.data))
    check("指令无效：实际位移接近 0",
          first.data.get("actual_m", 99) < 0.05, str(first.data))
    check("指令无效：持续故障只报一次（上升沿）",
          c.count("CMD_NO_EFFECT") == 1, str(counts(alerts)))


def scenario_grace_no_false_positive():
    """刚下发下降指令时飞控还在加速，前 1.2 s 高度几乎不变 —— 不应误报。"""
    m = LandingMonitor(target_z=TARGET_Z)
    t, z = 0.0, 20.0
    alerts = []
    a, t, z = fly(m, 1.0, z, 0.0, rate=0.0, t0=t)
    alerts += a
    a, t, z = fly(m, 1.2, z, -0.5, rate=0.0, t0=t)      # 起动延迟
    alerts += a
    a, t, z = fly(m, 4.0, z, -0.5, rate=0.5, t0=t)      # 正常下降
    alerts += a
    c = codes(alerts)
    check("起动延迟：不误报 CMD_NO_EFFECT", "CMD_NO_EFFECT" not in c, str(c))


def scenario_pause_resume():
    """评审第 2 条：下降途中上游暂停 2 s 再恢复 —— 恢复的头几帧不能误报。"""
    m = LandingMonitor(target_z=TARGET_Z)
    t, z = 0.0, 20.0
    alerts = []
    a, t, z = fly(m, 1.0, z, 0.0, rate=0.0, t0=t)
    alerts += a
    a, t, z = fly(m, 4.0, z, -0.5, rate=0.5, t0=t)      # 正常下降 4 s
    alerts += a
    base = len(alerts)
    a, t, z = fly(m, 2.0, z, 0.0, rate=0.0, t0=t)       # 暂停（指令清零）
    paused = a
    alerts += a
    a, t, z = fly(m, 1.0, z, -0.5, rate=0.5, t0=t)      # 恢复后的第一秒
    resume_first = a
    alerts += a
    check("暂停→恢复：恢复后的头几帧不报 CMD_NO_EFFECT",
          "CMD_NO_EFFECT" not in codes(resume_first), str(codes(resume_first)))
    check("暂停期间：指令消失会被判为 DESCEND_STALL（这是真实异常）",
          "DESCEND_STALL" in codes(paused), str(codes(paused)))
    check("暂停→恢复：全程没有 CMD_NO_EFFECT",
          "CMD_NO_EFFECT" not in codes(alerts), str(counts(alerts)))
    check("暂停→恢复：DESCEND_STALL 只报一次（上升沿）",
          codes(alerts).count("DESCEND_STALL") == 1, str(counts(alerts)))


def scenario_speed_change():
    """评审第 2 条：指令从 -0.2 提到 -0.8（飞机还在加速）不应误报。"""
    m = LandingMonitor(target_z=TARGET_Z)
    t, z = 0.0, 20.0
    alerts = []
    a, t, z = fly(m, 1.0, z, 0.0, rate=0.0, t0=t)
    alerts += a
    a, t, z = fly(m, 4.0, z, -0.2, rate=0.2, t0=t)      # 慢降
    alerts += a
    a, t, z = fly(m, 4.0, z, -0.8, rate=0.8, t0=t)      # 提速（其实是掉得更快，合理）
    alerts += a
    check("变速：不误报 CMD_NO_EFFECT", "CMD_NO_EFFECT" not in codes(alerts), str(counts(alerts)))


def scenario_descend_stall_real_vocabulary():
    """评审第 1 条：桥接层只说 VELOCITY/HOVER，没有 DESCEND。

    新判据不依赖状态串：下降途中指令消失、高度也不再下降、又没到目标 -> DESCEND_STALL。
    """
    m = LandingMonitor(target_z=TARGET_Z)
    t, z = 0.0, 20.0
    alerts = []
    a, t, z = fly(m, 1.0, z, 0.0, rate=0.0, t0=t)
    alerts += a
    a, t, z = fly(m, 3.0, z, -0.5, rate=0.5, t0=t)      # 下降
    alerts += a
    a, t, z = fly(m, 4.0, z, 0.0, rate=0.0, t0=t, status="HOVER")   # 指令消失且不落地
    alerts += a
    check("指令消失未落地：报 DESCEND_STALL（不依赖 DESCEND 字样）",
          "DESCEND_STALL" in codes(alerts), str(counts(alerts)))
    check("指令消失未落地：只报一次", codes(alerts).count("DESCEND_STALL") == 1, str(counts(alerts)))
    check("指令消失未落地：不误报 CMD_NO_EFFECT", "CMD_NO_EFFECT" not in codes(alerts),
          str(counts(alerts)))


def scenario_free_fall_in_grace():
    """小问题：起动宽限期不能把掉高判据也关掉。"""
    m = LandingMonitor(target_z=TARGET_Z)
    t, z = 0.0, 20.0
    alerts = []
    a, t, z = fly(m, 1.0, z, 0.0, rate=0.0, t0=t)
    alerts += a
    # 刚进入下降 0.5 s 就开始自由落体（6 m/s），指令只有 -0.5
    alerts_free, t, z = fly(m, 1.5, z, -0.5, rate=6.0, t0=t)
    check("宽限期内掉高：仍然报 FREE_FALL", "FREE_FALL" in codes(alerts_free),
          str(codes(alerts_free)))


def scenario_stale():
    m = LandingMonitor(target_z=TARGET_Z)
    alerts = []
    a, t, z = fly(m, 2.0, 20.0, 0.0, rate=0.0)
    alerts += a
    for i in range(60):
        alerts += m.tick(t + i * DT)
    c = codes(alerts)
    check("数据超时：报 INPUT_STALE", "INPUT_STALE" in c, str(c))
    check("数据超时：只报一次（上升沿）", c.count("INPUT_STALE") == 1, str(counts(alerts)))


def scenario_overshoot():
    m = LandingMonitor(target_z=TARGET_Z)
    alerts = []
    a, t, z = fly(m, 1.0, TARGET_Z + 3.0, 0.0, rate=0.0)
    alerts += a
    a, t, z = fly(m, 6.0, z, -0.5, rate=0.8, t0=t)      # 降过头
    alerts += a
    check("过冲：报 OVERSHOOT", "OVERSHOOT" in codes(alerts), str(counts(alerts)))
    check("过冲：结论 fail", m.report()["verdict"] == "fail", m.report()["verdict"])


def scenario_low_start():
    """评审第 4 条：飞机还在地面（0.5 m）就启动监控，不能一上来就报 OVERSHOOT。"""
    m = LandingMonitor(target_z=TARGET_Z)
    alerts = []
    a, t, z = fly(m, 3.0, 0.5, 0.0, rate=0.0, status="READY")   # 地面待机
    alerts += a
    check("低高度启动：不误报 OVERSHOOT", "OVERSHOOT" not in codes(alerts), str(codes(alerts)))
    check("低高度启动：不误报 LANDED", "LANDED" not in codes(alerts), str(codes(alerts)))
    # 之后正常起飞、降落，仍然能正确给出结论
    a, t, z = fly(m, 3.0, 0.5, 0.0, rate=-6.5, t0=t)           # 爬升到 20 m
    alerts += a
    a2, t, z = land_normally(m, z=z, t0=t)
    alerts += a2
    check("低高度启动：后续正常落地仍判 ok", m.report()["verdict"] == "ok",
          str(m.report()["verdict"]))
    check("低高度启动：最终阶段 LANDED", m.phase == PHASE_LANDED, m.phase)


def scenario_second_landing():
    """评审第 5 条：一次干净的落地之后，第二次降落（同样故障）必须还能报。"""
    m = LandingMonitor(target_z=TARGET_Z)
    land_normally(m)
    first = m.report()
    check("第二次降落：第一次 ok", first["verdict"] == "ok", str(first["verdict"]))
    # 爬回 20 m（触发自动复位）
    alerts = []
    a, t, z = fly(m, 3.0, TARGET_Z, 0.0, rate=-6.5, t0=first["duration_s"] or 0.0)
    alerts += a
    check("第二次降落：爬升后自动复位到 IDLE", m.phase == PHASE_IDLE, m.phase)
    # 复现完全相同的静默故障
    a, t, z = fly(m, 8.0, 20.0, -0.5, rate=0.0, t0=t)
    alerts += a
    check("第二次降落：故障仍能被抓到", "CMD_NO_EFFECT" in codes(alerts), str(counts(alerts)))


def scenario_cmd_not_released():
    """评审第 8 条：到位置了但上游还在发下降指令 —— 应报 CMD_NOT_RELEASED 而不是判超时。"""
    cfg = dict(DEFAULT_CONFIG)
    cfg["landed_cmd_grace_s"] = 2.0
    cfg["timeout_s"] = 60.0
    m = LandingMonitor(cfg, target_z=TARGET_Z)
    alerts = []
    a, t, z = fly(m, 1.0, TARGET_Z + 1.0, 0.0, rate=0.0)
    alerts += a
    a, t, z = fly(m, 2.0, TARGET_Z, -0.5, rate=0.0, t0=t)      # 到位置但指令不清零
    alerts += a
    a, t, z = fly(m, 4.0, TARGET_Z, -0.2, rate=0.0, t0=t)
    alerts += a
    c = codes(alerts)
    check("到位置但不松手：报 Z_REACHED", "Z_REACHED" in c, str(counts(alerts)))
    check("到位置但不松手：报 CMD_NOT_RELEASED", "CMD_NOT_RELEASED" in c, str(counts(alerts)))
    check("到位置但不松手：不误报 LANDING_TIMEOUT",
          "LANDING_TIMEOUT" not in c, str(counts(alerts)))
    check("到位置但不松手：未释放指令时不算 LANDED", "LANDED" not in c, str(counts(alerts)))


def scenario_timeout():
    cfg = dict(DEFAULT_CONFIG)
    cfg["timeout_s"] = 30.0
    m = LandingMonitor(cfg, target_z=TARGET_Z)
    alerts = []
    a, t, z = fly(m, 1.0, 20.0, 0.0, rate=0.0)
    alerts += a
    a, t, z = fly(m, 32.0, z, -0.5, rate=0.05, t0=t)     # 慢得离谱，30 s 也到不了
    alerts += a
    check("下降超时：报 LANDING_TIMEOUT", "LANDING_TIMEOUT" in codes(alerts), str(counts(alerts)))
    check("下降超时：阶段 ABORTED", m.phase == PHASE_ABORTED, m.phase)


def scenario_ned_conversion():
    """评审第 3 条：NED（下为正）必须能通过 to_enu 正确换算成 ENU。"""
    check("NED 换算：高度取负", abs(to_enu(15.0, z_up=False) + 15.0) < 1e-9)
    check("NED 换算：ENU 保持不变", abs(to_enu(15.0, z_up=True) - 15.0) < 1e-9)
    check("NED 换算：下降指令 +0.5 -> -0.5", abs(to_enu(0.5, z_up=False) + 0.5) < 1e-9)
    # 用换算后的数据喂核心，判据应当与 ENU 一致
    m = LandingMonitor(target_z=to_enu(-TARGET_Z, z_up=False))
    t, z = 0.0, to_enu(-20.0, z_up=False)
    alerts = []
    a, t, z = fly(m, 1.0, z, 0.0, rate=0.0, t0=t)
    alerts += a
    a, t, z = fly(m, 8.0, z, to_enu(0.5, z_up=False), rate=0.0, t0=t)
    alerts += a
    check("NED 换算：换算后仍能抓到 CMD_NO_EFFECT",
          "CMD_NO_EFFECT" in codes(alerts), str(counts(alerts)))


def scenario_param_parsing():
    check("参数解析：字符串 false -> False", parse_bool("false") is False)
    check("参数解析：字符串 true -> True", parse_bool("true") is True)
    check("参数解析：布尔原样", parse_bool(False) is False and parse_bool(True) is True)
    check("参数解析：0/1", parse_bool(0) is False and parse_bool(1) is True)
    check("参数解析：None 取默认", parse_bool(None, True) is True)


def scenario_unknown_config():
    """评审第 13 条：未知阈值键不能静默忽略。"""
    m = LandingMonitor({"cmd_effect_window": 3.0, "free_fall_rate": 2.0})
    check("未知配置：能报出来", m.unknown_keys == ["cmd_effect_window"], str(m.unknown_keys))
    check("未知配置：已知键生效", abs(m.cfg["free_fall_rate"] - 2.0) < 1e-9)
    m2 = LandingMonitor({"stale_input_s": 2.0})
    check("未知配置：没有未知键时不报", m2.unknown_keys == [], str(m2.unknown_keys))


def scenario_thread_safety():
    """评审第 7 条：多线程回调 + Timer 并发不能抛 deque mutated。"""
    m = LandingMonitor(target_z=TARGET_Z)
    errors = []

    def writer_odom():
        for i in range(400):
            try:
                m.on_odom(i * 0.01, 15.0 + 0.001 * i)
                m.tick(i * 0.01)
            except Exception as exc:                     # pragma: no cover
                errors.append(repr(exc))

    def writer_cmd():
        for i in range(400):
            try:
                m.on_cmd(i * 0.01, -0.5 if i % 7 else 0.0)
                m.on_status(i * 0.01, "VELOCITY" if i % 3 else "HOVER")
            except Exception as exc:                     # pragma: no cover
                errors.append(repr(exc))

    threads = [threading.Thread(target=writer_odom), threading.Thread(target=writer_cmd)]
    for th in threads:
        th.start()
    for th in threads:
        th.join()
    check("线程安全：并发回调不抛异常", not errors, str(errors[:2]))
    check("线程安全：并发后报告可读", isinstance(m.report(), dict))


def scenario_reset_and_keep_alerts():
    m = LandingMonitor(target_z=TARGET_Z)
    land_normally(m)
    before = len(m.report()["alerts"])
    m.reset("手动复位")
    after = m.report()
    check("复位：阶段回到 IDLE", m.phase == PHASE_IDLE, m.phase)
    check("复位：历史告警默认保留", len(after["alerts"]) >= before, str(len(after["alerts"])))
    check("复位：复位后能重新报 LANDED",
          "LANDED" in codes(land_normally(m)[0]), "no LANDED after reset")


def scenario_report_text():
    m = LandingMonitor(target_z=TARGET_Z)
    fly(m, 1.0, 20.0, 0.0, rate=0.0)
    fly(m, 6.0, 20.0, -0.5, rate=0.0, t0=1.0)
    text = m.report_text()
    check("报告文本：含结论", "结论" in text and "fail" in text)
    check("报告文本：含告警代码", "CMD_NO_EFFECT" in text)
    rep = m.report()
    check("报告字典：字段齐全",
          set(["phase", "verdict", "duration_s", "final_z", "target_z",
               "alert_counts", "active", "alerts"]).issubset(rep.keys()), str(rep.keys()))
    check("报告字典：告警可序列化",
          all(isinstance(a, dict) and "code" in a for a in rep["alerts"]))
    check("告警码表与级别表一致",
          set(ALERT_CODES) == set(LEVEL_OF.keys()), str(set(ALERT_CODES) ^ set(LEVEL_OF.keys())))


def scenario_target_unknown():
    cfg = dict(DEFAULT_CONFIG)
    cfg["target_unknown_s"] = 2.0
    m = LandingMonitor(cfg)
    alerts = []
    a, t, z = fly(m, 4.0, 20.0, 0.0, rate=0.0)
    alerts += a
    check("目标缺失：报 TARGET_UNKNOWN", "TARGET_UNKNOWN" in codes(alerts), str(counts(alerts)))
    check("目标缺失：只报一次", codes(alerts).count("TARGET_UNKNOWN") == 1, str(counts(alerts)))


def scenario_target_change_resets():
    m = LandingMonitor(target_z=TARGET_Z)
    land_normally(m)
    check("目标变化：落地后阶段 LANDED", m.phase == PHASE_LANDED, m.phase)
    m.set_target_z(12.0)
    check("目标变化：换目标后自动复位", m.phase == PHASE_IDLE, m.phase)
    check("目标变化：目标高度已更新", abs(m.target_z - 12.0) < 1e-9, str(m.target_z))


def scenario_plot_tool():
    """评审第 10 条：画图工具要能读 CSV，并识别出全部告警（不是只画第一次）。"""
    import plot_report

    rows = [
        "0.0,IDLE,20.00,15.83,0.00,READY,",
        "1.0,DESCENDING,20.00,15.83,-0.50,VELOCITY,CMD_NO_EFFECT",
        "2.0,DESCENDING,20.00,15.83,-0.50,VELOCITY,CMD_NO_EFFECT",
        "3.0,DESCENDING,20.00,15.83,-0.50,VELOCITY,CMD_NO_EFFECT|FREE_FALL",
        "4.0,LANDED,15.83,15.83,0.00,HOVER,LANDED",
    ]
    tmp = tempfile.NamedTemporaryFile("w", suffix=".csv", delete=False, encoding="utf-8")
    tmp.write("t,phase,z,target_z,vz_cmd,status,alerts\n")
    tmp.write("\n".join(rows) + "\n")
    tmp.close()
    try:
        ts, zs, targets, vzs, codes_list = plot_report.load(tmp.name)
        flat = [c for cs in codes_list for c in cs]
        check("画图工具：能读 CSV", len(ts) == 5 and len(zs) == 5, str(len(ts)))
        check("画图工具：告警按出现次数保留（不是去重成 1 条）",
              flat.count("CMD_NO_EFFECT") == 3, str(flat))
        check("画图工具：级别表来自 core", plot_report.LEVEL_OF is LEVEL_OF)
    finally:
        os.unlink(tmp.name)


def main():
    for fn in (scenario_normal, scenario_cmd_no_effect, scenario_grace_no_false_positive,
               scenario_pause_resume, scenario_speed_change,
               scenario_descend_stall_real_vocabulary, scenario_free_fall_in_grace,
               scenario_stale, scenario_overshoot, scenario_low_start,
               scenario_second_landing, scenario_cmd_not_released, scenario_timeout,
               scenario_ned_conversion, scenario_param_parsing, scenario_unknown_config,
               scenario_thread_safety, scenario_reset_and_keep_alerts,
               scenario_report_text, scenario_target_unknown,
               scenario_target_change_resets, scenario_plot_tool):
        fn()

    total = len(PASS) + len(FAIL)
    print("=" * 66)
    for name in PASS:
        print("  [通过] %s" % name)
    for name in FAIL:
        print("  [失败] %s" % name)
    print("=" * 66)
    print("结果：%d/%d 通过" % (len(PASS), total))
    return 0 if not FAIL else 1


if __name__ == "__main__":
    sys.exit(main())
