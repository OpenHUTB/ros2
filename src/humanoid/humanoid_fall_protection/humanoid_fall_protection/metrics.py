# -*- coding: utf-8 -*-
"""metrics.py —— 跌倒过程的指标采集与统计

指标口径（写进 docs，避免「头没着地」这种说法没有定义）：

  peak_head_force   头部球体与地面接触的法向力峰值（N），取整段仿真最大值。
                    接触坐标系 x 轴沿法线，所以取 mj_contactForce 的第 1 分量。
  head_min_gap      头部球面到地面的最小间隙（m）= 球心最低高度 − 0.09。
                    正值表示全程没碰到地面；负值表示压进去了多少。
  head_impact_speed 首次出现头部接触时，头部球心的竖直速度（m/s），负值为下落。
  head_impulse      头部法向接触力对时间的积分（N·s）。
  detect_delay      从推扰开始到触发保护的时间（s）。rigid/limp 也记录，
                    因为检测器三组共用，只是触发后做的事不同。
  settle_time       头部球心高度首次低于 0.20 m 的时刻（s），用作「倒地完成」。
  hand_first        手掌首次接触地面的时刻（s），用来验证「手先着地」。
"""

import csv
import os

import numpy as np

from . import config


class RunRecorder:
    def __init__(self, strategy, push_force, push_start, push_duration):
        self.strategy = strategy
        self.push_force = push_force
        self.push_start = push_start
        self.push_duration = push_duration
        self.rows = []
        self.head_min_z = 9.9
        self.head_peak_force = 0.0
        self.head_impulse = 0.0
        self.torso_peak_force = 0.0
        self.knee_peak_force = 0.0
        self.hand_peak_force = 0.0
        self.head_first_contact_t = None
        self.head_impact_speed = 0.0
        self.hand_first_t = None
        self.settle_time = None
        self.trigger_time = None
        self._prev_head_z = None

    def sample(self, sim, t, state):
        head_z = sim.head_height()
        vz = 0.0 if self._prev_head_z is None else (head_z - self._prev_head_z) / sim.dt
        self._prev_head_z = head_z

        head_force = sim.part_contact("head")
        hand_force = sim.part_contact("hand")
        torso_force = sim.part_contact("torso")
        knee_force = sim.part_contact("knee")

        if head_z < self.head_min_z:
            self.head_min_z = head_z
        if head_force > self.head_peak_force:
            self.head_peak_force = head_force
        if head_force > 0.0:
            self.head_impulse += head_force * sim.dt
            if self.head_first_contact_t is None:
                self.head_first_contact_t = t
                self.head_impact_speed = vz
        if hand_force > 2.0 and self.hand_first_t is None:
            self.hand_first_t = t
        self.hand_peak_force = max(self.hand_peak_force, hand_force)
        self.torso_peak_force = max(self.torso_peak_force, torso_force)
        self.knee_peak_force = max(self.knee_peak_force, knee_force)

        if self.settle_time is None and head_z < 0.20 and t > self.push_start:
            self.settle_time = t
        if state.get("just_triggered") and self.trigger_time is None:
            self.trigger_time = t

        self.rows.append((t, state["tilt"], state["tilt_rate"], head_z, head_force))

    def summary(self):
        # 检测延迟从「推扰开始」算起，而不是从推扰结束算：推扰只有 0.08 s，
        # 保护往往在推力还没撤掉时就已触发，从结束算会得到 0，看不出差别。
        delay = None if self.trigger_time is None else max(0.0, self.trigger_time - self.push_start)
        return {
            "strategy": self.strategy,
            "push_force": self.push_force,
            "head_min_z": self.head_min_z,
            "head_min_gap": self.head_min_z - 0.09,
            "head_peak_force": self.head_peak_force,
            "head_impulse": self.head_impulse,
            "head_impact_speed": self.head_impact_speed,
            "head_touched": self.head_first_contact_t is not None,
            "hand_first": self.hand_first_t,
            "torso_peak_force": self.torso_peak_force,
            "knee_peak_force": self.knee_peak_force,
            "detect_delay": delay,
            "settle_time": self.settle_time,
        }

    def write_csv(self, path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(["t", "tilt_rad", "tilt_rate_rad_s", "head_z", "head_force_n"])
            writer.writerows(self.rows)
        return path


def format_table(summaries):
    """把多组结果排成便于贴进终端/文档的文本表。"""
    header = ("策略", "头最低(m)", "头间隙(m)", "头峰值(N)", "躯干峰值(N)", "膝峰值(N)",
              "检测延迟(s)", "倒地(s)")
    lines = ["  ".join(header), "-" * 100]
    for s in summaries:
        delay = "-" if s["detect_delay"] is None else "{:.3f}".format(s["detect_delay"])
        settle = "-" if s["settle_time"] is None else "{:.2f}".format(s["settle_time"])
        label = config.STRATEGY_LABELS.get(s["strategy"], s["strategy"])
        # 中文按 2 个字符宽度算，手工补齐到 10 列，避免表格错位
        pad = " " * max(0, 10 - len(label) * 2)
        lines.append("  ".join((
            label + pad,
            "{:9.3f}".format(s["head_min_z"]),
            "{:9.3f}".format(s["head_min_gap"]),
            "{:9.1f}".format(s["head_peak_force"]),
            "{:9.0f}".format(s["torso_peak_force"]),
            "{:9.0f}".format(s["knee_peak_force"]),
            "{:>11s}".format(delay),
            "{:>7s}".format(settle),
        )))
    return "\n".join(lines)


def reduction_vs_best_passive(summaries):
    """保护策略相对「两个被动策略中更好的那个」的峰值力降低比例。

    Asimov 1 的评估脚本用的就是这个口径（impact_reduction_vs_best_passive）：
    与僵直/瘫软里表现更好的那个比，而不是与最差的那个比，避免夸大效果。
    """
    by = {s["strategy"]: s for s in summaries}
    if "protect" not in by:
        return None
    passive = [by[k]["head_peak_force"] for k in ("rigid", "limp") if k in by]
    if not passive:
        return None
    best_passive = min(passive)
    if best_passive <= 1e-9:
        return None
    return (best_passive - by["protect"]["head_peak_force"]) / best_passive


def summarize_sweep(rows):
    """阈值扫描结果的一行摘要（供绘图与文档表格使用）。"""
    return {
        "tilt_threshold_deg": rows[0]["tilt_threshold_deg"],
        "peak_head_force": rows[0]["head_peak_force"],
        "head_min_gap": rows[0]["head_min_gap"],
        "detect_delay": rows[0]["detect_delay"],
        "false_trigger": rows[0].get("false_trigger", False),
    }
