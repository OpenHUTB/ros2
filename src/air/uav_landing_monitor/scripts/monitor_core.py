#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""降落过程监控核心逻辑（纯 python，不依赖 ROS，便于单测）。

监控对象是"指令"和"实际运动"之间的一致性，而不是遥测数据本身的统计异常：

==========  ==========================================================
``INPUT_STALE``        里程计超时（超过 ``stale_input_s`` 没收到新数据）
``CMD_NO_EFFECT``      在下发垂直速度指令，但高度几乎不变（典型：桥接层
                       把速度指令的时间片设得太短，命令被下一条顶掉）
``DESCEND_STALL``      上游状态机声称在 DESCEND，但根本没有下降指令下来
``FREE_FALL``          高度掉得比任何指令都快（掉高 / 失速）
``OVERSHOOT``          已经降到目标高度以下（过冲，可能砸到屋顶）
``LANDING_TIMEOUT``    从开始下降算起超过 ``timeout_s`` 仍未到达目标高度
``LANDED``             正常到达目标高度（info 级事件）
==========  ==========================================================

设计约定：
* 时间一律用调用方传入的秒（``rospy.get_time()`` 或测试里的合成时间），核心不做 I/O。
* 同一类告警有冷却时间 ``alert_cooldown_s``，避免刷屏；``LANDED`` /
  ``LANDING_TIMEOUT`` / ``OVERSHOOT`` 属于一次性事件。
* 高度用"上为正"的 ENU（m）。若桥接层给的是 NED，由节点层先取负。
"""
from __future__ import annotations

from collections import deque

LEVEL_INFO = "info"
LEVEL_WARN = "warn"
LEVEL_ERROR = "error"

PHASE_IDLE = "IDLE"
PHASE_DESCENDING = "DESCENDING"
PHASE_LANDED = "LANDED"
PHASE_ABORTED = "ABORTED"

ALERT_CODES = (
    "INPUT_STALE",
    "CMD_NO_EFFECT",
    "DESCEND_STALL",
    "FREE_FALL",
    "OVERSHOOT",
    "LANDING_TIMEOUT",
    "LANDED",
)

DEFAULT_CONFIG = {
    # 数据新鲜度
    "stale_input_s": 1.0,
    # “在下发下降指令”的判据
    "cmd_min_vz": 0.15,          # m/s，|vz| 小于它视为没在下降
    # 指令-运动一致性
    "cmd_effect_window_s": 2.0,  # 观察窗
    "cmd_effect_ratio": 0.30,    # 实际位移 < ratio × 期望位移 -> 判为指令无效
    "cmd_effect_min_expect_m": 0.15,   # 期望位移太小时不判（避免噪声）
    "descend_grace_s": 1.5,            # 刚进入下降的起动宽限期（飞控要时间加速）
    # 下降停滞（状态机说下降、但没有指令）
    "descend_stall_s": 2.0,
    "descend_stall_eps_m": 0.10,
    # 掉高
    "free_fall_window_s": 0.6,
    "free_fall_rate": 1.2,       # m/s，向下
    "free_fall_cmd_slack": 0.6,  # 指令本身大于它时不判掉高
    # 落地
    "landing_tol_m": 0.35,
    "overshoot_m": 0.30,
    "timeout_s": 90.0,
    # 告警节流
    "alert_cooldown_s": 3.0,
}

_ONCE_CODES = ("LANDED", "LANDING_TIMEOUT", "OVERSHOOT", "DESCEND_STALL")


class Alert(object):
    """一条告警。``data`` 里放数值证据，方便落 CSV / 画图。"""

    __slots__ = ("code", "level", "t", "msg", "data")

    def __init__(self, code, level, t, msg, data=None):
        self.code = str(code)
        self.level = str(level)
        self.t = float(t)
        self.msg = str(msg)
        self.data = dict(data or {})

    def as_dict(self):
        return {"code": self.code, "level": self.level, "t": round(self.t, 3),
                "msg": self.msg, "data": self.data}

    def __repr__(self):                                   # pragma: no cover
        return "Alert(%s/%s @%.2fs %s)" % (self.code, self.level, self.t, self.msg)


def _merge(cfg):
    out = dict(DEFAULT_CONFIG)
    if cfg:
        for k, v in dict(cfg).items():
            if k in out:
                out[k] = v
    return out


class LandingMonitor(object):
    """降落过程监控器（纯逻辑，喂数据 -> 吐告警）。"""

    def __init__(self, config=None, target_z=None):
        self.cfg = _merge(config)
        self.target_z = None if target_z is None else float(target_z)

        self.phase = PHASE_IDLE
        self.t_first = None
        self.t_last = None
        self.t_descend_start = None
        self.t_landed = None

        self.z = None
        self.t_z = None
        self.vz_cmd = 0.0
        self.status = ""
        self.t_status = None

        self._z_hist = deque()          # (t, z)
        self._last_alert_t = {}         # code -> t
        self._fired = set()             # 一次性事件
        self.alerts = []                # 历史告警

    # ------------------------------------------------------------------ 输入
    def on_odom(self, t, z):
        self.t_z = float(t)
        self.z = float(z)
        if self.t_first is None:
            self.t_first = float(t)
        self.t_last = float(t)
        self._z_hist.append((float(t), float(z)))
        horizon = max(self.cfg["cmd_effect_window_s"], self.cfg["free_fall_window_s"]) + 2.0
        while self._z_hist and self._z_hist[0][0] < float(t) - horizon:
            self._z_hist.popleft()

    def on_cmd(self, t, vz):
        self.vz_cmd = float(vz)
        self.t_last = max(self.t_last or 0.0, float(t))

    def on_status(self, t, text):
        self.status = str(text or "").strip()
        self.t_status = float(t)
        self.t_last = max(self.t_last or 0.0, float(t))

    def set_target_z(self, z):
        self.target_z = float(z)

    # ------------------------------------------------------------------ 判据
    def _window(self, now, span):
        lo = now - span
        return [(t, z) for (t, z) in self._z_hist if t >= lo]

    def _descending_now(self):
        return self.vz_cmd <= -self.cfg["cmd_min_vz"]

    def _status_descending(self):
        return "DESCEND" in self.status.upper()

    def _drop_over(self, now, span):
        """返回 (实际下降量, 窗口起点 z, 样本数, 实际覆盖时长)，下降为正。

        ``span`` 只是"最多看多久"，真正参与计算的是**实际覆盖时长** —— 否则刚开始
        监控时窗口里只有一两个样本，会被误判成"指令无效"。
        """
        win = self._window(now, span)
        if len(win) < 2:
            return None, None, len(win), 0.0
        covered = float(win[-1][0] - win[0][0])
        return float(win[0][1] - win[-1][1]), float(win[0][1]), len(win), covered

    # ------------------------------------------------------------------ 主循环
    def tick(self, now):
        now = float(now)
        out = []
        if self.z is None or self.t_z is None:
            return out
        if self.t_first is None:
            self.t_first = now

        # 1) 进入下降阶段（有下降指令，或桥接层状态已经是 DESCEND）
        if self.phase == PHASE_IDLE and (self._descending_now() or self._status_descending()):
            self.phase = PHASE_DESCENDING
            self.t_descend_start = now

        # 2) 里程计超时
        if now - self.t_z > self.cfg["stale_input_s"]:
            out += self._alert(now, "INPUT_STALE", LEVEL_ERROR,
                               "里程计超时：%.2f s 没有新数据" % (now - self.t_z),
                               {"age_s": round(now - self.t_z, 3)})

        # 3) 有下降指令但高度不动
        grace = self.cfg.get("descend_grace_s", 0.0)
        in_grace = (self.t_descend_start is not None
                    and now - self.t_descend_start < grace)
        if in_grace and self._z_hist:
            # 宽限期内历史不清空的话，起动阶段那几秒会污染后面的观察窗
            last = self._z_hist[-1]
            self._z_hist.clear()
            self._z_hist.append(last)
        if self.phase == PHASE_DESCENDING and self._descending_now() and not in_grace:
            span = self.cfg["cmd_effect_window_s"]
            drop, z0, n, covered = self._drop_over(now, span)
            if drop is not None and n >= 3 and covered >= 0.8 * span:
                expect = abs(self.vz_cmd) * covered
                if (expect >= self.cfg["cmd_effect_min_expect_m"]
                        and drop < self.cfg["cmd_effect_ratio"] * expect):
                    out += self._alert(
                        now, "CMD_NO_EFFECT", LEVEL_ERROR,
                        "下降指令 %.2f m/s 持续 %.1f s，期望降 %.2f m，实际只降 %.2f m"
                        % (abs(self.vz_cmd), covered, expect, max(drop, 0.0)),
                        {"vz_cmd": round(self.vz_cmd, 3), "expect_m": round(expect, 3),
                         "actual_m": round(drop, 3), "window_s": round(covered, 3)})
        elif self.phase == PHASE_DESCENDING and self._status_descending() \
                and abs(self.vz_cmd) < self.cfg["cmd_min_vz"]:
            # 4) 状态机说在 DESCEND，但根本没有下降指令
            drop, z0, n, covered = self._drop_over(now, self.cfg["descend_stall_s"])
            if drop is not None and covered >= 0.8 * self.cfg["descend_stall_s"] \
                    and drop < self.cfg["descend_stall_eps_m"]:
                out += self._alert(now, "DESCEND_STALL", LEVEL_WARN,
                                   "状态为 DESCEND 但 %s 内没有下降指令，高度仅变化 %.2f m"
                                   % (self.status, max(drop, 0.0)),
                                   {"status": self.status, "actual_m": round(drop, 3)})

        # 5) 掉高
        span = self.cfg["free_fall_window_s"]
        drop, z0, n, covered = self._drop_over(now, span)
        if drop is not None and n >= 2 and covered >= 0.3:
            rate = drop / covered                        # 向下为正
            if (rate > self.cfg["free_fall_rate"]
                    and abs(self.vz_cmd) < self.cfg["free_fall_cmd_slack"]):
                out += self._alert(now, "FREE_FALL", LEVEL_ERROR,
                                   "高度以 %.2f m/s 下掉，但没有对应指令（指令 %.2f m/s）"
                                   % (rate, self.vz_cmd),
                                   {"rate_mps": round(rate, 3), "vz_cmd": round(self.vz_cmd, 3)})

        # 6) 过冲 / 落地 / 超时
        if self.target_z is not None:
            if self.z < self.target_z - self.cfg["overshoot_m"]:
                out += self._alert(now, "OVERSHOOT", LEVEL_ERROR,
                                   "高度 %.2f m 已低于目标 %.2f m 超过 %.2f m"
                                   % (self.z, self.target_z, self.cfg["overshoot_m"]),
                                   {"z": round(self.z, 3), "target_z": round(self.target_z, 3)})
            elif (self.phase == PHASE_DESCENDING
                  and abs(self.z - self.target_z) <= self.cfg["landing_tol_m"]
                  and abs(self.vz_cmd) < self.cfg["cmd_min_vz"]):
                out += self._alert(now, "LANDED", LEVEL_INFO,
                                   "已到达目标高度：z=%.2f m（目标 %.2f m，容差 %.2f m）"
                                   % (self.z, self.target_z, self.cfg["landing_tol_m"]),
                                   {"z": round(self.z, 3), "target_z": round(self.target_z, 3)})
                self.phase = PHASE_LANDED
                self.t_landed = now

        if (self.phase == PHASE_DESCENDING and self.t_descend_start is not None
                and now - self.t_descend_start > self.cfg["timeout_s"]):
            out += self._alert(now, "LANDING_TIMEOUT", LEVEL_ERROR,
                               "下降开始已 %.1f s 仍未到达目标高度（上限 %.1f s）"
                               % (now - self.t_descend_start, self.cfg["timeout_s"]),
                               {"elapsed_s": round(now - self.t_descend_start, 2)})
            self.phase = PHASE_ABORTED

        self.alerts.extend(out)
        self.t_last = now
        return out

    # ------------------------------------------------------------------ 工具
    def _alert(self, now, code, level, msg, data=None):
        if code in self._fired and code in _ONCE_CODES:
            return []
        last = self._last_alert_t.get(code)
        if last is not None and now - last < self.cfg["alert_cooldown_s"]:
            return []
        self._last_alert_t[code] = now
        if code in _ONCE_CODES:
            self._fired.add(code)
        return [Alert(code, level, now, msg, data)]

    # ------------------------------------------------------------------ 汇总
    def report(self):
        counts = {}
        for a in self.alerts:
            counts[a.code] = counts.get(a.code, 0) + 1
        errors = sum(c for code, c in counts.items()
                     if code in ("INPUT_STALE", "CMD_NO_EFFECT", "FREE_FALL",
                                 "OVERSHOOT", "LANDING_TIMEOUT"))
        if self.phase == PHASE_LANDED and errors == 0:
            verdict = "ok"
        elif errors == 0:
            verdict = "warn"            # 没落地但也没发现异常（可能只是没跑完）
        else:
            verdict = "fail"
        duration = None
        if self.t_first is not None and self.t_last is not None:
            duration = round(self.t_last - self.t_first, 2)
        return {
            "phase": self.phase,
            "verdict": verdict,
            "duration_s": duration,
            "final_z": None if self.z is None else round(self.z, 3),
            "target_z": None if self.target_z is None else round(self.target_z, 3),
            "alert_counts": counts,
            "alerts": [a.as_dict() for a in self.alerts],
        }

    def report_text(self):
        rep = self.report()
        lines = ["=" * 62,
                 "降落过程监控报告",
                 "  结论      : %s" % rep["verdict"],
                 "  阶段      : %s" % rep["phase"],
                 "  时长      : %s s" % rep["duration_s"],
                 "  最终高度  : %s m（目标 %s m）" % (rep["final_z"], rep["target_z"]),
                 "  告警统计  : %s" % (rep["alert_counts"] or "无"),
                 "-" * 62]
        for a in rep["alerts"]:
            lines.append("  [%s] %-16s %.2fs  %s" % (a["level"], a["code"], a["t"], a["msg"]))
        lines.append("=" * 62)
        return "\n".join(lines)
