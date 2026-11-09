#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""降落过程监控核心逻辑（纯 python，不依赖 ROS，便于单测与复用）。

监控对象是"速度指令"与"实际高度变化"之间的一致性，而不是遥测数据本身的统计异常：

==================  =======================================================
``CMD_NO_EFFECT``   在下发下降指令，但窗口内实际下降量远小于指令积分
``DESCEND_STALL``   已经开始下降，之后没有下降指令、高度也没继续往下走
``FREE_FALL``       高度下掉得比指令快得多（掉高 / 失控）
``Z_REACHED``       高度进入目标 ± 容差（物理上"到位置了"）
``LANDED``          到位置 + 上游已释放下降指令（一次完整的落地）
``CMD_NOT_RELEASED`` 已到位置，但上游还在下发下降指令超过 ``landed_cmd_grace_s``
``OVERSHOOT``       已经降到目标高度以下超过 ``overshoot_m``（过冲）
``LANDING_TIMEOUT`` 从开始下降算起超过 ``timeout_s`` 仍未到位置
``INPUT_STALE``     里程计超过 ``stale_input_s`` 没有新数据
``TARGET_UNKNOWN``  迟迟拿不到落地目标高度（``/uav/goal`` 或落点话题都没来）
==================  =======================================================

设计约定（都是评审里明确提出来的，见 docs 第 2 节）：

* **坐标系统一为 ENU（上为正）**：NED 之类的换算由调用方用 :func:`to_enu` 先做掉，
  核心内部只有一种坐标系，避免"z 换了、vz 没换"的半实现；**偏移量在 ENU 侧施加**。
* **期望位移用指令积分** ``∫|−vz|dt``（只累计向下的部分），不是"最后一帧速度 × 窗口长度"；
  指令变化（暂停后恢复、变速）会重置观察窗，避免拿历史空档算期望值。
* **告警是上升沿触发**：条件持续期间只报一次，条件消失后才允许再报（另有冷却）；
  ``Z_REACHED`` / ``LANDED`` 这类 info 事件进"一次性"集合，**不进 ``active``**
  （``active`` 只表示"仍未恢复的异常"，落地成功不该被当成异常）。
* **可复位**：``reset()`` 会把阶段、一次性事件、当前这一轮的告警清掉，并把这一轮
  归档进 ``report()["runs"]``；``tick()`` 在三种情况下自动复位，因此一个常驻节点
  可以连续监控多次降落：
  1. 真的重新爬升了（比结束那一刻高 ``resume_climb_m`` 以上）；
  2. 超时之后**下降指令重新出现**（现场重试往往就从同一高度再来一次，不会爬升）；
  3. 上游下发了新的 ``/uav/goal``（:meth:`LandingMonitor.note_goal`）。
* **结论按"当前这一轮"算**：``report()["verdict"]`` 只看本轮告警，上一轮的失败不会
  污染下一轮；``report()["runs"]`` 给出每轮结论，总览在 ``report()["summary"]``。
* **线程安全**：``on_*`` / ``tick`` / ``report`` 都在同一把 ``RLock`` 里进出，
  rospy 的多线程回调与 Timer 并发调用不会抛 ``deque mutated during iteration``。
"""
from __future__ import annotations

import threading
from collections import deque

LEVEL_INFO = "info"
LEVEL_WARN = "warn"
LEVEL_ERROR = "error"

PHASE_IDLE = "IDLE"
PHASE_DESCENDING = "DESCENDING"
PHASE_LANDED = "LANDED"
PHASE_ABORTED = "ABORTED"

ALERT_CODES = (
    "CMD_NO_EFFECT",
    "DESCEND_STALL",
    "FREE_FALL",
    "Z_REACHED",
    "LANDED",
    "CMD_NOT_RELEASED",
    "OVERSHOOT",
    "LANDING_TIMEOUT",
    "INPUT_STALE",
    "TARGET_UNKNOWN",
)

#: 告警级别（给节点 / 画图工具复用，避免第二份拷贝）
LEVEL_OF = {
    "CMD_NO_EFFECT": LEVEL_ERROR,
    "DESCEND_STALL": LEVEL_WARN,
    "FREE_FALL": LEVEL_ERROR,
    "Z_REACHED": LEVEL_INFO,
    "LANDED": LEVEL_INFO,
    "CMD_NOT_RELEASED": LEVEL_WARN,
    "OVERSHOOT": LEVEL_ERROR,
    "LANDING_TIMEOUT": LEVEL_ERROR,
    "INPUT_STALE": LEVEL_ERROR,
    "TARGET_UNKNOWN": LEVEL_WARN,
}

#: 每轮最多报一次的事件（靠 ``_done`` 去重；**不进 active**）
ONCE_CODES = ("Z_REACHED", "LANDED", "LANDING_TIMEOUT", "TARGET_UNKNOWN")
#: 让"这一轮"判失败的告警
FATAL_CODES = ("INPUT_STALE", "CMD_NO_EFFECT", "FREE_FALL", "OVERSHOOT", "LANDING_TIMEOUT")

DEFAULT_CONFIG = {
    # 数据新鲜度
    "stale_input_s": 1.0,
    # “在下发下降指令”的判据（ENU：下降为负）
    "cmd_min_vz": 0.15,
    # 指令-运动一致性
    "cmd_effect_window_s": 2.0,
    "cmd_effect_ratio": 0.30,
    "cmd_effect_min_expect_m": 0.15,
    "cmd_change_eps": 0.10,       # 指令变化超过它就重置观察窗
    "descend_grace_s": 1.5,       # 刚进入下降的起动宽限期
    # 下降停滞（指令消失但没落地）
    "descend_stall_s": 2.0,
    "descend_stall_eps_m": 0.10,
    # 掉高（比指令快出 slack 才算失控，避免把"略快于指令的正常下降"判成 error）
    "free_fall_window_s": 0.6,
    "free_fall_rate": 1.2,
    "free_fall_cmd_slack": 1.0,
    # 落地
    "landing_tol_m": 0.35,
    "landed_stable_s": 0.8,       # 到位置后再稳 N 秒才算 Z_REACHED
    "landed_cmd_grace_s": 5.0,    # 到位置后上游多久还不松手就告警
    "overshoot_m": 0.30,
    "timeout_s": 90.0,
    # 目标高度
    "target_unknown_s": 8.0,
    "target_change_tol_m": 0.35,  # 目标高度变化小于它视为抖动（不重装填）
    # 复位
    "resume_climb_m": 1.0,        # 比结束时高出这么多才算"重新起飞"
    # 告警节流
    "alert_cooldown_s": 1.0,
    "status_descend_keywords": "",  # 桥接层状态里表示"正在下降"的关键字（默认无）
}


def parse_bool(value, default=False):
    """把 YAML / rosparam / 字符串统一成布尔值。

    ``rospy.get_param`` 在 ``-p z_up:=false`` 这类非 roslaunch YAML 路径下会给出字符串
    ``"false"``，直接 ``bool("false")`` 会得到 ``True``，所以这里显式解析。
    """
    if value is None:
        return bool(default)
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    text = str(value).strip().lower()
    if text in ("1", "true", "yes", "y", "on"):
        return True
    if text in ("0", "false", "no", "n", "off", ""):
        return False
    return bool(default)


def to_enu(value, z_up=True):
    """把高度类数值换算成 ENU（上为正）。``z_up=False`` 表示输入是 NED（下为正）。

    注意：**偏移量要在换算之后施加**。NED 下"目标点 z 减 5 m"必须写成
    ``to_enu(goal.z, z_up=False) - 5``，写成 ``to_enu(goal.z - 5, z_up=False)``
    会把偏移量一起反号，目标高度差 2 × offset。
    """
    if value is None:
        return None
    return float(value) if z_up else -float(value)


def goal_target_z(goal_z, z_up=True, offset_m=5.0):
    """把 ``/uav/goal`` 的 z（可能是 NED）折算成 ENU 下的落地目标高度。

    偏移量（"屋面上方安全高度"）在 **ENU 侧** 施加：先换算、再减。
    NED 下写成 ``to_enu(z - offset, False)`` 会把偏移量一起反号，目标高 2 × offset。
    """
    z = to_enu(goal_z, z_up)
    return None if z is None else z - float(offset_m)


class CsvClock(object):
    """节点写 CSV 用的时间原点。

    和 :class:`LandingMonitor` 的"每轮 ``t_first``"解耦：自动复位会清 ``t_first``，
    如果 CSV 时间列跟着它走，同一份 CSV 里的 ``t`` 会跳回 0，画图曲线上会出现回跳。
    """

    def __init__(self):
        self.t0 = None

    def stamp(self, now):
        now = float(now)
        if self.t0 is None:
            self.t0 = now
        return now - self.t0


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

    def __repr__(self):                                     # pragma: no cover
        return "Alert(%s/%s @%.2fs %s)" % (self.code, self.level, self.t, self.msg)


def _merge(cfg):
    """合并配置，返回 (生效配置, 未知键列表)。未知键不再静默忽略。"""
    out = dict(DEFAULT_CONFIG)
    unknown = []
    if cfg:
        for key, value in dict(cfg).items():
            if key in out:
                out[key] = value
            else:
                unknown.append(str(key))
    return out, sorted(unknown)


def _counts(alerts):
    out = {}
    for a in alerts:
        out[a.code] = out.get(a.code, 0) + 1
    return out


def _verdict(counts, phase):
    """单轮结论：只看这一轮的告警。"""
    fatal = [code for code in FATAL_CODES if counts.get(code)]
    if phase == PHASE_LANDED and not fatal:
        return "ok"
    if fatal:
        return "fail"
    return "warn"


class LandingMonitor(object):
    """降落过程监控器（纯逻辑：喂数据 -> 吐告警）。"""

    def __init__(self, config=None, target_z=None):
        self.cfg, self.unknown_keys = _merge(config)
        self._lock = threading.RLock()

        self.target_z = None if target_z is None else float(target_z)
        self.t_first = None
        self.t_last = None
        self.t_start = None

        self.z = None
        self.t_z = None
        self.vz_cmd = 0.0
        self.status = ""

        self._z_hist = deque()        # (t, z)，给"一致性""掉高""稳定"用，不因宽限期清空
        self._cmd_hist = deque()      # (t, vz)，用于指令积分
        self._alerts = []             # **当前这一轮**的告警
        self._runs = []               # 已经结束的轮次（每轮一条汇总）
        self.run_index = 1
        self._active = {}             # code -> 首次触发时间（持续中、仍未恢复的告警）
        self._last_alert_t = {}       # code -> 最近一次触发时间（冷却）
        self._done = set()            # 一次性事件（每轮清空）

        self._reset_state()

    # ------------------------------------------------------------------ 状态
    def _reset_state(self):
        self.phase = PHASE_IDLE
        self.t_descend_start = None
        self._effect_from = None
        self._z_reach_t = None
        self._phase_end_z = None
        self._phase_end_t = None
        self._landed = False
        self._descend_edge_t = None   # 最近一次"从不下降到下降"的时刻
        self._goal_t = None           # 最近一次收到新 /uav/goal 的时刻
        self._done = set()

    def reset(self, reason="", keep_history=True):
        """结束当前这一轮并复位到 IDLE，准备监控下一次降落。

        当前这一轮的告警会被归档进 ``report()["runs"]``，不会污染下一轮的结论；
        ``keep_history=False`` 时连归档也一起清掉。
        """
        with self._lock:
            self._archive_run(reason)
            if not keep_history:
                self._runs = []
            self._reset_state()
            self._alerts = []
            self._active = {}
            self._last_alert_t = {}
            self._z_hist.clear()
            self._cmd_hist.clear()
            self.z = None
            self.t_z = None
            self.vz_cmd = 0.0
            self.status = ""
            self.t_first = None
            self.t_last = None
            self.t_start = None
            self.run_index += 1

    def _run_snapshot(self):
        counts = _counts(self._alerts)
        duration = None
        if self.t_first is not None and self.t_last is not None:
            duration = round(self.t_last - self.t_first, 2)
        return {
            "run": self.run_index,
            "phase": self.phase,
            "verdict": _verdict(counts, self.phase),
            "duration_s": duration,
            "final_z": None if self.z is None else round(self.z, 3),
            "target_z": None if self.target_z is None else round(self.target_z, 3),
            "alert_counts": counts,
            "n_alerts": len(self._alerts),
        }

    def _archive_run(self, reason=""):
        if not self._alerts and self.phase == PHASE_IDLE:
            return                    # 什么都没发生的空段不必留档
        snap = self._run_snapshot()
        snap["end_reason"] = str(reason or "")
        self._runs.append(snap)

    # ------------------------------------------------------------------ 输入
    def on_odom(self, t, z):
        with self._lock:
            t = float(t)
            self.t_z = t
            self.z = float(z)
            if self.t_first is None:
                self.t_first = t
                self.t_start = t
            self.t_last = max(self.t_last or t, t)
            self._z_hist.append((t, float(z)))
            horizon = max(self.cfg["cmd_effect_window_s"], self.cfg["free_fall_window_s"],
                          self.cfg["descend_stall_s"]) + 2.0
            while self._z_hist and self._z_hist[0][0] < t - horizon:
                self._z_hist.popleft()

    def on_cmd(self, t, vz):
        with self._lock:
            t = float(t)
            old = self.vz_cmd
            self.vz_cmd = float(vz)
            self.t_last = max(self.t_last or t, t)
            self._cmd_hist.append((t, float(vz)))
            horizon = max(self.cfg["cmd_effect_window_s"], 2.0) + 2.0
            while self._cmd_hist and self._cmd_hist[0][0] < t - horizon:
                self._cmd_hist.popleft()
            if not self._is_descending(old) and self._is_descending(self.vz_cmd):
                # 下降指令"重新出现"：超时之后上游在同一高度重试就看这个边沿
                self._descend_edge_t = t
            if abs(self.vz_cmd - old) > float(self.cfg["cmd_change_eps"]):
                # 指令变了（暂停后恢复、变速、重新接管）：观察窗从头算
                self._effect_from = t

    def on_status(self, t, text):
        with self._lock:
            self.status = str(text or "").strip()
            self.t_last = max(self.t_last or float(t), float(t))

    def note_goal(self, t=None):
        """上游下发了一个新的 ``/uav/goal``（离散事件）：落地/超时后据此复位。"""
        with self._lock:
            self._goal_t = float(t) if t is not None else float(self.t_last or 0.0)

    def set_target_z(self, z, tol=None):
        """设置落地目标高度。

        目标变化**超过容差** ``target_change_tol_m``（默认 0.35 m）才算真的换目标，
        否则视为落点话题的 cm 级抖动，不重新装填、不会重复报 ``LANDED``。
        """
        with self._lock:
            new = None if z is None else float(z)
            changed = False
            if new is not None and self.target_z is not None:
                limit = float(self.cfg["target_change_tol_m"]) if tol is None else float(tol)
                changed = abs(new - self.target_z) > limit
            self.target_z = new
            if changed and self.phase in (PHASE_LANDED, PHASE_ABORTED):
                self.reset("目标高度变化")
            return changed

    def notify(self, now, code, level, msg, data=None):
        """外部（节点层）注入的告警，走同一套节流与历史。"""
        with self._lock:
            return self._emit(float(now), code, level, msg, data)

    # ------------------------------------------------------------------ 判据内部
    def _window(self, now, span):
        lo = now - span
        return [item for item in self._z_hist if item[0] >= lo]

    def _is_descending(self, vz):
        return float(vz) <= -float(self.cfg["cmd_min_vz"])

    def _descending_now(self):
        return self._is_descending(self.vz_cmd)

    def _status_descending(self):
        words = str(self.cfg.get("status_descend_keywords") or "").strip()
        if not words:
            return False
        upper = self.status.upper()
        return any(w.strip().upper() in upper for w in words.split(",") if w.strip())

    def _covered_drop(self, t0, t1):
        """(实际下降量, 样本数, 实际覆盖时长)；下降为正。"""
        win = [item for item in self._z_hist if t0 <= item[0] <= t1]
        if len(win) < 2:
            return None, len(win), 0.0
        covered = win[-1][0] - win[0][0]
        return win[0][1] - win[-1][1], len(win), covered

    def _expected_drop(self, t0, t1):
        """按指令积分算期望下降量（只累计向下的部分），下降为正。"""
        hist = [item for item in self._cmd_hist if item[0] <= t1]
        if not hist:
            return 0.0
        total = 0.0
        for i, (t_a, v_a) in enumerate(hist):
            t_b = hist[i + 1][0] if i + 1 < len(hist) else t1
            a, b = max(t_a, t0), min(t_b, t1)
            if b > a:
                total += max(0.0, -float(v_a)) * (b - a)
        return total

    def _stable_at(self, z_target, tol, span):
        win = self._window((self.t_last or 0.0), span)
        if len(win) < 2:
            return False
        span_t = win[-1][0] - win[0][0]
        if span_t < span * 0.8:
            return False
        return all(abs(z - z_target) <= tol for (_t, z) in win)

    # ------------------------------------------------------------------ 主循环
    def tick(self, now):
        now = float(now)
        with self._lock:
            if self.z is None or self.t_z is None:
                return []

            out = []
            # 0) 目标高度迟迟不来
            if (self.target_z is None and self.t_first is not None
                    and now - self.t_first > float(self.cfg["target_unknown_s"])
                    and "TARGET_UNKNOWN" not in self._done):
                out += self._emit(now, "TARGET_UNKNOWN", LEVEL_WARN,
                                  "已运行 %.1f s 仍没有落地目标高度（/uav/goal 或落点话题都没来），"
                                  "只能监测“掉高 / 断流”，无法判断是否落地"
                                  % (now - self.t_first), {})

            # 1) 自动复位，准备监控下一次降落
            #    a) 真的重新爬升了（比结束那一刻高 resume_climb_m 以上）
            #    b) 超时之后下降指令重新出现（现场重试常常就在同一高度再来一次）
            #    c) 上游下发了新的 /uav/goal
            if self.phase in (PHASE_LANDED, PHASE_ABORTED):
                end_t = self._phase_end_t if self._phase_end_t is not None else 0.0
                climbed = (self._phase_end_z is not None
                           and self.z > self._phase_end_z + float(self.cfg["resume_climb_m"]))
                retried = self._descend_edge_t is not None and self._descend_edge_t > end_t
                new_goal = self._goal_t is not None and self._goal_t > end_t
                if climbed or retried or new_goal:
                    reason = ("重新爬升" if climbed else
                              ("下降指令重新出现" if retried else "新的目标点"))
                    self.reset(reason)
                    return []        # 复位清空了高度/指令历史，本帧不再判据

            # 2) 进入下降
            if self.phase == PHASE_IDLE and (self._descending_now() or self._status_descending()):
                self.phase = PHASE_DESCENDING
                self.t_descend_start = now
                self._effect_from = now

            # 3) 里程计断流
            if now - self.t_z > float(self.cfg["stale_input_s"]):
                out += self._emit(now, "INPUT_STALE", LEVEL_ERROR,
                                  "里程计超时：%.2f s 没有新数据" % (now - self.t_z),
                                  {"age_s": round(now - self.t_z, 3)})
            else:
                self._active.pop("INPUT_STALE", None)

            grace = float(self.cfg["descend_grace_s"])
            in_grace = (self.t_descend_start is not None
                        and now - self.t_descend_start < grace)

            # 起动宽限期结束后，观察窗从宽限期结束那一刻开始算（否则起动那段静止会被当成"没动"）
            if not in_grace and self.t_descend_start is not None:
                self._effect_from = max(self._effect_from or 0.0, self.t_descend_start + grace)

            # 4) 有下降指令但高度不动（期望值用指令积分）
            if self.phase == PHASE_DESCENDING and self._descending_now() and not in_grace:
                span = float(self.cfg["cmd_effect_window_s"])
                t0 = max(now - span, self._effect_from or (now - span))
                drop, n, covered = self._covered_drop(t0, now)
                if drop is not None and n >= 3 and covered >= 0.8 * span:
                    expect = self._expected_drop(t0, now)
                    if (expect >= float(self.cfg["cmd_effect_min_expect_m"])
                            and drop < float(self.cfg["cmd_effect_ratio"]) * expect):
                        out += self._emit(
                            now, "CMD_NO_EFFECT", LEVEL_ERROR,
                            "下降指令积分 %.2f m，实际只降 %.2f m（窗口 %.1f s）"
                            % (expect, max(drop, 0.0), covered),
                            {"expect_m": round(expect, 3), "actual_m": round(drop, 3),
                             "window_s": round(covered, 3), "vz_cmd": round(self.vz_cmd, 3)})
                    else:
                        self._active.pop("CMD_NO_EFFECT", None)
                else:
                    self._active.pop("CMD_NO_EFFECT", None)

            # 5) 没有下降指令但还没落地（用桥接层真实存在的状态串，不靠 DESCEND）
            if self.phase == PHASE_DESCENDING and not self._descending_now() and not in_grace:
                span = float(self.cfg["descend_stall_s"])
                drop, n, covered = self._covered_drop(now - span, now)
                not_landed = (self.target_z is None
                              or abs(self.z - self.target_z) > float(self.cfg["landing_tol_m"]))
                if (drop is not None and covered >= 0.8 * span and not_landed
                        and drop < float(self.cfg["descend_stall_eps_m"])):
                    out += self._emit(now, "DESCEND_STALL", LEVEL_WARN,
                                      "已经没有下降指令（当前 vz=%.2f m/s），"
                                      "%.1f s 内高度只变化 %.2f m，仍未到达目标"
                                      % (self.vz_cmd, covered, max(drop, 0.0)),
                                      {"status": self.status, "actual_m": round(drop, 3)})
                else:
                    self._active.pop("DESCEND_STALL", None)

            # 6) 掉高（用完整高度历史，不受下降宽限期影响）
            #    只有"比指令快出 free_fall_cmd_slack"才算失控，正常下降略快于指令不算
            span = float(self.cfg["free_fall_window_s"])
            drop, n, covered = self._covered_drop(now - span, now)
            if drop is not None and n >= 2 and covered >= 0.3:
                rate = drop / covered
                excess = rate - max(0.0, -float(self.vz_cmd))
                if (rate > float(self.cfg["free_fall_rate"])
                        and excess > float(self.cfg["free_fall_cmd_slack"])):
                    out += self._emit(now, "FREE_FALL", LEVEL_ERROR,
                                      "高度以 %.2f m/s 下掉，比指令（%.2f m/s）快 %.2f m/s"
                                      % (rate, self.vz_cmd, excess),
                                      {"rate_mps": round(rate, 3),
                                       "excess_mps": round(excess, 3),
                                       "vz_cmd": round(self.vz_cmd, 3)})
                else:
                    self._active.pop("FREE_FALL", None)

            # 7) 到位置 / 落地 / 过冲 / 超时
            if self.target_z is not None:
                tol = float(self.cfg["landing_tol_m"])
                near = abs(self.z - self.target_z) <= tol
                stable = self._stable_at(self.target_z, tol, float(self.cfg["landed_stable_s"]))

                if near and stable:
                    if self._z_reach_t is None:
                        self._z_reach_t = now
                        out += self._emit(now, "Z_REACHED", LEVEL_INFO,
                                          "高度已进入目标 ±%.2f m：z=%.2f m（目标 %.2f m）"
                                          % (tol, self.z, self.target_z),
                                          {"z": round(self.z, 3),
                                           "target_z": round(self.target_z, 3)})
                    released = abs(self.vz_cmd) < float(self.cfg["cmd_min_vz"])
                    if released and not self._landed:
                        self._landed = True
                        self.phase = PHASE_LANDED
                        self._phase_end_z = self.z
                        self._phase_end_t = now
                        self._active.pop("CMD_NOT_RELEASED", None)
                        out += self._emit(now, "LANDED", LEVEL_INFO,
                                          "落地完成：z=%.2f m（目标 %.2f m），上游已释放下降指令"
                                          % (self.z, self.target_z),
                                          {"z": round(self.z, 3),
                                           "target_z": round(self.target_z, 3)})
                    elif (not released and not self._landed
                          and now - self._z_reach_t > float(self.cfg["landed_cmd_grace_s"])):
                        out += self._emit(now, "CMD_NOT_RELEASED", LEVEL_WARN,
                                          "已到目标高度 %.1f s，上游仍在发下降指令 vz=%.2f m/s"
                                          % (now - self._z_reach_t, self.vz_cmd),
                                          {"vz_cmd": round(self.vz_cmd, 3)})
                elif not near:
                    self._z_reach_t = None
                    self._active.pop("CMD_NOT_RELEASED", None)

                if (self.phase == PHASE_DESCENDING
                        and now - (self.t_descend_start or now)
                        >= float(self.cfg["descend_grace_s"])
                        and self.z < self.target_z - float(self.cfg["overshoot_m"])):
                    out += self._emit(now, "OVERSHOOT", LEVEL_ERROR,
                                      "高度 %.2f m 已低于目标 %.2f m 超过 %.2f m"
                                      % (self.z, self.target_z, self.cfg["overshoot_m"]),
                                      {"z": round(self.z, 3),
                                       "target_z": round(self.target_z, 3)})
                else:
                    self._active.pop("OVERSHOOT", None)

            # 已经到位置（Z_REACHED）就不再判"没到位置"的超时：
            # 上游不松手有 CMD_NOT_RELEASED 这个更准确的告警，不该同时拖成 fail + ABORTED
            if (self.phase == PHASE_DESCENDING and not self._landed
                    and self._z_reach_t is None
                    and self.t_descend_start is not None
                    and now - self.t_descend_start > float(self.cfg["timeout_s"])):
                out += self._emit(now, "LANDING_TIMEOUT", LEVEL_ERROR,
                                  "下降开始已 %.1f s 仍未到位置（上限 %.1f s）"
                                  % (now - self.t_descend_start, self.cfg["timeout_s"]),
                                  {"elapsed_s": round(now - self.t_descend_start, 2)})
                self.phase = PHASE_ABORTED
                self._phase_end_z = self.z
                self._phase_end_t = now

            self._alerts.extend(out)
            self.t_last = now
            return out

    # ------------------------------------------------------------------ 告警发射
    def _emit(self, now, code, level, msg, data=None):
        """上升沿触发：条件持续期间只报一次；一次性事件每轮只报一次。

        ``ONCE_CODES``（含 ``Z_REACHED`` / ``LANDED``）**不登记 active** ——
        active 的语义是"仍未恢复的异常"，把落地成功写进去会让看板显示成故障。
        """
        once = code in ONCE_CODES
        if once and code in self._done:
            return []
        if not once and code in self._active:
            return []
        last = self._last_alert_t.get(code)
        if last is not None and now - last < float(self.cfg["alert_cooldown_s"]):
            return []
        if once:
            self._done.add(code)
        else:
            self._active[code] = now
        self._last_alert_t[code] = now
        return [Alert(code, level, now, msg, data)]

    # ------------------------------------------------------------------ 汇总
    def active_codes(self):
        with self._lock:
            return sorted(self._active.keys())

    def report(self):
        """本轮结论 + 历史轮次。``verdict`` 只看当前这一轮，不会被上一轮的失败污染。"""
        with self._lock:
            counts = _counts(self._alerts)
            duration = None
            if self.t_first is not None and self.t_last is not None:
                duration = round(self.t_last - self.t_first, 2)
            total = dict(counts)
            for run in self._runs:
                for code, n in run.get("alert_counts", {}).items():
                    total[code] = total.get(code, 0) + n
            return {
                "run": self.run_index,
                "phase": self.phase,
                "verdict": _verdict(counts, self.phase),
                "duration_s": duration,
                "final_z": None if self.z is None else round(self.z, 3),
                "target_z": None if self.target_z is None else round(self.target_z, 3),
                "alert_counts": counts,
                "active": self.active_codes(),
                "alerts": [a.as_dict() for a in self._alerts],
                "runs": [dict(r) for r in self._runs],
                "summary": {"n_runs": len(self._runs) + (1 if counts or
                                                         self.phase != PHASE_IDLE else 0),
                            "alert_counts": total},
            }

    def report_text(self):
        rep = self.report()
        lines = ["=" * 66,
                 "降落过程监控报告",
                 "  结论      : %s（第 %d 轮，只统计本轮告警）" % (rep["verdict"], rep["run"]),
                 "  阶段      : %s" % rep["phase"],
                 "  时长      : %s s" % rep["duration_s"],
                 "  最终高度  : %s m（目标 %s m）" % (rep["final_z"], rep["target_z"]),
                 "  告警统计  : %s" % (rep["alert_counts"] or "无"),
                 "  仍未恢复  : %s" % (rep["active"] or "无")]
        if rep["runs"]:
            lines.append("  历史轮次  : %s" % "；".join(
                "第 %d 轮 %s（%s）" % (r["run"], r["verdict"], r.get("end_reason") or "-")
                for r in rep["runs"]))
            lines.append("  累计告警  : %s" % (rep["summary"]["alert_counts"] or "无"))
        lines.append("-" * 66)
        for a in rep["alerts"]:
            lines.append("  [%s] %-16s %.2fs  %s" % (a["level"], a["code"], a["t"], a["msg"]))
        lines.append("=" * 66)
        return "\n".join(lines)
