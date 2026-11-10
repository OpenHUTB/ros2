#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""飞行状态识别、位姿积分与轨迹记录（纯逻辑，不依赖 ROS）.

把状态判断、轨迹积分、记录导出从 ROS 节点里拆出来，好处是：

* 没有 ROS 环境的机器也能直接 import 并做单元测试；
* 阈值/步长/指令序列等都可以参数化，节点只负责收发消息。

对外提供：

* ``classify_state``   依据 Twist 的线速度/角速度细分为 9 种飞行状态；
* ``PoseIntegrator``   按固定步长积分出 x / y / theta；
* ``TrajectoryRecorder`` 记录采样点与状态统计，导出 CSV 与 JSON 汇总；
* ``parse_command_sequence`` / ``command_to_twist_values`` 指令序列解析。
"""
from __future__ import annotations

import csv
import json
import math
import os
from datetime import datetime

# ---------------------------------------------------------------- 状态定义
STATE_HOVER = "悬停"
STATE_FORWARD = "直线前进"
STATE_BACKWARD = "直线后退"
STATE_TURN_LEFT = "原地左转"
STATE_TURN_RIGHT = "原地右转"
STATE_FORWARD_LEFT = "前进左转"
STATE_FORWARD_RIGHT = "前进右转"
STATE_BACKWARD_LEFT = "后退左转"
STATE_BACKWARD_RIGHT = "后退右转"

#: ``(线速度方向, 角速度方向)`` -> 状态名；方向取值 1 / 0 / -1。
STATE_LABELS = {
    (0, 0): STATE_HOVER,
    (1, 0): STATE_FORWARD,
    (-1, 0): STATE_BACKWARD,
    (0, 1): STATE_TURN_LEFT,
    (0, -1): STATE_TURN_RIGHT,
    (1, 1): STATE_FORWARD_LEFT,
    (1, -1): STATE_FORWARD_RIGHT,
    (-1, 1): STATE_BACKWARD_LEFT,
    (-1, -1): STATE_BACKWARD_RIGHT,
}

ALL_STATES = tuple(STATE_LABELS[key] for key in sorted(STATE_LABELS))

# ---------------------------------------------------------------- 指令定义
#: 指令名 -> (线速度系数, 角速度系数)，实际速度再乘上 linear_speed / angular_speed。
COMMANDS = {
    "forward": (1.0, 0.0),
    "backward": (-1.0, 0.0),
    "turn_left": (0.0, 1.0),
    "turn_right": (0.0, -1.0),
    "hover": (0.0, 0.0),
}

#: 保持原有演示节奏：前进 -> 左转 -> 悬停。
DEFAULT_SEQUENCE = ("forward", "turn_left", "hover")


def _direction(value, deadband):
    """把速度分量归一化成 1 / 0 / -1（|value| 不超过死区时视为 0）。"""
    if value > deadband:
        return 1
    if value < -deadband:
        return -1
    return 0


def classify_state(linear_x, angular_z, linear_deadband=0.01, angular_deadband=0.01):
    """根据线速度/角速度返回飞行状态名称。

    >>> classify_state(1.0, 0.0)
    '直线前进'
    >>> classify_state(0.0, 0.0)
    '悬停'
    """
    key = (_direction(linear_x, linear_deadband), _direction(angular_z, angular_deadband))
    return STATE_LABELS[key]


def parse_command_sequence(text):
    """把 ``"forward, turn_right , hover"`` 解析成指令列表。

    传入 ``None`` 或空串时返回 :data:`DEFAULT_SEQUENCE`；出现未知指令时抛
    ``ValueError``，避免参数写错后静默变成悬停。
    """
    if text is None:
        return list(DEFAULT_SEQUENCE)
    tokens = [token.strip().lower() for token in str(text).split(",")]
    tokens = [token for token in tokens if token]
    if not tokens:
        return list(DEFAULT_SEQUENCE)
    unknown = [token for token in tokens if token not in COMMANDS]
    if unknown:
        raise ValueError(
            "未知指令: %s（可选: %s）" % (", ".join(unknown), ", ".join(sorted(COMMANDS)))
        )
    return tokens


def command_to_twist_values(command, linear_speed=1.0, angular_speed=1.0):
    """指令 -> ``(linear.x, angular.z)``。"""
    try:
        linear_sign, angular_sign = COMMANDS[command]
    except KeyError:
        raise ValueError("未知指令: %s" % command)
    return linear_sign * linear_speed, angular_sign * angular_speed


class PoseIntegrator(object):
    """按固定步长把速度指令积分成平面位姿。

    与原实现保持一致：收到“前进”时按 ``step_length`` 米前进，收到“转向”时按
    ``turn_step`` 弧度改变朝向；因此 ``step_length=0.5`` / ``turn_step=0.2``
    时的轨迹与旧版本完全一致，只是步长、死区变成了可配置项。
    """

    def __init__(self, step_length=0.5, turn_step=0.2,
                 linear_deadband=0.01, angular_deadband=0.01):
        self.step_length = float(step_length)
        self.turn_step = float(turn_step)
        self.linear_deadband = float(linear_deadband)
        self.angular_deadband = float(angular_deadband)
        self.reset()

    def reset(self):
        """回到原点、朝向 0。"""
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def step(self, linear_x, angular_z):
        """推进一个采样步，返回 ``(x, y, theta)``。"""
        if abs(linear_x) > self.linear_deadband:
            direction = 1.0 if linear_x > 0 else -1.0
            self.x += direction * self.step_length * math.cos(self.theta)
            self.y += direction * self.step_length * math.sin(self.theta)
        if abs(angular_z) > self.angular_deadband:
            direction = 1.0 if angular_z > 0 else -1.0
            self.theta += direction * self.turn_step
            # 归一化到 [-pi, pi]，避免长时间旋转后数值漂移
            self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        return self.x, self.y, self.theta


class TrajectoryRecorder(object):
    """记录轨迹采样点与状态统计，并导出 CSV / JSON 两种结果文件。"""

    CSV_HEADER = ["index", "time_s", "x", "y", "theta_deg", "state", "linear_x", "angular_z"]

    def __init__(self, dt=1.0, output_dir=None,
                 csv_name="drone_trajectory.csv",
                 json_name="drone_trajectory_summary.json"):
        self.dt = float(dt)
        self.output_dir = output_dir
        self.csv_name = csv_name
        self.json_name = json_name
        self.reset()

    def reset(self):
        """清空已记录的采样点与统计。"""
        self.samples = []
        self.state_counts = {}
        self._next_index = 0

    def add(self, state, x, y, theta, linear_x=0.0, angular_z=0.0, timestamp=None):
        """追加一个采样点；``timestamp`` 缺省时按 ``dt`` 自动累加。"""
        if timestamp is None:
            timestamp = self._next_index * self.dt
        self.samples.append({
            "index": self._next_index,
            "time_s": float(timestamp),
            "x": float(x),
            "y": float(y),
            "theta": float(theta),
            "linear_x": float(linear_x),
            "angular_z": float(angular_z),
            "state": state,
        })
        self.state_counts[state] = self.state_counts.get(state, 0) + 1
        self._next_index += 1

    @property
    def distance_m(self):
        """沿采样点的折线距离（米）。"""
        total = 0.0
        previous = None
        for sample in self.samples:
            if previous is not None:
                total += math.hypot(sample["x"] - previous["x"], sample["y"] - previous["y"])
            previous = sample
        return total

    def _span_seconds(self):
        """首末采样点之间的真实时间跨度（秒）。"""
        if len(self.samples) < 2:
            return 0.0
        return max(0.0, self.samples[-1]["time_s"] - self.samples[0]["time_s"])

    def _state_durations(self):
        """按相邻采样点的**实际时间差**累计各状态时长；最后一个采样点不计时长。"""
        durations = {state: 0.0 for state in self.state_counts}
        for previous, current in zip(self.samples, self.samples[1:]):
            delta = current["time_s"] - previous["time_s"]
            if delta > 0:
                durations[previous["state"]] += delta
        return durations

    def state_totals(self):
        """每种状态的次数、时长与占比（时长按真实时间戳累计）。"""
        count_all = len(self.samples)
        durations = self._state_durations()
        total = sum(durations.values())
        totals = {}
        for state in sorted(self.state_counts):
            duration = durations.get(state, 0.0)
            totals[state] = {
                "count": self.state_counts[state],
                "duration_s": round(duration, 3),
                "ratio": round(duration / total, 4) if total > 0 else 0.0,
            }
        return totals

    def summary(self, params=None):
        """生成可序列化的汇总字典（写入 JSON）。"""
        count_all = len(self.samples)
        summary = {
            "generated_at": datetime.now().astimezone().isoformat(timespec="seconds"),
            "duration_s": round(self._span_seconds(), 3),
            "fallback_dt": self.dt,
            "sample_count": count_all,
            "distance_m": round(self.distance_m, 4),
            "start_pose": self._pose(self.samples[0]) if count_all else None,
            "final_pose": self._pose(self.samples[-1]) if count_all else None,
            "bounding_box": self._bounding_box(),
            "states": self.state_totals(),
            "params": params or {},
        }
        return summary

    def write_csv(self, output_dir=None):
        """写出逐点 CSV，返回文件路径。"""
        path = os.path.join(self._resolve_dir(output_dir), self.csv_name)
        self._ensure_dir(path)
        with open(path, "w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(self.CSV_HEADER)
            for sample in self.samples:
                writer.writerow([
                    sample["index"],
                    "%.3f" % sample["time_s"],
                    "%.4f" % sample["x"],
                    "%.4f" % sample["y"],
                    "%.3f" % math.degrees(sample["theta"]),
                    sample["state"],
                    "%.4f" % sample["linear_x"],
                    "%.4f" % sample["angular_z"],
                ])
        return path

    def write_json(self, params=None, output_dir=None):
        """写出汇总 JSON，返回文件路径。"""
        path = os.path.join(self._resolve_dir(output_dir), self.json_name)
        self._ensure_dir(path)
        with open(path, "w", encoding="utf-8") as handle:
            json.dump(self.summary(params=params), handle, ensure_ascii=False, indent=2)
        return path

    def export(self, params=None, output_dir=None):
        """同时导出 CSV 与 JSON，返回 ``{"csv": ..., "json": ...}``。"""
        return {
            "csv": self.write_csv(output_dir=output_dir),
            "json": self.write_json(params=params, output_dir=output_dir),
        }

    # ------------------------------------------------------------ 内部工具
    def _resolve_dir(self, output_dir=None):
        directory = output_dir or self.output_dir
        if not directory:
            directory = os.getcwd()
        return directory

    @staticmethod
    def _ensure_dir(path):
        parent = os.path.dirname(os.path.abspath(path))
        if parent and not os.path.isdir(parent):
            os.makedirs(parent)

    @staticmethod
    def _pose(sample):
        return {
            "x": round(sample["x"], 4),
            "y": round(sample["y"], 4),
            "theta_rad": round(sample["theta"], 4),
        }

    def _bounding_box(self):
        if not self.samples:
            return None
        xs = [sample["x"] for sample in self.samples]
        ys = [sample["y"] for sample in self.samples]
        return {
            "x_min": round(min(xs), 4),
            "x_max": round(max(xs), 4),
            "y_min": round(min(ys), 4),
            "y_max": round(max(ys), 4),
        }
