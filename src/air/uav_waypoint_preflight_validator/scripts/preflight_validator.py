#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""无人机相对 ENU 航点任务的飞行前校验核心。"""

from __future__ import annotations

import json
import math
from pathlib import Path

import yaml


DEFAULT_LIMITS = {
    "min_altitude_m": 0.5,
    "max_altitude_m": 20.0,
    "max_waypoints": 100,
    "min_segment_length_m": 0.25,
    "max_climb_angle_deg": 60.0,
    "warning_turn_angle_deg": 90.0,
    "max_turn_angle_deg": 150.0,
    "max_horizontal_speed_mps": 1.5,
    "max_vertical_speed_mps": 0.8,
    "max_total_distance_m": 500.0,
}

ANGLE_LIMITS = (
    "max_climb_angle_deg",
    "warning_turn_angle_deg",
    "max_turn_angle_deg",
)


def _finite_number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("只接受 int/float，布尔值和字符串均无效")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("数值必须有限")
    return number


def _issue(code, message, waypoint=None, segment=None):
    issue = {"code": code, "message": message}
    if waypoint is not None:
        issue["waypoint"] = int(waypoint)
    if segment is not None:
        issue["segment"] = int(segment)
    return issue


def _resolve_limits(raw, errors):
    limits = dict(DEFAULT_LIMITS)
    if raw is None:
        raw = {}
    if not isinstance(raw, dict):
        errors.append(_issue("E010", "limits 必须是映射"))
        return limits

    for key, default in DEFAULT_LIMITS.items():
        if key not in raw:
            continue
        try:
            value = _finite_number(raw[key])
        except (TypeError, ValueError):
            errors.append(_issue("E011", "%s 必须是有限数值" % key))
            continue
        if key == "max_waypoints":
            if value != int(value) or value < 2:
                errors.append(_issue("E011", "max_waypoints 必须是至少为 2 的整数"))
                continue
            limits[key] = int(value)
        elif key in ("max_horizontal_speed_mps", "max_vertical_speed_mps",
                     "max_total_distance_m") and value <= 0:
            errors.append(_issue("E011", "%s 必须大于 0" % key))
            continue
        elif key == "min_altitude_m" and value < 0:
            errors.append(_issue("E011", "min_altitude_m 不得为负"))
            continue
        elif key == "min_segment_length_m" and value < 0:
            errors.append(_issue("E011", "min_segment_length_m 不得为负"))
            continue
        elif key in ANGLE_LIMITS and not 0 <= value <= 180:
            errors.append(_issue("E011", "%s 必须在 [0, 180] 度" % key))
            continue
        else:
            limits[key] = value

    if limits["max_altitude_m"] <= limits["min_altitude_m"]:
        errors.append(_issue("E011", "max_altitude_m 必须大于 min_altitude_m"))
    if limits["warning_turn_angle_deg"] > limits["max_turn_angle_deg"]:
        errors.append(_issue("E011", "warning_turn_angle_deg 不得大于 max_turn_angle_deg"))
    return limits


def _distance(a, b):
    return math.sqrt(sum((b[i] - a[i]) ** 2 for i in range(3)))


def _horizontal(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


def _turn_angle_deg(a, b, c):
    incoming = (b[0] - a[0], b[1] - a[1])
    outgoing = (c[0] - b[0], c[1] - b[1])
    n_in = math.hypot(*incoming)
    n_out = math.hypot(*outgoing)
    if n_in <= 1e-9 or n_out <= 1e-9:
        return None
    dot = incoming[0] * outgoing[0] + incoming[1] * outgoing[1]
    cross = incoming[0] * outgoing[1] - incoming[1] * outgoing[0]
    return abs(math.degrees(math.atan2(cross, dot)))


def _base_report(mission_path="", frame=""):
    return {
        "schema_version": 1,
        "mission_file": str(mission_path),
        "frame": str(frame),
        "status": "INVALID",
        "valid": False,
        "limits": dict(DEFAULT_LIMITS),
        "summary": {
            "waypoint_count": 0,
            "segment_count": 0,
            "total_distance_m": 0.0,
            "horizontal_distance_m": 0.0,
            "vertical_distance_m": 0.0,
            "estimated_duration_s": 0.0,
            "max_turn_angle_deg": 0.0,
            "max_climb_angle_deg": 0.0,
            "error_count": 0,
            "warning_count": 0,
        },
        "waypoints": [],
        "segments": [],
        "turns": [],
        "errors": [],
        "warnings": [],
    }


def validate_mission_data(mission, mission_path=""):
    """校验已解析的任务字典并返回可序列化报告。"""
    report = _base_report(mission_path)
    errors = report["errors"]
    warnings = report["warnings"]

    if not isinstance(mission, dict):
        errors.append(_issue("E001", "任务文件根节点必须是映射"))
        return _finalize(report)

    frame = mission.get("frame")
    report["frame"] = "" if frame is None else str(frame)
    if frame != "enu_relative":
        errors.append(_issue(
            "E002",
            "frame 必须显式为 enu_relative；当前模块不接受隐式坐标约定",
        ))

    limits = _resolve_limits(mission.get("limits"), errors)
    report["limits"] = limits

    raw_waypoints = mission.get("waypoints")
    if not isinstance(raw_waypoints, list):
        errors.append(_issue("E003", "waypoints 必须是列表"))
        return _finalize(report)
    if len(raw_waypoints) < 2:
        errors.append(_issue("E004", "至少需要两个航点"))
    if len(raw_waypoints) > limits["max_waypoints"]:
        errors.append(_issue(
            "E005",
            "航点数量 %d 超过上限 %d" % (len(raw_waypoints), limits["max_waypoints"]),
        ))

    valid_points = []
    for index, item in enumerate(raw_waypoints, start=1):
        if not isinstance(item, (list, tuple)) or len(item) != 3:
            errors.append(_issue("E006", "航点必须是 [x, y, z]", waypoint=index))
            report["waypoints"].append({
                "index": index,
                "valid": False,
                "status": "ERROR",
            })
            continue
        try:
            point = tuple(_finite_number(value) for value in item)
        except (TypeError, ValueError):
            errors.append(_issue("E006", "航点 x/y/z 必须是有限数值", waypoint=index))
            report["waypoints"].append({
                "index": index,
                "valid": False,
                "status": "ERROR",
            })
            continue

        status = "VALID"
        if not limits["min_altitude_m"] <= point[2] <= limits["max_altitude_m"]:
            errors.append(_issue(
                "E007",
                "航点高度 %.3f m 超出 [%.3f, %.3f] m"
                % (point[2], limits["min_altitude_m"], limits["max_altitude_m"]),
                waypoint=index,
            ))
            status = "ERROR"
        report["waypoints"].append({
            "index": index,
            "point": [float(value) for value in point],
            "valid": status == "VALID",
            "status": status,
        })
        valid_points.append((index, point))

    if len(valid_points) < 2:
        return _finalize(report)

    route = [(0, (0.0, 0.0, 0.0))] + valid_points
    total_distance = 0.0
    horizontal_total = 0.0
    vertical_total = 0.0
    duration_total = 0.0
    max_climb = 0.0

    for segment_index, ((from_index, a), (to_index, b)) in enumerate(
            zip(route[:-1], route[1:])):
        horizontal = _horizontal(a, b)
        vertical = abs(b[2] - a[2])
        distance = _distance(a, b)
        slope = 0.0 if horizontal <= 1e-9 else math.degrees(
            math.atan2(vertical, horizontal)
        )
        duration = max(
            horizontal / limits["max_horizontal_speed_mps"],
            vertical / limits["max_vertical_speed_mps"],
        )
        status = "VALID"
        if distance < limits["min_segment_length_m"]:
            errors.append(_issue(
                "E008",
                "航段长度 %.3f m 小于下限 %.3f m（重复或过近航点）"
                % (distance, limits["min_segment_length_m"]),
                waypoint=to_index,
                segment=segment_index,
            ))
            status = "ERROR"
        elif horizontal > 1e-9 and slope > limits["max_climb_angle_deg"]:
            errors.append(_issue(
                "E009",
                "航段爬升角 %.1f° 超过上限 %.1f°"
                % (slope, limits["max_climb_angle_deg"]),
                waypoint=to_index,
                segment=segment_index,
            ))
            status = "ERROR"
        max_climb = max(max_climb, slope)
        total_distance += distance
        horizontal_total += horizontal
        vertical_total += vertical
        duration_total += duration
        report["segments"].append({
            "index": segment_index,
            "from": "origin" if from_index == 0 else int(from_index),
            "to": int(to_index),
            "start": [float(value) for value in a],
            "end": [float(value) for value in b],
            "horizontal_distance_m": horizontal,
            "vertical_distance_m": vertical,
            "distance_m": distance,
            "climb_angle_deg": slope,
            "estimated_duration_s": duration,
            "status": status,
        })

    max_turn = 0.0
    for waypoint_index, a, b, c in zip(
            range(1, len(valid_points)), route[:-2], route[1:-1], route[2:]):
        turn = _turn_angle_deg(a[1], b[1], c[1])
        if turn is None:
            continue
        status = "VALID"
        if turn > limits["max_turn_angle_deg"]:
            errors.append(_issue(
                "E012",
                "转弯角 %.1f° 超过上限 %.1f°"
                % (turn, limits["max_turn_angle_deg"]),
                waypoint=waypoint_index,
            ))
            status = "ERROR"
        elif turn > limits["warning_turn_angle_deg"]:
            warnings.append(_issue(
                "W001",
                "转弯角 %.1f° 超过建议值 %.1f°"
                % (turn, limits["warning_turn_angle_deg"]),
                waypoint=waypoint_index,
            ))
            status = "WARNING"
        for item in report["waypoints"]:
            if item["index"] == waypoint_index:
                if item["status"] != "ERROR":
                    item["status"] = status
                    item["valid"] = status == "VALID"
                break
        max_turn = max(max_turn, turn)
        report["turns"].append({
            "waypoint": int(waypoint_index),
            "angle_deg": turn,
            "status": status,
        })

    if total_distance > limits["max_total_distance_m"]:
        errors.append(_issue(
            "E013",
            "总航程 %.2f m 超过上限 %.2f m"
            % (total_distance, limits["max_total_distance_m"]),
        ))
    elif total_distance > 0.8 * limits["max_total_distance_m"]:
        warnings.append(_issue(
            "W002",
            "总航程已超过上限的 80%",
        ))

    report["summary"].update({
        "waypoint_count": len(raw_waypoints),
        "segment_count": len(report["segments"]),
        "total_distance_m": total_distance,
        "horizontal_distance_m": horizontal_total,
        "vertical_distance_m": vertical_total,
        "estimated_duration_s": duration_total,
        "max_turn_angle_deg": max_turn,
        "max_climb_angle_deg": max_climb,
    })
    return _finalize(report)


def _finalize(report):
    report["summary"]["error_count"] = len(report["errors"])
    report["summary"]["warning_count"] = len(report["warnings"])
    report["valid"] = not report["errors"]
    report["status"] = "VALID" if report["valid"] else "INVALID"
    return report


def validate_mission_file(path):
    """读取 YAML 文件并校验，文件/YAML 错误也转成结构化报告。"""
    path = Path(path).expanduser()
    try:
        text = path.read_text(encoding="utf-8")
    except OSError as exc:
        report = _base_report(str(path))
        report["errors"].append(_issue("E001", "无法读取任务文件: %s" % exc))
        return _finalize(report)
    try:
        mission = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        report = _base_report(str(path))
        report["errors"].append(_issue("E001", "YAML 解析失败: %s" % exc))
        return _finalize(report)
    return validate_mission_data(mission, mission_path=str(path))


def write_report(path, report):
    """将报告写为 UTF-8 JSON。"""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
