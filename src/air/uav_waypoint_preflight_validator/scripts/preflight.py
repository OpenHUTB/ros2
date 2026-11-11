#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线校验航点任务并可选输出 JSON。"""

import argparse
import json
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from preflight_validator import validate_mission_file, write_report


def _summary_text(report):
    summary = report["summary"]
    lines = [
        "status=%s valid=%s waypoints=%d segments=%d"
        % (report["status"], report["valid"],
           summary["waypoint_count"], summary["segment_count"]),
        "distance=%.3f m horizontal=%.3f m vertical=%.3f m duration=%.3f s"
        % (summary["total_distance_m"], summary["horizontal_distance_m"],
           summary["vertical_distance_m"], summary["estimated_duration_s"]),
        "max_turn=%.1f deg max_climb=%.1f deg errors=%d warnings=%d"
        % (summary["max_turn_angle_deg"], summary["max_climb_angle_deg"],
           summary["error_count"], summary["warning_count"]),
    ]
    for issue in report["errors"]:
        lines.append("ERROR %s: %s" % (issue["code"], issue["message"]))
    for issue in report["warnings"]:
        lines.append("WARN  %s: %s" % (issue["code"], issue["message"]))
    return "\n".join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description="无人机航点任务飞行前校验")
    parser.add_argument("--mission", required=True, help="任务 YAML 文件")
    parser.add_argument("--json-out", help="输出完整 JSON 报告的路径")
    parser.add_argument("--json", action="store_true", help="向标准输出打印完整 JSON")
    args = parser.parse_args(argv)

    report = validate_mission_file(args.mission)
    if args.json_out:
        write_report(args.json_out, report)
    if args.json:
        print(json.dumps(report, ensure_ascii=False, indent=2))
    else:
        print(_summary_text(report))
    return 0 if report["valid"] else 2


if __name__ == "__main__":
    sys.exit(main())
