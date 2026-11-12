"""汇总扫描结果的参数分组统计。Summarize scenario sweep results by parameter."""

import argparse
import csv
import json
import math
from pathlib import Path
from statistics import fmean


PARAMETERS = (
    "start_x_m",
    "wall_x_m",
    "initial_speed_m_s",
    "height_m",
)
REQUIRED_FIELDS = ("scenario", "collision_steps", "min_clearance_m", *PARAMETERS)


def _number(row, field):
    try:
        value = float(row[field])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"字段 {field} 必须为有效数字 / Field {field} must be numeric") from exc
    if not math.isfinite(value):
        raise ValueError(f"字段 {field} 必须为有限值 / Field {field} must be finite")
    return value


def analyze_sweep_csv(path):
    """按每个参数水平统计间距与样本碰撞占比。"""
    with Path(path).open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        missing = sorted(set(REQUIRED_FIELDS) - set(reader.fieldnames or ()))
        if missing:
            raise ValueError(f"缺少扫描字段 / Missing sweep fields: {', '.join(missing)}")
        rows = list(reader)
    if not rows:
        raise ValueError("扫描 CSV 为空 / Sweep CSV is empty")

    parsed = []
    for row in rows:
        collision_steps = _number(row, "collision_steps")
        if collision_steps < 0 or not collision_steps.is_integer():
            raise ValueError("collision_steps 必须为非负整数 / collision_steps must be a non-negative integer")
        scenario = (row.get("scenario") or "").strip()
        if not scenario:
            raise ValueError("scenario 不能为空 / scenario must not be empty")
        parsed.append({
            "scenario": scenario,
            "collision": collision_steps > 0,
            "min_clearance_m": _number(row, "min_clearance_m"),
            **{field: _number(row, field) for field in PARAMETERS},
        })

    factors = {}
    ranking = []
    for field in PARAMETERS:
        groups = {}
        for row in parsed:
            groups.setdefault(row[field], []).append(row)
        summaries = []
        for value, members in sorted(groups.items()):
            clearances = [member["min_clearance_m"] for member in members]
            collisions = sum(member["collision"] for member in members)
            summaries.append({
                "value": value,
                "scenario_count": len(members),
                "collision_scenarios": collisions,
                "collision_rate": collisions / len(members),
                "mean_min_clearance_m": fmean(clearances),
                "worst_min_clearance_m": min(clearances),
            })
        means = [summary["mean_min_clearance_m"] for summary in summaries]
        ranking.append({
            "parameter": field,
            "group_mean_clearance_span_m": max(means) - min(means),
        })
        factors[field] = summaries

    ranking.sort(key=lambda item: (-item["group_mean_clearance_span_m"], item["parameter"]))
    collision_scenarios = sum(row["collision"] for row in parsed)
    worst = min(parsed, key=lambda row: row["min_clearance_m"])
    return {
        "scenario_count": len(parsed),
        "collision_scenarios": collision_scenarios,
        "collision_rate": collision_scenarios / len(parsed),
        "minimum_clearance_m": worst["min_clearance_m"],
        "worst_scenario": worst["scenario"],
        "factor_summaries": factors,
        "factor_ranking": ranking,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sweep_csv", type=Path, help="场景扫描 CSV 路径 / scenario sweep CSV path")
    parser.add_argument("--output", type=Path, default=None, help="可选 JSON 输出路径 / optional JSON output path")
    args = parser.parse_args()
    report = analyze_sweep_csv(args.sweep_csv)
    payload = json.dumps(report, ensure_ascii=False, indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload + "\n", encoding="utf-8")
    print(payload)


if __name__ == "__main__":
    main()
