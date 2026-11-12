"""从轨迹 CSV 计算制动指标。Compute quantitative braking metrics from a trajectory CSV."""

import argparse
import csv
import json
import math
from pathlib import Path


REQUIRED_COLUMNS = {
    "time_s",
    "range_m",
    "speed_m_s",
    "accel_m_s2",
    "collision",
    "sensor_valid",
}


def _read_rows(path):
    """读取并校验轨迹行。Read and validate trajectory rows."""
    with Path(path).open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        columns = set(reader.fieldnames or ())
        missing = sorted(REQUIRED_COLUMNS - columns)
        if missing:
            raise ValueError(f"缺少轨迹字段 / Missing trajectory columns: {', '.join(missing)}")
        rows = list(reader)
    if not rows:
        raise ValueError("轨迹文件为空 / Trajectory file is empty")
    return rows


def _float(row, name):
    try:
        value = float(row[name])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"字段 {name} 不是有效数字 / Field {name} is not numeric") from exc
    if not math.isfinite(value):
        raise ValueError(f"字段 {name} 必须为有限值 / Field {name} must be finite")
    return value


def _flag(row, name):
    value = _float(row, name)
    if value not in (0.0, 1.0):
        raise ValueError(f"字段 {name} 必须为 0 或 1 / Field {name} must be 0 or 1")
    return int(value)


def _settling_time(rows, speed_threshold):
    """返回首次进入并保持速度阈值的时间。Return first time that speed stays settled."""
    speeds = [abs(_float(row, "speed_m_s")) for row in rows]
    for index, speed in enumerate(speeds):
        if speed <= speed_threshold and all(value <= speed_threshold for value in speeds[index:]):
            return _float(rows[index], "time_s")
    return None


def summarize_trajectory(path, speed_threshold=0.05):
    """计算单条轨迹的可比较指标。Compute comparable metrics for one trajectory."""
    if not math.isfinite(speed_threshold) or speed_threshold <= 0:
        raise ValueError("速度阈值必须为正的有限值 / Speed threshold must be positive and finite")
    rows = _read_rows(path)
    clearances = [_float(row, "range_m") for row in rows]
    speeds = [_float(row, "speed_m_s") for row in rows]
    accelerations = [_float(row, "accel_m_s2") for row in rows]
    initial_clearance = clearances[0]
    minimum_clearance = min(clearances)
    return {
        "samples": len(rows),
        "initial_clearance_m": initial_clearance,
        "min_clearance_m": minimum_clearance,
        "braking_distance_m": max(0.0, initial_clearance - minimum_clearance),
        "overshoot_m": max(0.0, -minimum_clearance),
        "peak_abs_accel_m_s2": max(abs(value) for value in accelerations),
        "settling_time_s": _settling_time(rows, speed_threshold),
        "final_speed_m_s": speeds[-1],
        "collision_steps": sum(_flag(row, "collision") for row in rows),
        "invalid_sensor_steps": sum(1 for row in rows if _flag(row, "sensor_valid") == 0),
        "speed_threshold_m_s": speed_threshold,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trajectory", type=Path, help="trajectory.csv path")
    parser.add_argument("--output", type=Path, default=None, help="optional JSON output path")
    parser.add_argument("--speed-threshold", type=float, default=0.05)
    args = parser.parse_args()
    metrics = summarize_trajectory(args.trajectory, args.speed_threshold)
    payload = json.dumps(metrics, ensure_ascii=False, indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload + "\n", encoding="utf-8")
    print(payload)


if __name__ == "__main__":
    main()
