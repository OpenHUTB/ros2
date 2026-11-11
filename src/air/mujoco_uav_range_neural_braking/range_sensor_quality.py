"""检查测距读数与机身几何间距的一致性。Check range readings against body clearance."""

import argparse
import csv
import json
import math
from pathlib import Path


REQUIRED_FIELDS = {"range_m", "sensor_range_m", "sensor_valid"}


def _finite_float(row, field):
    try:
        value = float(row[field])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"字段 {field} 必须为有效数字 / Field {field} must be numeric") from exc
    if not math.isfinite(value):
        raise ValueError(f"字段 {field} 必须为有限值 / Field {field} must be finite")
    return value


def analyze_trajectory(path, sensor_offset_m=0.01):
    """统计有效测距比例及补偿固定安装偏移后的几何误差。"""
    if not math.isfinite(sensor_offset_m):
        raise ValueError("传感器偏移必须为有限值 / Sensor offset must be finite")
    with Path(path).open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        missing = sorted(REQUIRED_FIELDS - set(reader.fieldnames or ()))
        if missing:
            raise ValueError(f"缺少轨迹字段 / Missing trajectory fields: {', '.join(missing)}")
        rows = list(reader)
    if not rows:
        raise ValueError("轨迹文件为空 / Trajectory file is empty")

    errors = []
    invalid_samples = 0
    for row in rows:
        reference_range = _finite_float(row, "range_m")
        valid_value = _finite_float(row, "sensor_valid")
        if valid_value not in (0.0, 1.0):
            raise ValueError("sensor_valid 必须为 0 或 1 / sensor_valid must be 0 or 1")
        if valid_value == 0:
            invalid_samples += 1
            continue
        measured_range = _finite_float(row, "sensor_range_m")
        errors.append(measured_range + sensor_offset_m - reference_range)

    valid_samples = len(errors)
    if not valid_samples:
        raise ValueError("没有有效测距样本 / No valid range samples")
    absolute_errors = sorted(abs(error) for error in errors)
    percentile_index = math.ceil(0.95 * valid_samples) - 1
    return {
        "samples": len(rows),
        "valid_samples": valid_samples,
        "invalid_samples": invalid_samples,
        "valid_rate": valid_samples / len(rows),
        "sensor_offset_m": sensor_offset_m,
        "mean_bias_m": sum(errors) / valid_samples,
        "mae_m": sum(absolute_errors) / valid_samples,
        "rmse_m": math.sqrt(sum(error * error for error in errors) / valid_samples),
        "max_abs_error_m": absolute_errors[-1],
        "p95_abs_error_m": absolute_errors[percentile_index],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trajectory", type=Path, help="trajectory.csv 路径 / trajectory.csv path")
    parser.add_argument("--sensor-offset", type=float, default=0.01,
                        help="传感器相对机身前缘的安装偏移（米）/ sensor offset from body edge in meters")
    parser.add_argument("--output", type=Path, default=None, help="可选 JSON 输出路径 / optional JSON output path")
    args = parser.parse_args()
    report = analyze_trajectory(args.trajectory, args.sensor_offset)
    payload = json.dumps(report, ensure_ascii=False, indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload + "\n", encoding="utf-8")
    print(payload)


if __name__ == "__main__":
    main()
