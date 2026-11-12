#!/usr/bin/env python3
"""从时序 CSV 拟合电机温度预测模型，默认预测未来五秒。"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from mujoco_motor_temperature_predictor.forecast import fit_and_evaluate


def main() -> None:
    parser = argparse.ArgumentParser(description="从时序 CSV 评估电机温度预测，默认预测未来五秒")
    parser.add_argument("--csv", required=True)
    parser.add_argument("--horizon", type=float, default=5.0, help="预测时域，单位秒，默认 5.0")
    parser.add_argument("--model", default="output/temperature_model.json")
    parser.add_argument("--results", default="output/forecast_results.json")
    args = parser.parse_args()
    result = fit_and_evaluate(args.csv, horizon_s=args.horizon)
    model_path = Path(args.model)
    model_path.parent.mkdir(parents=True, exist_ok=True)
    payload = {key: result[key] for key in ("horizon_s", "mean", "scale", "weights")}
    model_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    result_path = Path(args.results)
    result_path.parent.mkdir(parents=True, exist_ok=True)
    result_path.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(f"horizon={result['horizon_s']:.1f} s train={result['train_samples']} test={result['test_samples']} RMSE={result['rmse_c']:.3f} C")


if __name__ == "__main__":
    main()
