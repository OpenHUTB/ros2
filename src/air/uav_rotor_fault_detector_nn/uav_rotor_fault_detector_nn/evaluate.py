"""评测：在留出集上算指标、按故障类型分组报告、写 JSON/CSV。

报告的口径由 ``metrics`` 决定；这里只负责"把数据集切成能算指标的形状"并排版。
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from .data import FAULT_CN, load_dataset, split_by_episode
from .infer import HealthEstimator
from .metrics import summarize
from .model import load_checkpoint


def evaluate_checkpoint(
    checkpoint: str | Path,
    dataset_path: str | Path,
    split: str = "test",
    split_fractions: tuple[float, float, float] = (0.8, 0.1, 0.1),
) -> dict:
    """在 ``train/val/test`` 之一上评测，返回指标字典（含逐故障类型细分）。"""
    dataset = load_dataset(dataset_path)
    masks = split_by_episode(dataset, split_fractions)
    if split not in masks:
        raise ValueError(f"split 必须是 {sorted(masks)} 之一，收到 {split!r}")
    mask = masks[split]
    if not np.any(mask):
        raise RuntimeError(f"split={split} 上没有样本")

    estimator = HealthEstimator(checkpoint)
    predictions = estimator.estimate(dataset["X"][mask])
    targets = np.asarray(dataset["Y"][mask], dtype=np.float64)
    ends = np.asarray(dataset["end"][mask], dtype=np.int64)
    dt = float(dataset["dt"])

    kinds = np.asarray(dataset["kinds"], dtype=object)
    groups = kinds[dataset["episode"][mask]]

    result = summarize(predictions, targets, ends, dt, groups=groups)
    result.update(
        {
            "split": split,
            "checkpoint": str(checkpoint),
            "dataset": str(dataset_path),
            "window": int(estimator.window),
            "n_parameters": int(estimator.model.n_parameters()),
        }
    )
    return result


def _fmt(value, digits: int = 3) -> str:
    if value is None:
        return "-"
    if isinstance(value, str):
        return value
    if isinstance(value, float):
        if np.isnan(value):
            return "-"
        if np.isinf(value):
            return "未报警"
        return f"{value:.{digits}f}"
    return str(value)


def format_report(result: dict) -> str:
    """把指标字典排成终端里能看的中文表格。"""
    conf = result["confusion"]
    lines = [
        f"评测集合：{result['split']}    样本数：{result['n_samples']}    "
        f"窗口：{result['window']}    参数量：{result['n_parameters']}",
        "",
        f"四桨增益 MAE：{_fmt(result['mae'], 4)}    "
        f"逐桨 MAE：" + "  ".join(_fmt(v, 3) for v in result["per_rotor_mae"]),
        f"召回率：{_fmt(conf['recall'])}    精确率：{_fmt(conf['precision'])}    "
        f"误报率：{_fmt(conf['false_alarm_rate'])}    准确率：{_fmt(conf['accuracy'])}",
        f"混淆矩阵：TP={conf['tp']}  FP={conf['fp']}  TN={conf['tn']}  FN={conf['fn']}",
    ]

    per_group = result.get("per_group") or {}
    if per_group:
        lines.append("")
        header = f"{'故障类型':<10}{'样本':>6}{'MAE':>9}{'召回':>8}{'误报':>8}{'检测延时/s':>12}"
        lines.append(header)
        lines.append("-" * len(header.encode("utf-8")))
        for kind, row in per_group.items():
            name = FAULT_CN.get(kind, kind)
            lines.append(
                f"{name:<10}{row['n_samples']:>6}{_fmt(row['mae']):>9}"
                f"{_fmt(row['recall']):>8}{_fmt(row['false_alarm_rate']):>8}"
                f"{_fmt(row['latency']):>12}"
            )
    return "\n".join(lines)


def save_report(result: dict, path: str | Path) -> Path:
    """把指标写成 JSON（文档里的表格直接抄这里的数字）。"""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(result, handle, ensure_ascii=False, indent=2)
    return path


def save_prediction_csv(result: dict, path: str | Path) -> Path:
    """把分组指标写成 CSV，方便贴进报告。"""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = ["fault_kind,n_samples,mae,recall,false_alarm_rate,latency_s"]
    for kind, row in (result.get("per_group") or {}).items():
        latency = row["latency"]
        latency_text = "" if latency is None else str(latency)
        rows.append(
            f"{kind},{row['n_samples']},{row['mae']:.6f},{row['recall']:.6f},"
            f"{row['false_alarm_rate']:.6f},{latency_text}"
        )
    path.write_text("\n".join(rows) + "\n", encoding="utf-8")
    return path


def main(argv: list[str] | None = None) -> int:
    import argparse

    parser = argparse.ArgumentParser(description="评测桨故障检测网络")
    parser.add_argument("--checkpoint", default="results/detector.pt")
    parser.add_argument("--data", default="results/detector_dataset.npz")
    parser.add_argument("--split", default="test", choices=("train", "val", "test"))
    parser.add_argument("--json", default="results/detector_metrics.json")
    parser.add_argument("--csv", default="results/detector_by_fault.csv")
    args, _ = parser.parse_known_args(argv)

    result = evaluate_checkpoint(args.checkpoint, args.data, args.split)
    print(format_report(result))
    save_report(result, args.json)
    save_prediction_csv(result, args.csv)
    print(f"\n指标已写入 {args.json} 与 {args.csv}")
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
