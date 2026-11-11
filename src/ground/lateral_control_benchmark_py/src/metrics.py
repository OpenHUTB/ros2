"""统一的横向控制指标与场景内综合排名。"""

from __future__ import annotations

import math

import numpy as np

# 参与综合排名的指标，均为“越小越好”。
RANK_METRICS = (
    "mean_abs",
    "rms",
    "max_abs",
    "steady_abs",
    "convergence_s",
    "offroad_ratio",
    "steer_rate_rms",
)


def convergence_time(time, error, threshold: float = 0.2) -> float:
    """首次进入并一直保持在 threshold 内的时刻；从未满足则返回总时长。"""
    time = np.asarray(time, dtype=float)
    error = np.asarray(error, dtype=float)
    if time.size == 0:
        return 0.0
    inside = np.abs(error) <= threshold
    # 从末尾向前累计，找到最后一段连续满足条件的起点。
    tail = np.ones_like(inside, dtype=bool)
    for i in range(len(inside) - 2, -1, -1):
        tail[i] = bool(inside[i] and tail[i + 1])
    indices = np.flatnonzero(tail)
    if indices.size == 0:
        return float(time[-1] - time[0])
    return float(time[int(indices[0])] - time[0])


def evaluate(time, error, delta, dt: float, offroad_threshold: float = 0.8,
             convergence_threshold: float = 0.2, steady_fraction: float = 0.25) -> dict:
    """计算一组统一的横向跟踪指标。

    参数
    ----
    time, error, delta:
        时间、带符号横向偏差（左正）和前轮转角序列。
    dt:
        采样周期，用于转角变化率。
    offroad_threshold:
        离道判据 |e| > threshold。
    convergence_threshold:
        收敛判据 |e| <= 0.2 m。
    steady_fraction:
        稳态指标取末段比例。
    """
    time = np.asarray(time, dtype=float)
    error = np.asarray(error, dtype=float)
    delta = np.asarray(delta, dtype=float)
    if time.shape != error.shape or time.shape != delta.shape:
        raise ValueError("time, error and delta must have identical shapes")
    if time.size < 2:
        raise ValueError("at least two samples are required")
    if not np.all(np.isfinite(time)) or not np.all(np.isfinite(error)) or not np.all(np.isfinite(delta)):
        raise ValueError("metrics inputs contain non-finite values")

    abs_error = np.abs(error)
    tail_count = max(1, int(math.ceil(steady_fraction * len(abs_error))))
    steady_abs = float(np.mean(abs_error[-tail_count:]))
    steer_rate = np.diff(delta) / max(float(dt), 1e-12)

    return {
        "mean_abs": float(np.mean(abs_error)),
        "rms": float(np.sqrt(np.mean(error * error))),
        "max_abs": float(np.max(abs_error)),
        "steady_abs": steady_abs,
        "convergence_s": convergence_time(time, error, convergence_threshold),
        "offroad_ratio": float(np.mean(abs_error > offroad_threshold)),
        "steer_rate_rms": float(np.sqrt(np.mean(steer_rate * steer_rate))),
        "final_abs": float(abs_error[-1]),
    }


def average_ranks(values):
    """返回平均秩（1 最好），相同值取并列平均秩。"""
    values = np.asarray(values, dtype=float)
    n = len(values)
    ranks = np.empty(n, dtype=float)
    for i in range(n):
        less = np.sum(values < values[i] - 1e-12)
        equal = np.sum(np.abs(values - values[i]) <= 1e-12)
        ranks[i] = less + 0.5 * (equal + 1.0)
    return ranks


def rank_records(records):
    """按场景对每个指标排名，再对每个控制器的指标名次取平均。

    Parameters
    ----------
    records : list[dict]
        每项至少包含 scenario、controller、label 以及 RANK_METRICS 中的字段。

    Returns
    -------
    ranked : list[dict]
        复制并附加 metric_rank、scenario_rank。
    summary : list[dict]
        每个控制器的跨场景综合名次，按综合名次升序。
    """
    ranked = [dict(item) for item in records]
    scenarios = []
    for item in ranked:
        if item["scenario"] not in scenarios:
            scenarios.append(item["scenario"])

    for scenario in scenarios:
        subset = [item for item in ranked if item["scenario"] == scenario]
        metric_ranks = {key: average_ranks([item[key] for item in subset]) for key in RANK_METRICS}
        for index, item in enumerate(subset):
            per_metric = [float(metric_ranks[key][index]) for key in RANK_METRICS]
            item["metric_rank"] = float(np.mean(per_metric))
            item["scenario_rank"] = 0  # 稍后回填
        # 场景内综合名次
        order = np.argsort([item["metric_rank"] for item in subset])
        for place, idx in enumerate(order, start=1):
            subset[idx]["scenario_rank"] = place

    controllers = []
    for item in ranked:
        if item["controller"] not in controllers:
            controllers.append(item["controller"])
    summary = []
    for controller in controllers:
        items = [item for item in ranked if item["controller"] == controller]
        label = items[0]["label"]
        mean_rank = float(np.mean([item["metric_rank"] for item in items]))
        summary.append({
            "controller": controller,
            "label": label,
            "mean_rank": mean_rank,
            "scenarios": len(items),
        })
    summary.sort(key=lambda item: item["mean_rank"])
    for place, item in enumerate(summary, start=1):
        item["overall_rank"] = place
    return ranked, summary


def format_summary(records, summary, title="Lateral Control Benchmark"):
    """生成可直接写入 summary.txt 的中文文本报告。"""
    lines = [title, "=" * 78, ""]
    headers = ["Controller", "Scenario", "Mean|e|", "RMS", "Max|e|",
               "Steady|e|", "Conv[s]", "Offroad", "dDelta RMS", "Rank"]
    lines.append("{:<14} {:<12} {:>8} {:>8} {:>8} {:>9} {:>7} {:>8} {:>10} {:>6}".format(*headers))
    lines.append("-" * 100)
    for item in records:
        lines.append(
            "{:<14} {:<12} {:>8.4f} {:>8.4f} {:>8.4f} {:>9.4f} {:>7.2f} {:>8.3f} {:>10.4f} {:>6.2f}".format(
                item["label"], item["scenario"], item["mean_abs"], item["rms"],
                item["max_abs"], item["steady_abs"], item["convergence_s"],
                item["offroad_ratio"], item["steer_rate_rms"], item["metric_rank"],
            )
        )
    lines.extend(["", "Overall ranking (mean metric rank across scenarios)", "-" * 78])
    for item in summary:
        lines.append("{:d}. {:<16} mean rank = {:.3f}".format(
            item["overall_rank"], item["label"], item["mean_rank"]))
    lines.extend([
        "",
        "Notes:",
        "- e > 0 means the vehicle is left of the reference path.",
        "- Convergence: first time |e| <= 0.20 m and stays inside until the end.",
        "- Steady-state: mean |e| over the last 25% of the run.",
        "- Off-road: fraction of samples with |e| > 0.80 m.",
        "- Rank: average rank over 7 metrics within each scenario, then averaged.",
    ])
    return "\n".join(lines) + "\n"
