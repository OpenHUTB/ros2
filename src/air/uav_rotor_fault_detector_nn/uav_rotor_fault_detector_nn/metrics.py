"""检测效果的评测口径。

检测器和控制器的"好"不是一回事：控制器看位置误差，检测器看的是

* **估计精度**：四桨增益的平均绝对误差；
* **发现速度**：故障注入后多久报警（给容错控制器留出的反应时间）；
* **误报**：健康飞行时会不会乱报警。

三个指标互相拉扯——只盯精度可以靠"一直报故障"骗过去，只盯误报可以靠"从不报警"骗过去。
所以报告里必须同时给出，缺一条都不能算数。
"""

from __future__ import annotations

import numpy as np

from .features import ROTOR_COUNT

#: 判定"这一桨已经坏了"的增益阈值（真值侧）
FAULT_GAIN_THRESHOLD = 0.9
#: 判定"检测器认为这一桨坏了"的估计阈值（估计侧）
DETECT_GAIN_THRESHOLD = 0.8


def mean_absolute_error(pred: np.ndarray, target: np.ndarray) -> float:
    """四桨增益的平均绝对误差。"""
    pred = np.asarray(pred, dtype=np.float64)
    target = np.asarray(target, dtype=np.float64)
    return float(np.mean(np.abs(pred - target)))


def per_rotor_mae(pred: np.ndarray, target: np.ndarray) -> np.ndarray:
    """逐桨 MAE，形状 ``(4,)``。"""
    pred = np.asarray(pred, dtype=np.float64)
    target = np.asarray(target, dtype=np.float64)
    return np.mean(np.abs(pred - target), axis=0)


def faulty_mask(target: np.ndarray) -> np.ndarray:
    """真值里"这一帧存在故障桨"的样本掩码，形状 ``(n,)``。

    只要有任意一桨增益低于阈值，就认为这一帧是故障帧。
    """
    target = np.asarray(target, dtype=np.float64).reshape(-1, ROTOR_COUNT)
    return target.min(axis=1) < FAULT_GAIN_THRESHOLD


def predicted_faulty_mask(pred: np.ndarray) -> np.ndarray:
    """检测器判断"存在故障桨"的样本掩码。"""
    pred = np.asarray(pred, dtype=np.float64).reshape(-1, ROTOR_COUNT)
    return pred.min(axis=1) < DETECT_GAIN_THRESHOLD


def confusion(pred: np.ndarray, target: np.ndarray) -> dict:
    """按"整帧是否含故障"做的二分类混淆矩阵（健康 / 故障）。"""
    truth = faulty_mask(target)
    guess = predicted_faulty_mask(pred)
    tp = int(np.sum(truth & guess))
    fn = int(np.sum(truth & ~guess))
    fp = int(np.sum(~truth & guess))
    tn = int(np.sum(~truth & ~guess))

    def ratio(a: int, b: int) -> float:
        return float(a) / float(b) if b else 0.0

    return {
        "tp": tp,
        "fp": fp,
        "tn": tn,
        "fn": fn,
        "recall": ratio(tp, tp + fn),
        "precision": ratio(tp, tp + fp),
        "false_alarm_rate": ratio(fp, fp + tn),
        "accuracy": ratio(tp + tn, tp + fp + tn + fn),
    }


def detection_latency(
    pred: np.ndarray,
    target: np.ndarray,
    ends: np.ndarray,
    dt: float,
    faulty_rotors: np.ndarray | None = None,
    threshold: float = DETECT_GAIN_THRESHOLD,
) -> float:
    """故障注入后过多久才报警，单位秒；一直没报警返回 ``inf``。

    ``ends`` 是每个样本对应的时间下标（``make_windows`` 的第 3 个输出），
    用它可以把样本序号换算回仿真时间。
    """
    pred = np.asarray(pred, dtype=np.float64).reshape(-1, ROTOR_COUNT)
    target = np.asarray(target, dtype=np.float64).reshape(-1, ROTOR_COUNT)
    ends = np.asarray(ends, dtype=np.int64)

    onset_index = None
    rotor = None
    for i in range(len(target)):
        gains = target[i]
        if gains.min() < FAULT_GAIN_THRESHOLD:
            onset_index = i
            rotor = int(np.argmin(gains))
            break
    if onset_index is None or rotor is None:
        return float("nan")
    if faulty_rotors is not None and rotor not in set(int(r) for r in faulty_rotors):
        rotor = int(np.argmin(target[onset_index]))

    onset_time = float(ends[onset_index]) * dt
    for i in range(onset_index, len(target)):
        if pred[i, rotor] < threshold:
            return max(0.0, float(ends[i]) * dt - onset_time)
    return float("inf")


def summarize(
    pred: np.ndarray,
    target: np.ndarray,
    ends: np.ndarray,
    dt: float,
    groups: np.ndarray | None = None,
) -> dict:
    """一次性算齐报告要用的指标，``groups`` 为逐样本所属故障类型。"""
    result = {
        "mae": mean_absolute_error(pred, target),
        "per_rotor_mae": [float(v) for v in per_rotor_mae(pred, target)],
        "confusion": confusion(pred, target),
        "n_samples": int(len(pred)),
    }

    if groups is not None:
        groups = np.asarray(groups).astype(str)
        per_group = {}
        for kind in sorted(set(groups.tolist())):
            mask = groups == kind
            if not np.any(mask):
                continue
            latency = detection_latency(pred[mask], target[mask], ends[mask], dt)
            per_group[kind] = {
                "n_samples": int(mask.sum()),
                "mae": mean_absolute_error(pred[mask], target[mask]),
                "latency": None if np.isnan(latency) else (
                    "n/a" if np.isinf(latency) else round(latency, 3)
                ),
                "recall": confusion(pred[mask], target[mask])["recall"],
                "false_alarm_rate": confusion(pred[mask], target[mask])["false_alarm_rate"],
            }
        result["per_group"] = per_group

    return result
