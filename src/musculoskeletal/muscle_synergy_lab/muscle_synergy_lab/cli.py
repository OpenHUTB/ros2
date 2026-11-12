# -*- coding: utf-8 -*-
"""把协同分解结果排成 Markdown 表格。"""

from typing import Dict, List, Optional, Sequence

from .core import k_for_vaf, nmf, recovery_check, subspace_angle_deg, synth_data, vaf


def table_scan(rows: Sequence[Dict[str, float]]) -> str:
    lines = ["| 协同数 k | VAF（重建率） | 重建误差 SSE | 相邻 VAF 增量 |",
             "|---|---|---|---|"]
    prev = 0.0
    for r in rows:
        lines.append("| %d | %.4f | %.4f | %+.4f |"
                     % (r["k"], r["vaf"], r["sse"], r["vaf"] - prev))
        prev = r["vaf"]
    return "\n".join(lines)


def table_recovery(E, W_true, ks: Sequence[int]) -> str:
    lines = ["| k | VAF | 与真值协同的夹角 (°) | 判断 |", "|---|---|---|---|"]
    for k in ks:
        r = recovery_check(E, W_true, k)
        if k < W_true.shape[1]:
            note = "协同数不足：VAF 明显偏低（夹角仍小——子空间嵌套，夹角抓不住少选）"
        elif k == W_true.shape[1]:
            note = "**刚好**：VAF 与增量都到位"
        else:
            note = "过参数化：VAF 增量趋近 0（夹角同样小，区分不出来）"
        lines.append("| %d | %.4f | %.2f | %s |"
                     % (r["k"], r["vaf"], r["angle_deg"], note))
    return "\n".join(lines)


def table_synergy_vectors(W: List[List[float]], names: List[str]) -> str:
    """把估计出的协同权重按肌肉列成表，便于看出"哪个协同用哪几块肌肉"。"""
    k = len(W[0])
    head = "| 肌肉 | " + " | ".join("协同 %d" % (j + 1) for j in range(k)) + " |"
    lines = [head, "|" + "---|" * (k + 1)]
    for i, name in enumerate(names):
        cells = " | ".join("%.2f" % W[i][j] for j in range(k))
        lines.append("| `%s` | %s |" % (name, cells))
    return "\n".join(lines)


def json_summary(rows, E, W_true, k_true: int) -> Dict:
    rec = recovery_check(E, W_true, k_true)
    return {
        "n_muscles": int(E.shape[0]),
        "n_samples": int(E.shape[1]),
        "k_true": int(k_true),
        "k_for_vaf_95": k_for_vaf(rows, 0.95),
        "vaf_at_k_true": rec["vaf"],
        "angle_at_k_true_deg": rec["angle_deg"],
        "scan": [{"k": r["k"], "vaf": r["vaf"]} for r in rows],
    }
