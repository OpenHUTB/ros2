#!/usr/bin/env python3
"""多种子基准：为文档生成对照表格（Markdown）与原始数据（CSV）。

    python sim/benchmark.py --seeds 10
"""
from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE.parent / "src"), str(HERE.parent)]

import numpy as np  # noqa: E402

from sim.experiments import run_episode, summarise  # noqa: E402
from sim.scenarios import ALL_SCENARIOS, SCENARIOS  # noqa: E402

EXPECT_ZH = {"escape": "脱困后继续前进", "abort": "停车上报", "no_trigger": "不误报"}


def collect(seeds: int) -> list[dict]:
    rows = []
    for name in ALL_SCENARIOS:
        for rec in (False, True):
            items = [summarise(run_episode(name, seed=s, use_recovery=rec)) for s in range(seeds)]
            det = [i["first_detect"] for i in items if not math.isnan(i["first_detect"])]
            esc = [i["escape_time"] - i["first_detect"] for i in items if i["escape_time"] >= 0]
            rows.append(
                {
                    "scenario": name,
                    "title": SCENARIOS[name].title,
                    "expect": EXPECT_ZH[SCENARIOS[name].expect],
                    "recovery": rec,
                    "seeds": seeds,
                    "success": sum(1 for i in items if i["success"]),
                    "triggers": float(np.mean([i["triggers"] for i in items])),
                    "detect": float(np.mean(det)) if det else float("nan"),
                    "escape_delay": float(np.mean(esc)) if esc else float("nan"),
                    "post_travel": float(np.mean([i["post_travel"] for i in items])),
                    "stuck_ratio": float(np.mean([i["stuck_ratio"] for i in items])),
                }
            )
    return rows


def as_markdown(rows: list[dict]) -> str:
    head = (
        "| 场景 | 期望行为 | 脱困层 | 达成 | 触发次数 | 首次检测/s | 脱困耗时/s | 脱困后里程/m | 卡死时间占比 |\n"
        "|---|---|---|---|---|---|---|---|---|\n"
    )
    body = ""
    for r in rows:
        det = "-" if math.isnan(r["detect"]) else f"{r['detect']:.2f}"
        esc = "-" if math.isnan(r["escape_delay"]) else f"{r['escape_delay']:.2f}"
        body += (
            f"| {r['title']} | {r['expect']} | {'开' if r['recovery'] else '关'} | "
            f"{r['success']}/{r['seeds']} | {r['triggers']:.1f} | {det} | {esc} | "
            f"{r['post_travel']:.2f} | {r['stuck_ratio']:.1%} |\n"
        )
    return head + body


def main() -> None:
    ap = argparse.ArgumentParser(description="卡死检测与脱困：多种子基准")
    ap.add_argument("--seeds", type=int, default=10)
    ap.add_argument("--out", default=str(HERE.parent / "docs_figs"))
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    rows = collect(args.seeds)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    with open(out / "benchmark.csv", "w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    table = as_markdown(rows)
    (out / "benchmark.md").write_text(table, encoding="utf-8")
    if not args.quiet:
        print(table)
        print(f"已写入 {out / 'benchmark.md'} 与 benchmark.csv")


if __name__ == "__main__":
    main()
