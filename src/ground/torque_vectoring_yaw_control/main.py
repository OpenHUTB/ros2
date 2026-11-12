#!/usr/bin/env python3
"""离线入口：跑场景、打印指标、可选画图。

用法::

    python main.py                    # 四个场景 × 有/无扭矩矢量
    python main.py --scenario over_limit
    python main.py --plots ./fig
"""

from __future__ import annotations

import argparse
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "src"))

from sim import scenarios  # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser(description="扭矩矢量横摆控制：离线实验入口")
    ap.add_argument("--scenario", default="all")
    ap.add_argument("--plots", default=None)
    args = ap.parse_args()

    keys = list(scenarios.SCENARIOS) if args.scenario == "all" else [args.scenario]
    for k in keys:
        if k not in scenarios.SCENARIOS:
            raise SystemExit(f"未知场景：{k}")

    print(f"  {'场景':<12}{'横摆RMS(无)':>12}{'横摆RMS(有)':>12}{'改善':>8}"
          f"{'侧偏角(无→有)':>16}{'峰值利用率':>14}")
    print("  " + "-" * 76)
    for k in keys:
        sc = scenarios.SCENARIOS[k]
        a = scenarios.run(k, False)
        b = scenarios.run(k, True)
        gain = 100.0 * (1.0 - b.yaw_rms_error / max(a.yaw_rms_error, 1e-9))
        print(f"  {sc.title:<10}{a.yaw_rms_error:>14.4f}{b.yaw_rms_error:>14.4f}"
              f"{gain:>7.1f}%{a.max_sideslip_deg:>9.1f} → {b.max_sideslip_deg:<5.1f}"
              f"{a.utilization.max():>7.2f}/{b.utilization.max():<5.2f}")

    if args.plots:
        from sim import experiments

        out = pathlib.Path(args.plots)
        out.mkdir(parents=True, exist_ok=True)
        for fn in experiments.PLOTS:
            fn(out)
        print(f"\n图片已写入 {out}")


if __name__ == "__main__":
    main()
