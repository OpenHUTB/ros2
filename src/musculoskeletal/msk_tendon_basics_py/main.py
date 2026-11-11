#!/usr/bin/env python
"""肌肉-肌腱串联弹性实验复现入口。

用法::

    python main.py
    python main.py --quick
    python main.py --outdir out

不依赖 ROS / MuJoCo；没有 matplotlib 时会自动跳过 PNG，仍生成 CSV 和
summary.txt。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

# 允许从任意工作目录用 `python <path>/main.py` 启动。
_PROJECT_ROOT = Path(__file__).resolve().parent
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

# 被管道/CI 捕获时用 UTF-8，避免 Windows 默认 cp936 把中文日志写成乱码；
# 交互式终端保持系统编码，直接在控制台运行时不会乱码。
for _stream in (sys.stdout, sys.stderr):
    if hasattr(_stream, "reconfigure") and not _stream.isatty():
        try:
            _stream.reconfigure(encoding="utf-8")
        except Exception:
            pass

from scripts.experiments import run_all  # noqa: E402


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="复现 OpenHUTB 肌肉-肌腱串联弹性实验（纯 Python）"
    )
    parser.add_argument(
        "--quick", action="store_true",
        help="减少激活扫描点、放大积分步长，用于快速检查",
    )
    parser.add_argument(
        "--outdir", default="out",
        help="输出目录（默认 out）",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    outdir = Path(args.outdir)
    try:
        result = run_all(outdir=outdir, quick=args.quick)
    except Exception as exc:  # 明确报错并以非零退出，方便 CI 发现
        print(f"运行失败: {exc}", file=sys.stderr)
        return 1

    iso = result["isometric"]
    stiff = result["stiffness"]
    release = result["release"]
    elastic = release["elastic"]
    rigid = release["rigid"]

    import numpy as np

    print("=" * 64)
    print("肌肉-肌腱串联弹性实验复现完成")
    print("=" * 64)
    for a in (0.5, 1.0):
        idx = int(np.argmin(np.abs(iso["activations"] - a)))
        print(
            f"实验1 a={a:.1f}: 弹性 {iso['elastic_force'][idx]:.2f} N / "
            f"刚性 {iso['rigid_force'][idx]:.2f} N, "
            f"损失 {iso['loss_pct'][idx]:.2f}%, "
            f"l_m/l_opt={iso['elastic_fiber_length'][idx]:.4f}, "
            f"应变={iso['elastic_strain_pct'][idx]:.2f}%, "
            f"储能={iso['elastic_energy'][idx]:.4f} J"
        )
    for i, multiplier in enumerate(stiff["multipliers"]):
        print(
            f"实验2 k={multiplier:.1f}x ({stiff['stiffness'][i]:.0f} N/m): "
            f"F={stiff['force'][i]:.2f} N, 应变={stiff['strain_pct'][i]:.2f}%, "
            f"储能={stiff['energy'][i]:.3f} J"
        )
    print(
        f"实验3 松手前: F={elastic.pre_force:.2f} N, "
        f"E={elastic.pre_energy:.4f} J; "
        f"最低 E={elastic.min_energy:.4f} J, "
        f"释放 {100.0 * elastic.release_fraction:.1f}%, "
        f"峰值角={elastic.peak_angle:.3f} rad (刚性 {rigid.peak_angle:.3f} rad)"
    )
    print("-" * 64)
    print(f"输出目录: {result['outdir'].resolve()}")
    print("PNG: " + ", ".join(
        f"{name}={'OK' if ok else '跳过'}" for name, ok in result["plots"].items()
    ))
    print("CSV: isometric.csv, tendon_stiffness.csv, release.csv")
    print("汇总: summary.txt")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
