"""生成文档用的图：模式对比动图、生存率/误差柱状图、分配误差直方图。

用法（在包目录下）：

    python tools/gen_figures.py            # 全部
    python tools/gen_figures.py --skip-gif # 只出静态图（快）
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent          # <pkg>/tools
_PKG_DIR = _HERE.parent                          # <pkg>
_REPO_ROOT = _HERE.parents[3]                    # 仓库根：tools → 包 → air → src → 仓库
sys.path.insert(0, str(_PKG_DIR))

IMG_DIR = _REPO_ROOT / "docs" / "img" / "air" / "uav_neural_fault_tolerant_control"


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="生成 PR3 文档图")
    parser.add_argument("--checkpoint", default=str(_PKG_DIR / "results" / "allocator.pt"))
    parser.add_argument("--dataset", default=str(_PKG_DIR / "results" / "allocator_dataset.npz"))
    parser.add_argument("--skip-gif", action="store_true")
    args = parser.parse_args(argv)

    IMG_DIR.mkdir(parents=True, exist_ok=True)

    from uav_neural_fault_tolerant_control import data as D
    from uav_neural_fault_tolerant_control import evaluate as E
    from uav_neural_fault_tolerant_control import faults as FL
    from uav_neural_fault_tolerant_control import model as M
    from uav_neural_fault_tolerant_control import render as R
    from uav_neural_fault_tolerant_control._compat import load_base

    net, meta = M.load_checkpoint(args.checkpoint)
    print(f"权重：{meta.get('n_parameters')} 个参数")

    dataset = D.load_dataset(args.dataset)
    path = R.make_allocator_png(dataset, net, IMG_DIR / "allocation_error.png")
    print(f"已生成 {path}")

    api = load_base()
    rows = E.flight_table(api, trajectories=("hover", "zigzag"), net=net)
    path = R.make_summary_png(rows, IMG_DIR / "mode_summary.png")
    print(f"已生成 {path}")

    if not args.skip_gif:
        plan = FL.FaultPlan(kind="degradation", rotor=1, onset=2.0, severity=0.5)
        path = R.make_altitude_gif(IMG_DIR / "fault_response.gif", plan=plan,
                                   trajectory="hover", seconds=8.0, fps=10,
                                   net=net, api=api)
        print(f"已生成 {path}")

    tab = E.allocator_table(dataset, net)
    print("分配误差：", {k: tab[k] for k in ("n_samples", "command_mae_n",
                                            "residual_mae", "neural_ms")})
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
