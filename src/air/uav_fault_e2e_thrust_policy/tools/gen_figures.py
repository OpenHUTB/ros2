"""生成文档配图：三种网络结构的对比图 + （可选）演示动图。

    python tools/gen_figures.py            # 只出静态图
    python tools/gen_figures.py --gif      # 顺带出演示动图
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
_PKG_DIR = _HERE.parent
_REPO_ROOT = _HERE.parents[3]
_IMG_DIR = _REPO_ROOT / "docs" / "img" / "air" / "uav_fault_e2e_thrust_policy"

sys.path.insert(0, str(_PKG_DIR))

from uav_fault_e2e_thrust_policy import render as R  # noqa: E402


def _load_summary(path: Path):
    with path.open("r", encoding="utf-8") as fh:
        payload = json.load(fh)
    return payload.get("summary", payload.get("aggregate", payload))


def summary_figure(summary: dict, out_path: Path) -> Path:
    """成功率 / 末端进度 / 指令偏离 三联图。"""
    import numpy as np

    plt = R._pyplot()
    modes = [m for m in ("expert", "mlp", "cnn", "cnn_stack")
             if m in summary and summary[m].get("n", 0) > 0]
    colors = {"expert": "#4C72B0", "mlp": "#8172B3", "cnn": "#55A868",
              "cnn_stack": "#C44E52"}
    fig, axes = plt.subplots(1, 3, figsize=(11.0, 3.0), dpi=90)
    bars = [
        ("success rate", [summary[m]["success_rate"] for m in modes], 1.05),
        ("end progress (m)", [summary[m]["mean_final_x"] for m in modes], 10.5),
        ("deviation from expert cmd", [summary[m]["mean_cmd_dev"] for m in modes], None),
    ]
    for ax, (title, values, top) in zip(axes, bars):
        ax.bar(modes, values, color=[colors.get(m, "#4C72B0") for m in modes])
        ax.set_title(title, fontsize=9)
        ax.tick_params(labelsize=8)
        if top is not None:
            ax.set_ylim(0.0, top)
        for i, v in enumerate(values):
            ax.text(i, v, f"{v:.2f}", ha="center", va="bottom", fontsize=7)
    n = summary[modes[0]]["n"] if modes else 0
    fig.suptitle(f"end-to-end thrust policy ({n} flights per mode, "
                 f"incl. fault cases)", fontsize=9)
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path)
    plt.close(fig)
    return out_path


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="生成 PR5 文档配图")
    ap.add_argument("--json", default=str(_PKG_DIR / "results" / "policy_comparison.json"))
    ap.add_argument("--gif", action="store_true", help="顺带出演示动图")
    ap.add_argument("--checkpoint", default=str(_PKG_DIR / "results" / "policy_cnn.pt"))
    ap.add_argument("--fault", default="degradation")
    ap.add_argument("--rotor", type=int, default=2)
    ap.add_argument("--severity", type=float, default=0.5)
    args = ap.parse_args(argv)

    json_path = Path(args.json)
    if not json_path.is_file():
        print(f"缺少 {json_path}，先跑 python main.py --eval")
        return 2
    summary = _load_summary(json_path)
    out = summary_figure(summary, _IMG_DIR / "policy_comparison.png")
    print(f"已生成 {out}（{out.stat().st_size} B）")

    if args.gif:
        from uav_fault_e2e_thrust_policy import _compat, policy as P

        ckpt = Path(args.checkpoint)
        if not ckpt.is_file():
            print(f"缺少权重 {ckpt}，跳过动图")
            return 0
        module, _ = P.load_checkpoint(ckpt)
        api = _compat.load_base()
        spec = {"kind": args.fault, "rotor": args.rotor, "severity": args.severity,
                "onset": 2.5}
        gif = R.make_policy_gif(_IMG_DIR / "fault_policy.gif", api=api,
                                policy_module=module, spec=spec, seed=0, fps=10,
                                arch="cnn")
        print(f"已生成 {gif}（{gif.stat().st_size} B）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
