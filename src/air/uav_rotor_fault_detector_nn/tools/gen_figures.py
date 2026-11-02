"""生成文档用的静态图（训练曲线、分故障类型指标）。

    python tools/gen_figures.py

依赖 matplotlib；缺了会打印提示并跳过，不让整体失败。
产物默认写到 ``docs/img/air/uav_rotor_fault_detector_nn/``。

图里的文字会优先用中文；找不到中文字体时自动退回英文，避免出现方块。
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
_REPO_ROOT = _HERE.parents[3]
OUT_DIR = _REPO_ROOT / "docs" / "img" / "air" / "uav_rotor_fault_detector_nn"
PKG_DIR = _HERE.parent

# 直接 ``python tools/gen_figures.py`` 运行时，包目录不在 sys.path 上，手动补一下
if str(PKG_DIR) not in sys.path:
    sys.path.insert(0, str(PKG_DIR))

DEFAULT_CHECKPOINT = PKG_DIR / "results" / "detector.pt"
DEFAULT_METRICS = PKG_DIR / "results" / "detector_metrics.json"

_CJK_FONTS = (
    "Microsoft YaHei",
    "SimHei",
    "Noto Sans CJK SC",
    "Noto Sans SC",
    "Source Han Sans SC",
    "WenQuanYi Zen Hei",
    "PingFang SC",
    "Heiti SC",
    "Arial Unicode MS",
)


def _prepare_matplotlib():
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib import font_manager
    except ImportError:
        print("未安装 matplotlib，跳过静态图生成（pip install matplotlib）")
        return None, False

    has_cjk = False
    available = {font.name for font in font_manager.fontManager.ttflist}
    for name in _CJK_FONTS:
        if name in available:
            plt.rcParams["font.family"] = "sans-serif"
            plt.rcParams["font.sans-serif"] = [name] + list(plt.rcParams["font.sans-serif"])
            plt.rcParams["axes.unicode_minus"] = False
            has_cjk = True
            break

    plt.rcParams["font.size"] = 9
    plt.rcParams["axes.grid"] = True
    plt.rcParams["grid.alpha"] = 0.3
    return plt, has_cjk


def _series_from_history(meta: dict) -> dict[str, list[float]]:
    """把检查点里的 history 统一成 {名字: 数值列表}。"""
    history = meta.get("history")
    series: dict[str, list[float]] = {}
    if isinstance(history, dict):
        for key, value in history.items():
            if isinstance(value, (list, tuple)) and value and isinstance(value[0], (int, float)):
                series[str(key)] = [float(v) for v in value]
    elif isinstance(history, list) and history and isinstance(history[0], dict):
        keys = [k for k in history[0] if isinstance(history[0][k], (int, float))]
        for key in keys:
            series[str(key)] = [float(row[key]) for row in history]
    return series


def plot_training_curve(checkpoint: Path, plt, has_cjk: bool) -> Path | None:
    from uav_rotor_fault_detector_nn.model import load_checkpoint

    if not checkpoint.is_file():
        print(f"找不到检查点 {checkpoint}，跳过训练曲线")
        return None

    _, meta = load_checkpoint(checkpoint)
    series = _series_from_history(meta)
    if not series:
        print("检查点里没有 history，跳过训练曲线")
        return None

    loss_keys = [k for k in series if "loss" in k.lower()]
    mae_keys = [k for k in series if "mae" in k.lower()]
    if not loss_keys and not mae_keys:
        print("history 里既没有 loss 也没有 mae，跳过训练曲线")
        return None

    fig, ax1 = plt.subplots(figsize=(7.2, 3.0), dpi=110)
    for key in loss_keys:
        epochs = range(1, len(series[key]) + 1)
        ax1.plot(epochs, series[key], label=key, linewidth=1.4)
    ax1.set_xlabel("epoch")
    ax1.set_ylabel("训练损失" if has_cjk else "training loss")
    ax1.set_yscale("log")

    if mae_keys:
        ax2 = ax1.twinx()
        for key in mae_keys:
            epochs = range(1, len(series[key]) + 1)
            ax2.plot(epochs, series[key], linestyle="--", label=key, linewidth=1.4)
        ax2.set_ylabel("验证 MAE" if has_cjk else "validation MAE")
        ax2.grid(False)
        handles1, labels1 = ax1.get_legend_handles_labels()
        handles2, labels2 = ax2.get_legend_handles_labels()
        ax1.legend(handles1 + handles2, labels1 + labels2, loc="upper right", fontsize=8)
    else:
        ax1.legend(loc="upper right", fontsize=8)

    best = meta.get("best_val_mae")
    params = meta.get("n_parameters", 0)
    if isinstance(best, (int, float)):
        title = (f"训练曲线（最佳验证 MAE {best:.4f}，参数量 {params}）" if has_cjk
                 else f"Training curve (best val MAE {best:.4f}, {params} params)")
        ax1.set_title(title)
    else:
        ax1.set_title("训练曲线" if has_cjk else "Training curve")
    fig.tight_layout()

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    path = OUT_DIR / "training_curve.png"
    fig.savefig(path)
    plt.close(fig)
    print(f"已生成 {path}（{path.stat().st_size / 1024:.1f} KB）")
    return path


def _find_per_group(payload) -> dict | None:
    """在评测报告里找到 per_group（结构可能嵌套一层）。"""
    if isinstance(payload, dict):
        if "per_group" in payload and isinstance(payload["per_group"], dict):
            return payload["per_group"]
        for value in payload.values():
            found = _find_per_group(value)
            if found:
                return found
    return None


def plot_detection_by_fault(metrics_path: Path, plt, has_cjk: bool) -> Path | None:
    if not metrics_path.is_file():
        print(f"找不到评测报告 {metrics_path}，跳过分组柱状图")
        return None

    with metrics_path.open("r", encoding="utf-8") as handle:
        payload = json.load(handle)
    per_group = _find_per_group(payload)
    if not per_group:
        print("评测报告里没有 per_group，跳过分组柱状图")
        return None

    kinds = sorted(per_group)
    recalls = [float(per_group[k].get("recall") or 0.0) for k in kinds]
    latencies = []
    for kind in kinds:
        value = per_group[kind].get("latency")
        latencies.append(float(value) if isinstance(value, (int, float)) else 0.0)

    fig, axes = plt.subplots(1, 2, figsize=(9.0, 2.9), dpi=110)
    positions = list(range(len(kinds)))

    axes[0].barh(positions, recalls, color="#4C72B0")
    axes[0].set_yticks(positions)
    axes[0].set_yticklabels(kinds, fontsize=8)
    axes[0].set_xlim(0.0, 1.05)
    axes[0].set_xlabel("召回率" if has_cjk else "recall")
    axes[0].set_title("分故障类型的召回率" if has_cjk else "Recall by fault type")

    axes[1].barh(positions, latencies, color="#DD8452")
    axes[1].set_yticks(positions)
    axes[1].set_yticklabels(kinds, fontsize=8)
    # 右侧留白，否则最长那根柱子的数值标签会被坐标轴裁掉
    axes[1].set_xlim(0.0, max(latencies + [0.1]) * 1.3)
    axes[1].set_xlabel("检测延时 / s" if has_cjk else "detection latency / s")
    axes[1].set_title("故障注入到报警的延时" if has_cjk else "Latency from onset to alarm")
    for index, value in enumerate(latencies):
        axes[1].text(value + 0.02, index, f"{value:.2f}", va="center", fontsize=7)

    fig.tight_layout()
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    path = OUT_DIR / "detection_by_fault.png"
    fig.savefig(path)
    plt.close(fig)
    print(f"已生成 {path}（{path.stat().st_size / 1024:.1f} KB）")
    return path


def main(argv: list[str] | None = None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    checkpoint = Path(argv[0]) if len(argv) > 0 else DEFAULT_CHECKPOINT
    metrics = Path(argv[1]) if len(argv) > 1 else DEFAULT_METRICS

    plt, has_cjk = _prepare_matplotlib()
    if plt is None:
        return 0

    produced = [
        plot_training_curve(checkpoint, plt, has_cjk),
        plot_detection_by_fault(metrics, plt, has_cjk),
    ]
    if not any(produced):
        print("没有生成任何图，检查检查点与评测报告是否存在")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
