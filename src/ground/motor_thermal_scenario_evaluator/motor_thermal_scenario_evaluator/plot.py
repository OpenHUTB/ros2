"""Plot CSV curves produced by the independent scenario evaluator."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
from matplotlib import font_manager
import matplotlib.pyplot as plt

for font_path in ("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", "C:/Windows/Fonts/msyh.ttc"):
    if Path(font_path).exists():
        font_manager.fontManager.addfont(font_path)
        plt.rcParams["font.family"] = font_manager.FontProperties(fname=font_path).get_name()
        plt.rcParams["axes.unicode_minus"] = False
        break


def plot_curves(csv_path: str | Path, output_path: str | Path) -> None:
    with Path(csv_path).open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    scenarios = list(dict.fromkeys(row["scenario"] for row in rows))
    scenario_labels = {"cool_cruise": "常温巡航", "warm_turn": "暖环境转弯", "hot_high_load": "高温高负载"}
    mode_labels = {"fixed_speed": "固定速度", "derated": "温度降额"}
    figure, axes = plt.subplots(len(scenarios), 1, figsize=(8, 3 * len(scenarios)), sharex=True, constrained_layout=True)
    if len(scenarios) == 1:
        axes = [axes]
    for axis, scenario in zip(axes, scenarios):
        for mode, color in (("fixed_speed", "#2563a6"), ("derated", "#d97706")):
            selected = [row for row in rows if row["scenario"] == scenario and row["mode"] == mode]
            axis.plot([float(row["time_s"]) for row in selected], [float(row["temperature_c"]) for row in selected], label=mode_labels[mode], color=color)
        axis.set(title=scenario_labels.get(scenario, scenario), ylabel="电机温度（°C）")
        axis.grid(alpha=0.25)
        axis.legend(frameon=False)
    axes[-1].set_xlabel("时间（秒）")
    figure.savefig(output_path, dpi=160)
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", default="output/thermal_evaluation/scenario_curves.csv")
    parser.add_argument("--output", default="docs/img/ground/motor_thermal_scenario_evaluator/scenario_curves.png")
    args = parser.parse_args()
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    plot_curves(args.csv, args.output)


if __name__ == "__main__":
    main()
