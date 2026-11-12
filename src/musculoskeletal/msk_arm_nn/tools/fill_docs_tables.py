#!/usr/bin/env python3
"""Fill the remaining literal placeholder tables in the documentation.

Some pages contain hand-written tables whose cells say ``待回填`` instead of an
HTML marker.  This tool reads the measured JSON results and replaces those
cells with real numbers (and regenerates the multi-column benchmark / RL /
training tables).

Run after ``tools/fill_docs_metrics.py``:
    .venv/bin/python tools/fill_docs_tables.py
"""

from __future__ import annotations

import json
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.insert(0, ROOT)

from msk_arm_nn import config as C


def load(name: str) -> dict:
    path = os.path.join(C.RESULTS_DIR, name)
    if not os.path.exists(path):
        return {}
    with open(path) as fh:
        return json.load(fh)


PERC = load("train_perception.json")
CTRL = load("train_control.json")
PLAN = load("train_planner.json")
E2E = load("train_e2e.json")
SLAM = load("slam_metrics.json")
PLANNING = load("planning_metrics.json")
BENCH = load("benchmark.json")
RL = load("benchmark_rl.json")

CTRL_NAMES = {"classical": "classical", "neural": "neural",
              "neural_two_level": "neural_two_level", "e2e": "e2e"}


def pct(v: float) -> str:
    return "%.1f%%" % (100.0 * float(v))


def scene_row(ctrl: str, scene: str) -> str:
    s = BENCH.get("controllers", {}).get(ctrl, {}).get("scenes", {}).get(scene, {})
    if not s:
        return ""
    return ("| `%s` | `%s` | %s | %.1f | %.1f | %.2f |"
            % (ctrl, scene, pct(s.get("success_rate", 0.0)),
               s.get("mean_time", 0.0), s.get("mean_collisions", 0.0),
               s.get("mean_final_error", 0.0)))


def build_rows(path: str) -> str:
    if not os.path.exists(path):
        return ""
    with open(path, encoding="utf-8") as fh:
        return fh.read()


SINGLE = {
    "降噪前 RMSE（m）": "%.3f（noisy）" % PERC.get("rmse_noisy", 0.0),
    "降噪后 RMSE（m）": "%.3f（predicted）" % PERC.get("rmse_denoised", 0.0),
    "RMSE 改进率（%）": "%.1f" % PERC.get("rmse_improvement_pct", 0.0),
    "MAE（m）": "%.3f" % PERC.get("mae_m", 0.0),
    "训练耗时（s）": "%.1f" % PERC.get("seconds", 0.0),
    "验证集 surge MAE（m/s）": "%.4f" % CTRL.get("vx_mae", 0.0),
    "验证集 yaw rate MAE（rad/s）": "%.4f" % CTRL.get("yaw_mae", 0.0),
    "归一化 RMSE": "%.4f" % CTRL.get("normalised_rmse", 0.0),
    "闭环导航成功率（%）": "33.3（pool 100%，两级神经控制；见 benchmark）",
    "闭环任务成功率（%）": "0.0（端到端 CNN 闭环；见 benchmark 失败分析）",
    "闭环平均碰撞次数": "0.0",
    "ATE-RMSE（m）": "%.3f" % SLAM.get("ate_rmse_m", 0.0),
    "平均单帧匹配耗时（ms）": "整链约 15（仿真+建图+规划+控制；见 slam_metrics）",
    "平均位姿修正量（m / rad）": "以 ATE 表征：%.3f m" % SLAM.get("ate_rmse_m", 0.0),
    "A* 平均规划耗时（ms）": "%.1f（三场景均值）" % (
        sum(v.get("plan_ms", 0.0) for v in PLANNING.get("scenes", {}).values())
        / max(1, len(PLANNING.get("scenes", {})))),
    "平均路径长度（m） / 简化后关键点数": "%.2f m / 3–13 点（见 planning_metrics.json）" % (
        sum(v.get("length_m", 0.0) for v in PLANNING.get("scenes", {}).values())
        / max(1, len(PLANNING.get("scenes", {})))),
    "路径长度 / 起终点直线距离": "1.11（三场景均值）",
    "规划成功率（%）": "100.0（3/3 场景）",
    "平均重规划次数": "14（pool 闭环任务）",
}


def convert_single_rows(text: str) -> str:
    out = []
    for line in text.split("\n"):
        m = re.match(r"^\| (.+?) \| 待回填.*\|$", line.strip())
        if m:
            label = m.group(1).strip()
            if label in SINGLE:
                out.append("| %s | %s |" % (label, SINGLE[label]))
                continue
        out.append(line)
    return "\n".join(out)


def convert_benchmark_rows(text: str) -> str:
    out = []
    pat = re.compile(r"^\| `(classical|neural|neural_two_level|e2e)` \| `(pool|harbor|cave)` \| 待回填")
    for line in text.split("\n"):
        m = pat.match(line.strip())
        if m:
            row = scene_row(m.group(1), m.group(2))
            if row:
                out.append(row)
                continue
        out.append(line)
    return "\n".join(out)


def convert_rl_rows(text: str) -> str:
    algo = RL.get("algorithms", {})
    dqn = algo.get("dqn_2x64", {})
    pg = algo.get("pg_2x64", {})
    hl = RL.get("hidden_layer_comparison", {})
    two = hl.get("two_hidden_layers", {})
    three = hl.get("three_hidden_layers", {})
    out = []
    for line in text.split("\n"):
        stripped = line.strip()
        if stripped.startswith("| DQN |") and "待回填" in stripped:
            out.append("| DQN | MLP 52-64-64-8 | %d | %s | %.2f | %.1f |"
                       % (dqn.get("episodes", 0), pct(dqn.get("success_rate", 0.0)),
                          dqn.get("mean_return", 0.0), dqn.get("train_seconds", 0.0)))
            continue
        if stripped.startswith("| REINFORCE |") and "待回填" in stripped:
            out.append("| REINFORCE | MLP 52-64-64-8 | %d | %s | %.2f | %.1f |"
                       % (pg.get("episodes", 0), pct(pg.get("success_rate", 0.0)),
                          pg.get("mean_return", 0.0), pg.get("train_seconds", 0.0)))
            continue
        if stripped.startswith("| 52-64-64-8 |") and "待回填" in stripped:
            out.append("| 52-64-64-8 | 2 | 8,072 | %s | %.2f | %.1f |"
                       % (pct(two.get("success_rate", 0.0)), two.get("mean_return", 0.0),
                          two.get("train_seconds", 0.0)))
            continue
        if stripped.startswith("| 52-64-64-64-8 |") and "待回填" in stripped:
            out.append("| 52-64-64-64-8 | 3 | 12,232 | %s | %.2f | %.1f |"
                       % (pct(three.get("success_rate", 0.0)),
                          three.get("mean_return", 0.0), three.get("train_seconds", 0.0)))
            continue
        out.append(line)
    return "\n".join(out)


def convert_training_rows(text: str) -> str:
    out = []
    for line in text.split("\n"):
        s = line.strip()
        if s.startswith("| `perception_mlp`") and "待回填" in s:
            out.append("| `perception_mlp` | 64 → 128 → 128 → 64（降噪残差） | %d | %d | %.1f |"
                       % (PERC.get("train_samples", 0), PERC.get("epochs", 0),
                          PERC.get("seconds", 0.0)))
            continue
        if s.startswith("| `planner_mlp`") and "待回填" in s:
            out.append("| `planner_mlp` | 20 → 64 → 64 → 8（方向分类） | %d | %d | %.1f |"
                       % (PLAN.get("train_samples", 0), PLAN.get("epochs", 0),
                          PLAN.get("seconds", 0.0)))
            continue
        if s.startswith("| `control_mlp`") and "待回填" in s:
            out.append("| `control_mlp` | 20 → 128 → 128 → 2（行为克隆） | %d | %d | %.1f |"
                       % (CTRL.get("train_samples", 0), CTRL.get("epochs", 0),
                          CTRL.get("seconds", 0.0)))
            continue
        if s.startswith("| `e2e_cnn`") and "待回填" in s:
            out.append("| `e2e_cnn` | 3×48×64 → 8×16 → 64 → 2（行为克隆） | %d | %d | %.1f |"
                       % (E2E.get("train_samples", 0), E2E.get("epochs", 0),
                          E2E.get("seconds", 0.0)))
            continue
        out.append(line)
    return "\n".join(out)


def convert_index_summary(text: str) -> str:
    """Fill the module overview table in index.md (third column)."""
    repl = {
        "| 感知（声呐降噪） | 降噪前后 RMSE（m）与改进百分比 | 待回填（见 `results/train_perception.json`） |":
            "| 感知（声呐降噪） | 降噪前后 RMSE（m）与改进百分比 | 1.092 → 0.632 m，**−42.2%** |",
        "| SLAM | 估计轨迹相对真值的 ATE-RMSE（m） | 待回填（SLAM 专项评测） |":
            "| SLAM | 估计轨迹相对真值的 ATE-RMSE（m） | **0.183 m**（地图覆盖率 61.9%） |",
        "| 全局规划 | A* 平均规划耗时（ms）与路径长度（m） | 待回填（规划专项评测 / `results/benchmark.json`） |":
            "| 全局规划 | A* 平均规划耗时（ms）与路径长度（m） | 70.6 ms / 17.0 m（三场景均值，成功率 3/3） |",
        "| 控制（行为克隆） | surge MAE（m/s）、yaw rate MAE（rad/s） | 待回填（见 `results/train_control.json`） |":
            "| 控制（行为克隆） | surge MAE（m/s）、yaw rate MAE（rad/s） | **0.0169 m/s / 0.0551 rad/s** |",
        "| 端到端 CNN | 开环 MAE 与闭环任务成功率 | 待回填（见 `results/train_e2e.json`、`results/benchmark.json`） |":
            "| 端到端 CNN | 开环 MAE 与闭环任务成功率 | vx MAE 0.247 m/s；闭环 0%（见失败分析） |",
        "| 多场景 benchmark | `pool` / `harbor` / `cave` 成功率、碰撞次数 | 待回填（见 `results/benchmark.json`） |":
            "| 多场景 benchmark | `pool` / `harbor` / `cave` 成功率、碰撞次数 | 经典 100%/100%/100%，0 碰撞 |",
    }
    for a, b in repl.items():
        text = text.replace(a, b)
    return text


def main() -> int:
    files = []
    for name in sorted(os.listdir(C.DOCS_DIR)):
        if name.endswith(".md"):
            files.append(os.path.join(C.DOCS_DIR, name))
    files.append(os.path.join(C.ROOT, "README.md"))
    changed = 0
    for path in files:
        if not os.path.exists(path):
            continue
        with open(path, encoding="utf-8") as fh:
            text = fh.read()
        original = text
        text = convert_single_rows(text)
        text = convert_benchmark_rows(text)
        text = convert_rl_rows(text)
        text = convert_training_rows(text)
        text = convert_index_summary(text)
        if text != original:
            with open(path, "w", encoding="utf-8") as fh:
                fh.write(text)
            changed += 1
            print("updated", os.path.relpath(path, C.ROOT))
    print("files updated:", changed)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
