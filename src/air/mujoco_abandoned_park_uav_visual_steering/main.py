#!/usr/bin/env python3
"""Unified module entry point / 模块统一入口。"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

MODULE_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(MODULE_ROOT))

from mujoco_abandoned_park_uav_visual_steering import SimulationConfig, UavSimulation


def build_parser() -> argparse.ArgumentParser:
    """Build the command-line interface / 构建命令行接口。"""
    parser = argparse.ArgumentParser(
        description=(
            "Run the MuJoCo abandoned-park UAV module / "
            "运行 MuJoCo 失落公园无人机模块"
        )
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=MODULE_ROOT / "config" / "simulation.yaml",
        help="simulation YAML / 仿真 YAML 配置",
    )
    parser.add_argument(
        "--model",
        type=Path,
        default=MODULE_ROOT / "models" / "scene.xml",
        help="MJCF scene / MJCF 场景文件",
    )
    parser.add_argument(
        "--steps",
        type=_positive_int,
        default=120,
        help="control steps / 控制步数",
    )
    parser.add_argument(
        "--steering",
        type=_unit_float,
        default=0.0,
        help="normalized steering in [-1, 1] / [-1, 1] 归一化转向量",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    """Run a deterministic headless episode / 运行确定性的无窗口回合。"""
    args = build_parser().parse_args(argv)
    try:
        config = SimulationConfig.from_yaml(args.config.resolve())
        simulation = UavSimulation(args.model.resolve(), config)
        for _ in range(args.steps):
            simulation.step(args.steering)
    except (FileNotFoundError, ValueError, RuntimeError) as error:
        print(f"Run failed / 运行失败: {error}", file=sys.stderr)
        return 2

    result = {
        "status": "ok",
        "status_zh": "正常",
        "mode": "simulation",
        "seed": config.seed,
        "steps": args.steps,
        "state": simulation.state(),
    }
    print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    return 0


def _positive_int(text: str) -> int:
    value = int(text)
    if value <= 0:
        raise argparse.ArgumentTypeError("must be positive / 必须为正整数")
    return value


def _unit_float(text: str) -> float:
    value = float(text)
    if not -1.0 <= value <= 1.0:
        raise argparse.ArgumentTypeError("must be within [-1, 1] / 必须位于 [-1, 1]")
    return value


if __name__ == "__main__":
    raise SystemExit(main())


