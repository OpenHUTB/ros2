"""Unified project entry point / 项目统一入口。"""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import random
import sys
from typing import Any

import numpy as np
import yaml

from rooftop_pv.dataset import DatasetConfig, generate_dataset
from rooftop_pv.rendering import render_rgb, save_rgb
from rooftop_pv.route import RouteConfig, generate_lawnmower_route, route_length, save_route
from rooftop_pv.scene import SCENARIOS, load_model, set_drone_pose


MODULE_ROOT = Path(__file__).resolve().parent
DEFAULT_CONFIG = MODULE_ROOT / "config" / "survey.yaml"


def read_config(path: Path) -> dict[str, Any]:
    """Read and validate the YAML mapping / 读取并校验 YAML 映射。"""
    if not path.is_file():
        raise FileNotFoundError(f"configuration file is missing / 配置文件缺失: {path}")
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("configuration root must be a mapping / 配置根节点必须为映射")
    return payload


def run_scene(config: dict[str, Any], output: Path) -> dict[str, Any]:
    """Load, route, render, and summarize one scene / 加载、规划、渲染并汇总一个场景。"""
    scenario = str(config.get("scenario", "sparse"))
    if scenario not in SCENARIOS:
        raise ValueError(f"scenario must be one of {SCENARIOS} / 场景名称无效")
    seed = int(config.get("seed", 20261016))
    random.seed(seed)
    np.random.seed(seed)

    route_config = RouteConfig(
        roof_width=float(config["roof_width"]),
        roof_depth=float(config["roof_depth"]),
        altitude=float(config["altitude"]),
        strip_spacing=float(config["strip_spacing"]),
        speed=float(config["speed"]),
        edge_margin=float(config["edge_margin"]),
    )
    route = generate_lawnmower_route(route_config)
    model, data = load_model(scenario)
    set_drone_pose(model, data, 0.0, 0.0, route_config.altitude, 0.0)

    output.mkdir(parents=True, exist_ok=True)
    route_path = output / "route.json"
    image_path = output / "scene_rgb.png"
    summary_path = output / "summary.json"
    save_route(route, route_config, route_path)
    image = render_rgb(
        model,
        data,
        width=int(config.get("image_width", 320)),
        height=int(config.get("image_height", 240)),
    )
    save_rgb(image, image_path)
    summary = {
        "schema_version": 1,
        "stage": "scene_and_route",
        "scenario": scenario,
        "seed": seed,
        "mujoco_model": model.names.decode("utf-8", errors="ignore").split("\x00", 1)[0],
        "route_parameters": asdict(route_config),
        "waypoint_count": len(route),
        "route_length_m": round(route_length(route), 6),
        "planned_duration_s": round(route[-1].arrival_time, 6),
        "pv_geom_count": sum(
            1
            for geom_id in range(model.ngeom)
            if (model.geom(geom_id).name or "").startswith("pv_panel_")
        ),
        "artifacts": {"route": route_path.name, "rgb": image_path.name},
    }
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return summary


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="MuJoCo urban rooftop photovoltaic coverage survey / MuJoCo 城市屋顶光伏覆盖率测量"
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG, help="YAML configuration / YAML 配置")
    parser.add_argument("--output", type=Path, default=MODULE_ROOT / "artifacts" / "scene", help="output directory / 输出目录")
    parser.add_argument("--scenario", choices=SCENARIOS, help="override scenario / 覆盖场景名称")
    parser.add_argument("--mode", choices=("scene", "dataset"), default="scene", help="execution mode / 执行模式")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        config = read_config(args.config.resolve())
        if args.scenario:
            config["scenario"] = args.scenario
        if args.mode == "scene":
            summary = run_scene(config, args.output.resolve())
        else:
            route_config = RouteConfig(
                roof_width=float(config["roof_width"]),
                roof_depth=float(config["roof_depth"]),
                altitude=float(config["altitude"]),
                strip_spacing=float(config["strip_spacing"]),
                speed=float(config["speed"]),
                edge_margin=float(config["edge_margin"]),
            )
            dataset_config = DatasetConfig(
                frames_per_scene=int(config.get("frames_per_scene", 6)),
                image_width=int(config.get("image_width", 160)),
                image_height=int(config.get("image_height", 120)),
                seed=int(config.get("seed", 20261016)),
                cross_track_std=float(config.get("cross_track_std", 0.35)),
                heading_std=float(config.get("heading_std", 0.12)),
            )
            summary = generate_dataset(args.output.resolve(), route_config, dataset_config)
    except (KeyError, TypeError, ValueError, OSError, RuntimeError) as exc:
        print(f"error / 错误: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(summary, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
