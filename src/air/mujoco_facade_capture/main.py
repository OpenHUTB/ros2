"""无人机幕墙采集统一入口。"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from facade.learning import DenseController
from facade.simulation import run_scan
from train import neural_policy


def main() -> None:
    parser = argparse.ArgumentParser(description="MuJoCo 无人机幕墙定距扫描")
    parser.add_argument("--output", type=Path, default=Path("output/facade_scan"), help="结果目录")
    parser.add_argument("--style", choices=["grid", "brick", "plain"], default="grid")
    parser.add_argument("--duration", type=float, default=70., help="仿真秒数")
    parser.add_argument("--distance", type=float, default=4., help="目标壁距，米")
    parser.add_argument("--offset", type=float, default=0., help="初始壁距偏移，米")
    parser.add_argument("--controller", choices=["proportional", "neural"], default="proportional")
    parser.add_argument("--model", type=Path, default=Path(__file__).resolve().parent / "models/facade_controller.json")
    parser.add_argument("--no-images", action="store_true", help="仅计算轨迹，不输出照片")
    args = parser.parse_args()
    policy = None
    if args.controller == "neural":
        policy = neural_policy(DenseController.load(args.model), args.distance)
    metrics = run_scan(args.output, args.style, args.duration, args.distance, args.offset,
                       not args.no_images, policy=policy)
    print(json.dumps(metrics, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
