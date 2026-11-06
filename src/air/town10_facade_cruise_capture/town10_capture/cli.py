"""命令行统一入口。"""

import argparse
import json
from pathlib import Path

from .config import CruiseConfig
from .mission import run_cruise


def main():
    parser = argparse.ArgumentParser(description="Town10立面自主巡航与图像采集")
    parser.add_argument("--backend", choices=["mujoco", "airsim"], default="mujoco")
    parser.add_argument("--style", choices=["brick", "gray", "glass"], default="brick")
    parser.add_argument("--output", type=Path, default=Path("output/town10_cruise"))
    parser.add_argument("--duration", type=float, default=100.0)
    parser.add_argument("--distance", type=float, default=4.0)
    parser.add_argument("--speed", type=float, default=0.8)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--capture-rate", type=float, default=2.0)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=41451)
    parser.add_argument("--vehicle", default="")
    parser.add_argument("--camera", default="0")
    parser.add_argument("--route", choices=["relative", "town10"], default="relative")
    parser.add_argument(
        "--route-offset",
        type=float,
        nargs=3,
        default=(0.0, 0.0, 0.0),
        metavar=("N", "E", "D"),
    )
    parser.add_argument("--waypoint-timeout", type=float, default=35.0)
    parser.add_argument("--model", type=Path)
    parser.add_argument(
        "--takeoff", action="store_true", help="启动时请求解锁起飞；默认要求已悬停"
    )
    parser.add_argument("--gif", action="store_true", help="连续采样生成本次巡航动图")
    args = parser.parse_args()
    try:
        result = run_cruise(CruiseConfig(**vars(args)))
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0 if result["status"] == "complete" else 2
    except (ValueError, RuntimeError) as exc:
        parser.exit(2, f"巡航终止：{exc}\n")


if __name__ == "__main__":
    raise SystemExit(main())
