"""运行三个纹理场景，保存独立指标和实际连续采样图像。"""

import argparse
import json
from pathlib import Path

from town10_capture.config import CruiseConfig
from town10_capture.mission import run_cruise


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=Path("output/three_scenes"))
    args = parser.parse_args()
    reports = [
        run_cruise(CruiseConfig(style=style, output=args.output / style, gif=True))
        for style in ("brick", "gray", "glass")
    ]
    (args.output / "summary.json").write_text(
        json.dumps(reports, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(reports, ensure_ascii=False, indent=2))
    if any(item["status"] != "complete" for item in reports):
        raise SystemExit(2)


if __name__ == "__main__":
    main()
