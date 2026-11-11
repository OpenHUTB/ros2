# -*- coding: utf-8 -*-
"""离线评测限速牌红色定位与数字切分，不加载 CNN、不启动 ROS。

对多个限速值和退化条件生成合成相机画面，检查 ROI 中心和半径是否准确、
字符切分数量是否等于牌面数字位数，并写出 artifacts/vision_eval.json。

用法：python3 scripts/evaluate_vision.py
"""
import argparse
import json
import random
import sys
from pathlib import Path

import numpy as np
from PIL import Image

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))

from traffic_sign_speed_control.sign_render import (  # noqa: E402
    augment, draw_limit_sign,
)
from traffic_sign_speed_control.vision import find_sign_roi, split_chars  # noqa: E402

WIDTH, HEIGHT = 320, 240
CENTER = (WIDTH // 2, HEIGHT // 2)
VALUES = (15, 20, 30, 40, 50, 60, 80)
CONDITIONS = {
    "clean": {},
    "blur": {"blur_sigma": 1.0},
    "dark": {"brightness": 0.35, "contrast": 0.2},
    "rotate": {"rotate": 8},
    "noise": {"noise_std": 0.04},
    "combined": {
        "rotate": 5,
        "blur_sigma": 0.8,
        "brightness": 0.2,
        "contrast": 0.2,
        "noise_std": 0.02,
    },
}


def make_scene(value_kmh, seed, **degrade):
    sign_size = 96
    sign = draw_limit_sign(value_kmh, size=sign_size, alpha=True)
    if degrade:
        sign = augment(sign, random.Random(seed), **degrade)
    image = Image.new("RGB", (WIDTH, HEIGHT), (72, 118, 76))
    left = CENTER[0] - sign_size // 2
    top = CENTER[1] - sign_size // 2
    image.paste(sign, (left, top), sign)
    return np.asarray(image)[:, :, ::-1].copy()


def evaluate_case(value_kmh, condition, kwargs, seed):
    bgr = make_scene(value_kmh, seed, **kwargs)
    roi = find_sign_roi(bgr)
    if roi is None:
        return {
            "value": value_kmh,
            "condition": condition,
            "roi": None,
            "char_count": 0,
            "expected_chars": len(str(value_kmh)),
            "passed": False,
        }
    cx, cy, radius = roi
    chars = split_chars(bgr, cx, cy, radius)
    roi_ok = (abs(cx - CENTER[0]) <= 3 and abs(cy - CENTER[1]) <= 3
              and 42 <= radius <= 50)
    count_ok = len(chars) == len(str(value_kmh))
    return {
        "value": value_kmh,
        "condition": condition,
        "roi": [cx, cy, radius],
        "char_count": len(chars),
        "expected_chars": len(str(value_kmh)),
        "passed": roi_ok and count_ok,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, default=123)
    args = parser.parse_args()

    rows = []
    for condition, kwargs in CONDITIONS.items():
        for value_kmh in VALUES:
            rows.append(evaluate_case(
                value_kmh, condition, kwargs, args.seed + value_kmh))

    print(f"{'condition':<10}{'value':>7}{'roi':>18}{'chars':>8}{'result':>9}")
    print("-" * 52)
    for row in rows:
        roi = "-" if row["roi"] is None else ",".join(map(str, row["roi"]))
        result = "PASS" if row["passed"] else "FAIL"
        print(f"{row['condition']:<10}{row['value']:>7}{roi:>18}"
              f"{row['char_count']:>4}/{row['expected_chars']:<3}{result:>9}")

    passed = sum(row["passed"] for row in rows)
    print(f"\n{passed}/{len(rows)} cases passed")
    art = PKG / "artifacts"
    art.mkdir(exist_ok=True)
    output = art / "vision_eval.json"
    output.write_text(
        json.dumps({"passed": passed, "total": len(rows), "cases": rows},
                   indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    print(f"written: {output}")
    return 0 if passed == len(rows) else 1


if __name__ == "__main__":
    sys.exit(main())
