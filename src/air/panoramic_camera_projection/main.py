"""Run an offline cubemap-to-equirectangular panorama demonstration."""

from __future__ import annotations

import argparse
import time
from pathlib import Path
from typing import Dict

import cv2

try:
    from .stitcher import FACE_NAMES, create_demo_cubemap, stitch_cubemap
except ImportError:
    from stitcher import FACE_NAMES, create_demo_cubemap, stitch_cubemap


def _load_faces(directory: Path) -> Dict[str, "cv2.Mat"]:
    faces = {}
    for name in FACE_NAMES:
        path = directory / f"{name}.png"
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            raise FileNotFoundError(f"cannot read {path}")
        faces[name] = image
    return faces


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Stitch six cube faces into an equirectangular panorama"
    )
    parser.add_argument(
        "--faces-dir",
        type=Path,
        help="directory containing front/right/back/left/up/down PNG files",
    )
    parser.add_argument("--output", type=Path, default=Path("output/panorama.png"))
    parser.add_argument("--width", type=int, default=1024)
    parser.add_argument("--height", type=int, default=512)
    parser.add_argument("--frame-delay", type=float, default=0.0)
    args = parser.parse_args()

    if args.width != args.height * 2:
        print("ERROR: panorama width must be twice its height")
        return 1

    print("loading six cube faces")
    faces = _load_faces(args.faces_dir) if args.faces_dir else create_demo_cubemap(256)
    if args.frame_delay > 0:
        time.sleep(args.frame_delay)
    print("projecting equirectangular pixels")
    panorama = stitch_cubemap(faces, (args.height, args.width))
    if args.frame_delay > 0:
        time.sleep(args.frame_delay)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(args.output), panorama):
        print(f"ERROR: cannot write {args.output}")
        return 1

    print(f"faces: {', '.join(FACE_NAMES)}")
    print(f"output: {args.output}")
    print(f"resolution: {args.width}x{args.height}")
    print("PASS: equirectangular panorama generated")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
