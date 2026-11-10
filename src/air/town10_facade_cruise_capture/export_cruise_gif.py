"""压缩已记录的巡航动图，保留时间跨度和帧顺序。"""

import argparse
from pathlib import Path

from PIL import Image


def export(source, output, stride=2):
    frames = []
    durations = []
    with Image.open(source) as image:
        pending = 0
        for index in range(image.n_frames):
            image.seek(index)
            if index % stride == 0:
                frames.append(
                    image.convert("RGB").resize((320, 264)).quantize(colors=128)
                )
                durations.append(0)
            pending = int(image.info.get("duration", 500))
            durations[-1] += pending
    frames[0].save(
        output,
        save_all=True,
        append_images=frames[1:],
        duration=durations,
        loop=0,
        optimize=True,
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--stride", type=int, default=2)
    args = parser.parse_args()
    if args.stride < 1:
        parser.error("stride必须大于0")
    export(args.source, args.output, args.stride)
