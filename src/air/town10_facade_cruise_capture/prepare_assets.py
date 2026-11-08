"""Inspect and adapt the supplied facade image collection."""

import argparse
import hashlib
import json
from pathlib import Path

from PIL import Image

SELECTION = {
    "brick": ("rgb_1792251577_002.png", [25, 30, 615, 345]),
    "gray": ("rgb_1792251340_000.png", [40, 0, 600, 350]),
    "glass": ("rgb_1792253773_001.png", [50, 25, 620, 345]),
}


def adapt(
    source: Path, output: Path, selection: dict[str, tuple[str, list[int]]]
) -> None:
    output.mkdir(parents=True, exist_ok=True)
    records = []
    for style, (name, crop) in selection.items():
        path = source / name
        original = path.read_bytes()
        image = Image.open(path).convert("RGB")
        image.crop(crop).resize((1000, 800), Image.Resampling.LANCZOS).save(
            output / f"{style}.png"
        )
        records.append(
            {
                "style": style,
                "source_file": name,
                "source_relative_path": f"dataset_manual/images/{name}",
                "source_sha256": hashlib.sha256(original).hexdigest(),
                "source_size_px": list(image.size),
                "crop_xyxy": crop,
                "texture_file": f"{style}.png",
                "texture_sha256": hashlib.sha256(
                    (output / f"{style}.png").read_bytes()
                ).hexdigest(),
                "mapping": "one RGB crop on one 10 m by 8 m planar facade; no recovered depth or original camera pose",
            }
        )
    (output / "provenance.json").write_text(
        json.dumps(records, indent=2, ensure_ascii=False), encoding="utf-8"
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    adapt(args.source, args.output, SELECTION)
