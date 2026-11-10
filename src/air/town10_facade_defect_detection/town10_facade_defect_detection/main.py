"""目录图像推理入口：保留原图文件名并输出结构化识别结果。"""
import argparse
import json
from pathlib import Path

try:
    from .detector import (
        DEFAULT_MODEL,
        FacadeDetector,
        annotate,
        asset_directory,
        read_image,
        write_image,
    )
except ImportError:
    from detector import (
        DEFAULT_MODEL,
        FacadeDetector,
        annotate,
        asset_directory,
        read_image,
        write_image,
    )


def run(input_path, output_path, model=DEFAULT_MODEL, confidence=0.25, nms_iou=0.45):
    source, output = Path(input_path), Path(output_path)
    files = [source] if source.is_file() else sorted(
        p for p in source.iterdir() if p.is_file() and p.suffix.lower() in {".png", ".jpg", ".jpeg"})
    if not files:
        raise ValueError(f"输入目录没有图像: {source}")
    detector = FacadeDetector(model, confidence, nms_iou)
    records = []
    for path in files:
        image = read_image(path)
        detections = detector.predict(image)
        target = output / "annotated" / (path.stem + ".png")
        write_image(target, annotate(image, detections))
        records.append({"image": path.name, "width": image.shape[1], "height": image.shape[0],
                        "n_defects": sum(d["is_defect"] for d in detections),
                        "detections": detections})
    output.mkdir(parents=True, exist_ok=True)
    (output / "detections.json").write_text(json.dumps({"confidence_threshold": confidence,
        "nms_iou": nms_iou, "images": records}, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"已处理 {len(records)} 张图像，结果: {output.resolve()}")
    return records


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=asset_directory() / "samples")
    parser.add_argument("--output", type=Path, default=Path("output"))
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--confidence", type=float, default=0.25)
    parser.add_argument("--nms-iou", type=float, default=0.45)
    args = parser.parse_args()
    run(args.input, args.output, args.model, args.confidence, args.nms_iou)


if __name__ == "__main__":
    main()
