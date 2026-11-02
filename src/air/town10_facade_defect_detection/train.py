"""通过外部 YOLO 数据目录训练四类模型，训练产物保存至指定目录。"""
import argparse
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True, help="含 images/train、images/val、labels 的目录")
    parser.add_argument("--output", type=Path, default=Path("training"))
    parser.add_argument("--weights", default="yolov8n.pt")
    parser.add_argument("--epochs", type=int, default=60)
    parser.add_argument("--batch", type=int, default=8)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--export", action="store_true", help="训练后导出 ONNX")
    args = parser.parse_args()
    root = args.dataset.resolve()
    for sub in ("images/train", "images/val", "labels/train", "labels/val"):
        if not (root / sub).is_dir():
            raise FileNotFoundError(root / sub)
    if args.epochs <= 0 or args.batch <= 0:
        parser.error("epochs 与 batch 必须为正整数")
    from ultralytics import YOLO
    args.output.mkdir(parents=True, exist_ok=True)
    # 重建便携数据索引，不使用数据包中硬编码的旧机器绝对路径。
    data_yaml = args.output.resolve() / "data.yaml"
    data_yaml.write_text(f"path: '{root.as_posix()}'\ntrain: images/train\nval: images/val\n"
                         "names: [crack, stain, peel, glass]\n", encoding="utf-8")
    model = YOLO(args.weights)
    model.train(data=str(data_yaml), epochs=args.epochs, batch=args.batch, imgsz=640, device=args.device,
                project=str(args.output.resolve()), name="facade_4class", seed=20261018, exist_ok=False)
    if args.export:
        best = args.output.resolve() / "facade_4class" / "weights" / "best.pt"
        YOLO(str(best)).export(format="onnx", opset=12, simplify=False, dynamic=False)


if __name__ == "__main__":
    main()
