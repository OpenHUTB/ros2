#!/usr/bin/env python3
"""Train YOLOv8n on the collected drone dataset."""

import argparse

from ultralytics import YOLO


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", default="~/.ros/drone_dataset/data.yaml")
    parser.add_argument("--model", default="yolov8n.pt")
    parser.add_argument("--epochs", type=int, default=20)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=int, default=8)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--project", default="~/.ros/drone_runs")
    parser.add_argument("--name", default="yolov8n_drone")
    args = parser.parse_args()

    model = YOLO(args.model)
    model.train(
        data=args.data,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        project=args.project,
        name=args.name,
        exist_ok=True,
    )
    metrics = model.val(
        data=args.data,
        imgsz=args.imgsz,
        device=args.device,
        project=args.project,
        name=args.name + "_val",
        exist_ok=True,
    )
    print("mAP50: %.4f" % metrics.box.map50)
    print("mAP50-95: %.4f" % metrics.box.map)


if __name__ == "__main__":
    main()
