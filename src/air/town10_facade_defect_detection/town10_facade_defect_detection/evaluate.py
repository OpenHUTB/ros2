"""在 YOLO 格式验证集上统计检测指标，不依赖训练框架。"""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

try:
    from .detector import CLASSES, DEFAULT_MODEL, FacadeDetector, iou_matrix, read_image
except ImportError:
    from detector import CLASSES, DEFAULT_MODEL, FacadeDetector, iou_matrix, read_image


def interpolated_ap(tp, fp, total):
    if total == 0:
        return None
    if not len(tp):
        return 0.0
    precision = np.cumsum(tp) / np.maximum(np.cumsum(tp) + np.cumsum(fp), 1)
    recall = np.cumsum(tp) / total
    return float(np.mean([np.max(precision[recall >= r], initial=0) for r in np.linspace(0, 1, 101)]))


def score_class(records, class_id, threshold, operating_confidence):
    ground = {r["image"]: np.array([x["xyxy"] for x in r["ground_truth"] if x["class_id"] == class_id]).reshape(-1, 4)
              for r in records}
    predictions = sorted([(d["confidence"], r["image"], d["xyxy"]) for r in records
                          for d in r["predictions"] if d["class_id"] == class_id], reverse=True)
    used = {name: set() for name in ground}
    tp, fp, confidences = [], [], []
    for confidence, name, box in predictions:
        overlap = iou_matrix([box], ground[name])[0]
        eligible = [j for j in range(len(overlap)) if j not in used[name] and overlap[j] >= threshold]
        correct = bool(eligible)
        if correct:
            used[name].add(max(eligible, key=lambda j: overlap[j]))
        tp.append(int(correct)); fp.append(int(not correct)); confidences.append(confidence)
    total = sum(len(v) for v in ground.values())
    ap = interpolated_ap(np.array(tp), np.array(fp), total)
    op = np.array(confidences) >= operating_confidence
    true_pos, false_pos = int(np.array(tp)[op].sum()), int(np.array(fp)[op].sum())
    precision = true_pos / max(true_pos + false_pos, 1)
    recall = true_pos / max(total, 1)
    return {"ap": ap, "ground_truth": total, "true_positive": true_pos, "false_positive": false_pos,
            "false_negative": total - true_pos, "precision": precision, "recall": recall,
            "f1": 2 * precision * recall / max(precision + recall, 1e-12)}


def evaluate(dataset, output, model=DEFAULT_MODEL, operating_confidence=0.25):
    dataset, output = Path(dataset), Path(output)
    detector = FacadeDetector(model, confidence=0.001)
    records = []
    for path in sorted((dataset / "images" / "val").glob("*.png")):
        image = read_image(path)
        h, w = image.shape[:2]
        labels = []
        label = dataset / "labels" / "val" / (path.stem + ".txt")
        for line in label.read_text(encoding="utf-8").splitlines():
            if not line.strip():
                continue
            cid, cx, cy, bw, bh = map(float, line.split())
            labels.append({"class_id": int(cid), "xyxy": [(cx - bw / 2) * w, (cy - bh / 2) * h,
                           (cx + bw / 2) * w, (cy + bh / 2) * h]})
        records.append({"image": path.name, "ground_truth": labels, "predictions": detector.predict(image)})
    if not records:
        raise ValueError("dataset/images/val 中没有 PNG 验证图像")
    per_class = {}
    for cid, name in enumerate(CLASSES):
        result = score_class(records, cid, 0.5, operating_confidence)
        aps = [score_class(records, cid, float(t), operating_confidence)["ap"] for t in np.arange(0.5, 0.951, 0.05)]
        result["ap50_101"] = result.pop("ap")
        result["ap50_95_101"] = float(np.mean([x for x in aps if x is not None]))
        per_class[name] = result
    tp = sum(v["true_positive"] for v in per_class.values())
    fp = sum(v["false_positive"] for v in per_class.values())
    fn = sum(v["false_negative"] for v in per_class.values())
    defect_tp = sum(per_class[c]["true_positive"] for c in CLASSES[:3])
    defect_fp = sum(per_class[c]["false_positive"] for c in CLASSES[:3])
    defect_fn = sum(per_class[c]["false_negative"] for c in CLASSES[:3])
    summary = {"images": len(records), "matching_iou": 0.5, "operating_confidence": operating_confidence,
               "inference_confidence": 0.001, "nms_iou": 0.45,
               "ap_method": "101 recall points; class-specific one-to-one IoU matching; max 300 detections/image",
               "validation_scope": "图像级划分；连续帧可能分布于训练与验证，不能解释为严格跨楼泛化",
               "precision_micro": tp / max(tp + fp, 1), "recall_micro": tp / max(tp + fn, 1),
               "defect_precision_micro": defect_tp / max(defect_tp + defect_fp, 1),
               "defect_recall_micro": defect_tp / max(defect_tp + defect_fn, 1),
               "map50_101": float(np.mean([v["ap50_101"] for v in per_class.values()])),
               "map50_95_101": float(np.mean([v["ap50_95_101"] for v in per_class.values()])),
               "model_sha256": hashlib.sha256(Path(model).read_bytes()).hexdigest(),
               "opencv_version": cv2.__version__, "numpy_version": np.__version__,
               "per_class": per_class}
    output.mkdir(parents=True, exist_ok=True)
    (output / "metrics.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    (output / "predictions.json").write_text(json.dumps(records, ensure_ascii=False, indent=2), encoding="utf-8")
    validation_files = []
    for record in records:
        name = record["image"]
        validation_files.append({"image": name,
            "image_sha256": hashlib.sha256((dataset / "images" / "val" / name).read_bytes()).hexdigest(),
            "label_sha256": hashlib.sha256((dataset / "labels" / "val" / (Path(name).stem + ".txt")).read_bytes()).hexdigest()})
    (output / "validation_files.json").write_text(json.dumps(validation_files, indent=2), encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=Path("evaluation"))
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    args = parser.parse_args()
    evaluate(args.dataset, args.output, args.model)


if __name__ == "__main__":
    main()
