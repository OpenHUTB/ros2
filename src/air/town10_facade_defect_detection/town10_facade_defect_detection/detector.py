"""幕墙四类检测的预处理、推理与坐标恢复。"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

CLASSES = ("crack", "stain", "peel", "glass")
DEFECT_CLASSES = frozenset(CLASSES[:3])
COLORS = ((40, 40, 240), (0, 160, 240), (220, 40, 200), (50, 200, 50))
def asset_directory():
    """源码目录与 ROS 安装目录均使用同一份模型和样本。"""
    source_root = Path(__file__).resolve().parents[1]
    if (source_root / "models" / "facade_4class.onnx").is_file():
        return source_root
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory("town10_facade_defect_detection"))


DEFAULT_MODEL = asset_directory() / "models" / "facade_4class.onnx"


def iou_matrix(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """计算两组 xyxy 边框的交并比，空输入保持二维返回形状。"""
    a, b = np.asarray(a, dtype=float).reshape(-1, 4), np.asarray(b, dtype=float).reshape(-1, 4)
    lo = np.maximum(a[:, None, :2], b[None, :, :2])
    hi = np.minimum(a[:, None, 2:], b[None, :, 2:])
    wh = np.maximum(hi - lo, 0)
    inter = wh[..., 0] * wh[..., 1]
    aa = np.prod(np.maximum(a[:, 2:] - a[:, :2], 0), axis=1)
    bb = np.prod(np.maximum(b[:, 2:] - b[:, :2], 0), axis=1)
    return inter / np.maximum(aa[:, None] + bb[None, :] - inter, 1e-12)


def class_nms(boxes, scores, labels, threshold=0.45, max_detections=300):
    """同类别进行非极大值抑制，不让玻璃构件压掉同位置的病害框。"""
    boxes, scores, labels = np.asarray(boxes), np.asarray(scores), np.asarray(labels)
    kept = []
    for label in np.unique(labels):
        ids = np.flatnonzero(labels == label)
        ids = ids[np.argsort(-scores[ids], kind="stable")]
        while len(ids):
            best = int(ids[0])
            kept.append(best)
            if len(ids) == 1:
                break
            others = ids[1:]
            ids = others[iou_matrix(boxes[best:best + 1], boxes[others])[0] <= threshold]
    return sorted(kept, key=lambda i: -scores[i])[:max_detections]


def letterbox(image, size=640):
    if image is None or image.ndim != 3 or image.shape[2] != 3 or image.size == 0:
        raise ValueError("输入必须是非空 BGR 三通道图像")
    h, w = image.shape[:2]
    ratio = min(size / h, size / w)
    nw, nh = int(round(w * ratio)), int(round(h * ratio))
    left, top = (size - nw) // 2, (size - nh) // 2
    resized = cv2.resize(image, (nw, nh), interpolation=cv2.INTER_LINEAR)
    padded = cv2.copyMakeBorder(resized, top, size - nh - top, left, size - nw - left,
                               cv2.BORDER_CONSTANT, value=(114, 114, 114))
    return padded, ratio, left, top


class FacadeDetector:
    """使用 OpenCV CPU 后端执行 YOLOv8 ONNX，无需 PyTorch。"""

    def __init__(self, model=DEFAULT_MODEL, confidence=0.25, nms_iou=0.45):
        if not np.isfinite(confidence) or not 0 <= confidence <= 1:
            raise ValueError("confidence 必须为 [0,1] 内有限数")
        if not np.isfinite(nms_iou) or not 0 < nms_iou <= 1:
            raise ValueError("nms_iou 必须为 (0,1] 内有限数")
        self.model = Path(model)
        self.confidence, self.nms_iou = confidence, nms_iou
        if not self.model.is_file():
            raise FileNotFoundError(self.model)
        # 随包模型使用固定摘要与类别清单，防止权重与类别顺序错配。
        manifest_path = self.model.with_suffix(".json")
        if manifest_path.is_file():
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            digest = hashlib.sha256(self.model.read_bytes()).hexdigest()
            if digest != manifest["sha256"] or manifest["classes"] != list(CLASSES):
                raise ValueError("模型摘要或类别顺序与清单不一致")
        self.net = cv2.dnn.readNetFromONNX(str(self.model))
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

    def predict(self, image):
        padded, ratio, left, top = letterbox(image)
        self.net.setInput(cv2.dnn.blobFromImage(padded, 1 / 255.0, (640, 640), swapRB=True))
        out = self.net.forward()
        if out.ndim != 3 or out.shape[0] != 1 or out.shape[1] != 4 + len(CLASSES):
            raise ValueError(f"不支持的模型输出 {out.shape}，预期 (1,8,N)")
        pred = out[0].T
        labels = pred[:, 4:].argmax(axis=1)
        scores = pred[:, 4:].max(axis=1)
        mask = np.isfinite(pred).all(axis=1) & (scores >= self.confidence)
        pred, labels, scores = pred[mask, :4], labels[mask], scores[mask]
        boxes = np.empty_like(pred)
        boxes[:, :2], boxes[:, 2:] = pred[:, :2] - pred[:, 2:] / 2, pred[:, :2] + pred[:, 2:] / 2
        boxes[:, [0, 2]] = (boxes[:, [0, 2]] - left) / ratio
        boxes[:, [1, 3]] = (boxes[:, [1, 3]] - top) / ratio
        h, w = image.shape[:2]
        boxes[:, [0, 2]] = boxes[:, [0, 2]].clip(0, w)
        boxes[:, [1, 3]] = boxes[:, [1, 3]].clip(0, h)
        valid = (boxes[:, 2] > boxes[:, 0]) & (boxes[:, 3] > boxes[:, 1])
        boxes, labels, scores = boxes[valid], labels[valid], scores[valid]
        ids = class_nms(boxes, scores, labels, self.nms_iou)
        return [{"class_id": int(labels[i]), "class_name": CLASSES[labels[i]],
                 "confidence": float(scores[i]), "xyxy": [float(x) for x in boxes[i]],
                 "is_defect": CLASSES[labels[i]] in DEFECT_CLASSES} for i in ids]


def annotate(image, detections):
    result = image.copy()
    for det in detections:
        box = np.round(det["xyxy"]).astype(int)
        color = COLORS[det["class_id"]]
        cv2.rectangle(result, tuple(box[:2]), tuple(box[2:]), color, 2)
        cv2.putText(result, f"{det['class_name']} {det['confidence']:.2f}",
                    (int(box[0]), max(16, int(box[1]) - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
    count = sum(d["is_defect"] for d in detections)
    cv2.putText(result, f"Defects: {count} | Glass: {len(detections) - count}",
                (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (20, 20, 20), 3)
    cv2.putText(result, f"Defects: {count} | Glass: {len(detections) - count}",
                (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (250, 250, 250), 1)
    return result


def read_image(path):
    """字节读取兼容 Windows 中文路径。"""
    image = cv2.imdecode(np.frombuffer(Path(path).read_bytes(), dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"无法读取图像: {path}")
    return image


def write_image(path, image):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    ok, encoded = cv2.imencode(path.suffix, image)
    if not ok:
        raise ValueError(f"无法编码图像: {path}")
    path.write_bytes(encoded.tobytes())
