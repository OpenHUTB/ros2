#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线评价：在多场景视频上评测检测+识别性能。

对每段真值帧区间：
    - 逐帧运行 预处理 + CNN 推理；
    - 区间内“被检测到帧的多数投票”作为该段识别结果，与真值比对；
    - 统计检测率、平均置信度、单帧推理耗时。

输出：artifacts/scene_eval.json、artifacts/scene_examples.png、控制台 markdown 表格。

用法：python3 scripts/evaluate_scenes.py
"""
import json
import os
import sys
import time
from collections import Counter

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from mnist_digit_control.model import load_model  # noqa: E402
from mnist_digit_control.preprocess import extract_digit  # noqa: E402
from scripts.train_mnist import MODULE_DIR  # noqa: E402

import torch  # noqa: E402


def evaluate_video(model, video_path: str, segments: list, conf_gate: float = 0.7) -> list:
    cap = cv2.VideoCapture(video_path)
    results = []
    frame_id = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        seg = next((s for s in segments if s["start_frame"] <= frame_id <= s["end_frame"]), None)
        if seg is not None:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            t0 = time.perf_counter()
            digit28, _ = extract_digit(gray)
            ms = (time.perf_counter() - t0) * 1000.0
            pred, conf = -1, 0.0
            if digit28 is not None:
                x = torch.from_numpy(digit28.astype(np.float32)[None, None] / 255.0)
                with torch.no_grad():
                    probs = torch.softmax(model(x), dim=1)[0]
                conf, pred = float(probs.max()), int(probs.argmax())
            seg.setdefault("frames", []).append(
                {"pred": pred, "conf": conf, "ms": ms, "frame_id": frame_id}
            )
        frame_id += 1
    cap.release()

    for seg in segments:
        preds = [f["pred"] for f in seg["frames"] if f["pred"] >= 0]
        gated = [f["pred"] for f in seg["frames"] if f["pred"] >= 0 and f["conf"] >= conf_gate]
        detected = len(preds)
        majority = Counter(preds).most_common(1)[0][0] if preds else -1
        # 门限后多数投票：只统计 conf>=conf_gate 的帧，与在线防抖触发条件一致
        gated_majority = Counter(gated).most_common(1)[0][0] if gated else -1
        confs = [f["conf"] for f in seg["frames"] if f["pred"] >= 0]
        mss = [f["ms"] for f in seg["frames"]]
        results.append({
            "label": seg["label"], "majority": majority, "gated_majority": gated_majority,
            "correct": int(majority == seg["label"]),
            "gated_correct": int(gated_majority == seg["label"]),
            "detected_ratio": detected / max(1, len(seg["frames"])),
            "mean_conf": float(np.mean(confs)) if confs else 0.0,
            "mean_ms": float(np.mean(mss)) if mss else 0.0,
        })
    return results


def sample_frames(video_path: str, segments: list) -> list:
    """每个视频取第一段指令的中间帧，用于生成示例图。"""
    cap = cv2.VideoCapture(video_path)
    want = segments[0]["start_frame"] + 5
    frame = None
    for i in range(want + 1):
        ok, frame = cap.read()
        if not ok:
            break
    cap.release()
    return frame


def main() -> None:
    model_path = os.path.join(MODULE_DIR, "models", "mnist_cnn.pth")
    model = load_model(model_path)
    gt_path = os.path.join(MODULE_DIR, "artifacts", "scene_ground_truth.json")
    with open(gt_path, encoding="utf-8") as fh:
        gt = json.load(fh)

    all_rows, summary = [], {}
    examples = []
    for video, segments in gt["scenes"].items():
        path = os.path.join(MODULE_DIR, "videos", video)
        rows = evaluate_video(model, path, segments)
        all_rows.append((video, rows))
        acc = np.mean([r["correct"] for r in rows])
        gacc = np.mean([r["gated_correct"] for r in rows])
        det = np.mean([r["detected_ratio"] for r in rows])
        conf = np.mean([r["mean_conf"] for r in rows])
        ms = np.mean([r["mean_ms"] for r in rows])
        summary[video] = {"accuracy": float(acc), "gated_accuracy": float(gacc),
                          "detected_ratio": float(det),
                          "mean_conf": float(conf), "mean_infer_ms": float(ms)}
        example = sample_frames(path, segments)
        if example is not None:
            examples.append((video, example, rows[0]))

    # 控制台表格
    print("| 场景 | 原始识别准确率 | 门限后准确率 | 平均检测率 | 平均置信度 | 单帧推理耗时(ms) |")
    print("| --- | ---: | ---: | ---: | ---: | ---: |")
    for video, s in summary.items():
        print(f"| {video} | {s['accuracy']:.2%} | {s['gated_accuracy']:.2%} "
              f"| {s['detected_ratio']:.2%} | {s['mean_conf']:.3f} | {s['mean_infer_ms']:.2f} |")

    out_json = os.path.join(MODULE_DIR, "artifacts", "scene_eval.json")
    with open(out_json, "w", encoding="utf-8") as fh:
        json.dump({"summary": summary, "detail": {v: r for v, r in all_rows}},
                  fh, ensure_ascii=False, indent=2)
    print(f"详细结果已写入 {out_json}")

    # 示例图：4 个场景各取一帧
    if examples:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(1, len(examples), figsize=(3.2 * len(examples), 2.9), dpi=110)
        for ax, (video, frame, row) in zip(axes, examples):
            ax.imshow(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            ax.set_title(f"{video.replace('scene_', '').replace('.mp4', '')}\n"
                         f"true={row['label']} pred={row['majority']} conf={row['mean_conf']:.2f}",
                         fontsize=9)
            ax.axis("off")
        fig.suptitle("Scene examples: ground truth vs. majority prediction", fontsize=10)
        fig.tight_layout()
        fig.savefig(os.path.join(MODULE_DIR, "artifacts", "scene_examples.png"),
                    bbox_inches="tight")
        print("示例图已保存 artifacts/scene_examples.png")


if __name__ == "__main__":
    main()
