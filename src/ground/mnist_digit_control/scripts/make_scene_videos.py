#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成多场景“虚拟相机”演示视频（无需真实摄像头）。

因为测试虚拟机没有摄像头设备，本脚本用 MNIST 测试集数字合成 4 个场景的视频，
模拟摄像头拍摄的“纸上手写数字 / 深色背景白板字 / 旋转数字 / 屏幕显示数字”，
用于检测节点的可复现演示与多场景性能评价：

    scene_paper.mp4    纸质背景 + 深色墨迹数字
    scene_dark.mp4     深色背景 + 白色数字
    scene_rotated.mp4  纸质背景 + 旋转 ±10°~25° 的数字
    scene_screen.mp4   深蓝色屏幕背景 + 亮绿色数字

每个视频按固定指令序列 [2,9,2,5,0,7,2,5] 依次显示数字，同时写出
artifacts/scene_ground_truth.json（每段数字的帧区间真值），供离线评价脚本使用。

视频开头有约 1.2 s 的纯背景“空白段”（无数字）：系统实际运行时，检测节点
与控制节点经 DDS 互相发现需要时间，若视频第 0 帧就出现数字，最早的识别
结果可能在对端订阅建立前发布而丢失，空白段保证第一个指令稳定触发。

用法：python3 scripts/make_scene_videos.py
"""
import json
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from scripts.train_mnist import (  # noqa: E402
    FILES, MODULE_DIR, download_mnist, load_idx_images, load_idx_labels,
)

W, H, FPS = 320, 240, 10
LEAD_FRAMES = 12        # 起始空白帧 1.2 s，覆盖节点间 DDS 发现延迟
DIGIT_FRAMES = 16       # 每个数字显示 1.6 s
GAP_FRAMES = 3          # 数字间空隙 0.3 s
SEQUENCE = [2, 9, 2, 5, 0, 7, 2, 5]


def rng_for(seed: int) -> np.random.Generator:
    return np.random.default_rng(seed)


def paste_digit(bg: np.ndarray, digit_img: np.ndarray, ink: tuple, center: tuple,
                size: int, angle: float = 0.0) -> np.ndarray:
    """把 28x28 白底黑字数字按指定大小/角度/颜色贴到背景上。"""
    scaled = cv2.resize(digit_img, (size, size), interpolation=cv2.INTER_CUBIC)
    if angle != 0.0:
        mat = cv2.getRotationMatrix2D((size / 2, size / 2), angle, 1.0)
        scaled = cv2.warpAffine(scaled, mat, (size, size), flags=cv2.INTER_CUBIC,
                                borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    alpha = np.clip(scaled.astype(np.float32) / 255.0, 0, 1)[..., None]
    x, y = center
    x0, y0 = int(x - size / 2), int(y - size / 2)
    x1, y1 = x0 + size, y0 + size
    px0, py0 = max(0, -x0), max(0, -y0)
    x0, y0 = max(0, x0), max(0, y0)
    x1, y1 = min(W, x1), min(H, y1)
    roi = bg[y0:y1, x0:x1].astype(np.float32)
    a = alpha[py0:py0 + (y1 - y0), px0:px0 + (x1 - x0)]
    ink_arr = np.array(ink, dtype=np.float32)
    bg[y0:y1, x0:x1] = (roi * (1 - a) + ink_arr * a).astype(np.uint8)
    return bg


def make_background(style: str, rng: np.random.Generator) -> np.ndarray:
    if style == "paper":
        base = np.full((H, W, 3), 235, np.float32) + rng.normal(0, 7, (H, W, 3))
    elif style == "dark":
        base = np.full((H, W, 3), 42, np.float32) + rng.normal(0, 5, (H, W, 3))
    elif style == "screen":
        base = np.zeros((H, W, 3), np.float32)
        base[..., 0] = 78.0   # B
        base[..., 2] = 28.0   # R
        base += rng.normal(0, 4, (H, W, 3))
    else:
        raise ValueError(style)
    return np.clip(base, 0, 255).astype(np.uint8)


SCENE_STYLE = {
    "scene_paper": dict(bg="paper", ink=(58, 58, 58),
                        size_range=(70, 100), angle_range=(0, 0), blur=1.1),
    "scene_dark": dict(bg="dark", ink=(228, 228, 228),
                       size_range=(70, 100), angle_range=(0, 0), blur=1.1),
    "scene_rotated": dict(bg="paper", ink=(58, 58, 58),
                          size_range=(80, 110), angle_range=(10, 25), blur=1.3),
    "scene_screen": dict(bg="screen", ink=(90, 255, 130),
                         size_range=(70, 100), angle_range=(0, 0), blur=0.9),
}


def render_scene(style_name: str, images: np.ndarray, labels: np.ndarray,
                 writer: cv2.VideoWriter, rng: np.random.Generator) -> list:
    style = SCENE_STYLE[style_name]
    gt = []
    frame_id = 0
    # 起始空白段：给 DDS 发现与订阅建立留出时间，避免第一个指令丢失。
    # 使用独立随机源，避免消耗主随机流、改变后续数字样本的可复现选择
    lead_rng = rng_for(7)
    for _ in range(LEAD_FRAMES):
        writer.write(make_background(style["bg"], lead_rng))
        frame_id += 1
    for digit in SEQUENCE:
        # 每个数字随机取一张对应类别的测试样本
        candidates = np.where(labels == digit)[0]
        sample = images[int(rng.choice(candidates))]
        size = int(rng.uniform(*style["size_range"]))
        angle = float(rng.uniform(*style["angle_range"]))
        cx = int(rng.uniform(size / 2 + 20, W - size / 2 - 20))
        cy = int(rng.uniform(size / 2 + 20, H - size / 2 - 20))
        start = frame_id
        for _ in range(DIGIT_FRAMES):
            bg = make_background(style["bg"], rng)
            jx = int(rng.uniform(-2, 2))
            jy = int(rng.uniform(-2, 2))
            js = max(40, int(size * rng.uniform(0.97, 1.03)))
            frame = paste_digit(bg, sample, style["ink"], (cx + jx, cy + jy), js, angle)
            if style["blur"] > 0:
                k = max(1, int(style["blur"]))
                frame = cv2.GaussianBlur(frame, (k * 2 + 1, k * 2 + 1), style["blur"])
            writer.write(frame)
            frame_id += 1
        gt.append({"label": digit, "start_frame": start, "end_frame": frame_id - 1})
        for _ in range(GAP_FRAMES):  # 空隙帧
            writer.write(make_background(style["bg"], rng))
            frame_id += 1
    return gt


def main() -> None:
    out_dir = os.path.join(MODULE_DIR, "videos")
    artifact_dir = os.path.join(MODULE_DIR, "artifacts")
    os.makedirs(out_dir, exist_ok=True)
    os.makedirs(artifact_dir, exist_ok=True)

    data_dir = os.path.join(MODULE_DIR, "data", "mnist")
    download_mnist(data_dir)
    # 贴图需要 0..255 的 uint8 图像（load_idx_images 返回 0..1 float）
    images = (load_idx_images(os.path.join(data_dir, FILES[2])) * 255.0).astype(np.uint8)
    labels = load_idx_labels(os.path.join(data_dir, FILES[3]))

    ground_truth = {}
    for i, style_name in enumerate(SCENE_STYLE):
        # 每个场景使用不同随机种子，覆盖不同的书写样本/位置/角度
        rng = rng_for(101 + i)
        # 优先 mp4（mp4v），失败则回退 avi（XVID）
        for ext, fourcc in (("mp4", "mp4v"), ("avi", "XVID")):
            path = os.path.join(out_dir, f"{style_name}.{ext}")
            writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*fourcc), FPS, (W, H))
            if writer.isOpened():
                break
        else:
            raise RuntimeError("无法创建视频写入器（缺少编码器）")
        gt = render_scene(style_name, images, labels, writer, rng)
        writer.release()
        ground_truth[os.path.basename(path)] = gt
        print(f"生成 {path}（{len(gt)} 段指令）")

    gt_path = os.path.join(artifact_dir, "scene_ground_truth.json")
    with open(gt_path, "w", encoding="utf-8") as fh:
        json.dump({"fps": FPS, "frame_size": [W, H], "sequence": SEQUENCE,
                   "scenes": ground_truth}, fh, ensure_ascii=False, indent=2)
    print(f"真值已写入 {gt_path}")


if __name__ == "__main__":
    main()
