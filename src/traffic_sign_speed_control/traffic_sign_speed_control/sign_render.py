# -*- coding: utf-8 -*-
"""标志牌渲染与增广（纯 PIL/numpy，无外部素材依赖）。

限速牌采用国标风格：红圈 + 白底 + 黑色数字；
渲染后再施加几何/光度退化，模拟相机采集的真实条件。
"""
import math
import random

import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageFilter

# 常见无衬线粗体字体（按优先级尝试）
FONT_CANDIDATES = [
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf",
    "/usr/share/fonts/truetype/freefont/FreeSansBold.ttf",
    "C:/Windows/Fonts/arialbd.ttf",
]


def _load_font(size):
    for path in FONT_CANDIDATES:
        try:
            return ImageFont.truetype(path, size)
        except Exception:
            continue
    return ImageFont.load_default()


def draw_limit_sign(value_kmh, size=128, alpha=False):
    """绘制限速牌：红圈白底黑字。

    alpha=False 返回 RGB（白底）；alpha=True 返回 RGBA，圆外四角透明。
    """
    img = Image.new("RGBA", (size, size), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    r = size // 2 - 2
    cx = cy = size // 2
    ring_w = max(4, size // 10)
    # 红色外圈
    d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=(200, 16, 26, 255))
    ri = r - ring_w
    d.ellipse([cx - ri, cy - ri, cx + ri, cy + ri], fill=(255, 255, 255, 255))
    # 数字（一位或两位）
    text = str(int(value_kmh))
    font = _load_font(int(ri * (1.15 if len(text) == 1 else 0.95)))
    bbox = d.textbbox((0, 0), text, font=font)
    tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
    d.text((cx - tw / 2 - bbox[0], cy - th / 2 - bbox[1]), text,
           font=font, fill=(15, 15, 15, 255))
    if alpha:
        return img
    bg = Image.new("RGB", (size, size), (255, 255, 255))
    bg.paste(img, mask=img)
    return bg


def canonicalize_char(gray_u8, size=32):
    """字符归一化（训练与推理共用）：暗像素紧裁剪 -> 白底方形 -> 缩放 size。

    输入 HxW uint8 灰度图；返回 size x size uint8，无有效像素时返回 None。
    """
    a = np.asarray(gray_u8)
    mask = a < 128
    if not mask.any():
        return None
    ys, xs = np.where(mask)
    y0, y1 = int(ys.min()), int(ys.max()) + 1
    x0, x1 = int(xs.min()), int(xs.max()) + 1
    g = a[y0:y1, x0:x1]
    h, w = g.shape
    side = max(h, w)
    canvas = np.full((side, side), 255, np.uint8)
    oy, ox = (side - h) // 2, (side - w) // 2
    canvas[oy:oy + h, ox:ox + w] = g
    return np.asarray(Image.fromarray(canvas).resize(
        (size, size), Image.BILINEAR))


def augment(img, rng=None, rotate=0.0, blur_sigma=0.0, brightness=0.0,
            contrast=0.0, noise_std=0.0):
    """对标志图像施加几何/光度退化（支持 RGB 与 RGBA，alpha 通道不扰动）。

    rotate: 最大旋转角（度，正负随机）；blur_sigma: 最大高斯模糊 σ；
    brightness/contrast: 最大亮度/对比度扰动幅度；noise_std: 高斯噪声强度（0-1）。
    """
    rng = rng or random
    rgba = img.mode == "RGBA"
    fill = (0, 0, 0, 0) if rgba else (255, 255, 255)
    if rotate:
        img = img.rotate(rng.uniform(-rotate, rotate), resample=Image.BILINEAR,
                         fillcolor=fill)
    if blur_sigma:
        img = img.filter(ImageFilter.GaussianBlur(rng.uniform(0.0, blur_sigma)))
    if brightness or contrast or noise_std:
        arr = np.asarray(img).astype(np.float32)
        alpha = arr[..., 3:] if rgba else None
        rgb = arr[..., :3]
        if brightness or contrast:
            rgb /= 255.0
            if brightness:
                rgb += rng.uniform(-brightness, brightness) * 0.25
            if contrast:
                rgb = (rgb - 0.5) * (1.0 + rng.uniform(0.0, contrast)) + 0.5
            rgb = np.clip(rgb, 0.0, 1.0) * 255.0
        if noise_std:
            rgb += np.random.default_rng(rng.randint(0, 2**31)).normal(
                0, noise_std * 255, rgb.shape)
        rgb = np.clip(rgb, 0, 255)
        out = np.concatenate([rgb, alpha], axis=-1) if rgba else rgb
        img = Image.fromarray(out.astype(np.uint8),
                              mode="RGBA" if rgba else "RGB")
    return img


def render_char_dataset(n_per_class=400, size=32, seed=0):
    """生成字符级训练/测试数据：0-9 数字 + 仿真牌面合成与增广。

    返回 (images float32 [N,1,H,W] in [0,1], labels int64 [N])
    """
    rng = random.Random(seed)
    xs, ys = [], []
    for digit in range(10):
        for _ in range(n_per_class):
            big = 96
            canvas = Image.new("RGB", (big, big), (255, 255, 255))
            d = ImageDraw.Draw(canvas)
            font = _load_font(int(big * rng.uniform(0.55, 0.75)))
            bbox = d.textbbox((0, 0), str(digit), font=font)
            tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
            d.text(((big - tw) / 2 - bbox[0], (big - th) / 2 - bbox[1]),
                   str(digit), font=font, fill=(rng.randint(0, 60),) * 3)
            # 部分样本叠加红圈边缘，模拟裁剪到环内的真实牌面
            if rng.random() < 0.5:
                r = big // 2 - rng.randint(0, 3)
                d.ellipse([big // 2 - r, big // 2 - r, big // 2 + r, big // 2 + r],
                          outline=(200, 16, 26), width=rng.randint(2, 6))
            canvas = augment(canvas, rng, rotate=18, blur_sigma=1.2,
                             brightness=0.45, contrast=0.6, noise_std=0.06)
            g = canonicalize_char(np.asarray(canvas.convert("L")), size=size)
            if g is None:
                continue
            xs.append(g.astype(np.float32) / 255.0)
            ys.append(digit)
    x = np.stack(xs)[:, None, :, :]
    y = np.array(ys, dtype=np.int64)
    idx = np.random.default_rng(seed).permutation(len(y))
    return x[idx], y[idx]
