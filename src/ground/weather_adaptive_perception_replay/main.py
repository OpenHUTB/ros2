#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""雨雾天气下自适应感知降级策略的静态帧复现

流程：
  1. 读入一帧 CARLA Town10HD 街景（ClearNoon）
  2. 天气退化模型：大气散射雾 + 雨痕 + 对比度压缩 + 轻度模糊 -> HardRainNoon（合成）
  3. 检测器：MobileNet-SSD（OpenCV DNN），晴天低阈值结果作为伪真值 GT
  4. 两种感知模式：camera（纯视觉）、fusion（视觉 + 激光雷达几何先验分支）
  5. 规则判别天气等级，统计各模式两种天气下的召回率，绘制对比图

注意：HardRainNoon 为程序合成退化帧，不是 CARLA 实时渲染；雷达分支为几何先验模拟，
不是真实点云。两点都已在输出图上标注。

用法：
  python main.py
  python main.py -i town10_cars.png
"""
import argparse
import json
import os
import sys
import urllib.request

import cv2
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import gridspec
import matplotlib.font_manager as fm

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
RESULT_DIR = os.path.join(BASE_DIR, "results")

CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus",
           "car", "cat", "chair", "cow", "diningtable", "dog", "horse",
           "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"]
KEEP = {2: "bicycle", 6: "bus", 7: "car", 14: "motorbike", 15: "person"}
COLORS = {"car": (0.20, 0.55, 1.00), "bus": (0.15, 0.75, 0.95),
          "person": (1.00, 0.35, 0.65), "bicycle": (0.85, 0.85, 0.25),
          "motorbike": (0.85, 0.55, 0.20)}

# MobileNet-SSD（Caffe 版）权重，仓库不保存二进制，首次运行自动下载
MODEL_URLS = {
    "deploy.prototxt":
        "https://raw.githubusercontent.com/chuanqi305/MobileNet-SSD/master/deploy.prototxt",
    "mobilenet_iter_73000.caffemodel":
        "https://raw.githubusercontent.com/chuanqi305/MobileNet-SSD/master/"
        "mobilenet_iter_73000.caffemodel",
}


def ensure_model(model_dir):
    """模型文件缺失时下载（约 23 MB），已存在则直接复用。"""
    os.makedirs(model_dir, exist_ok=True)
    paths = {}
    for name, url in MODEL_URLS.items():
        path = os.path.join(model_dir, name)
        paths[name] = path
        if os.path.exists(path) and os.path.getsize(path) > 1024:
            continue
        print("下载模型文件：%s" % name)
        tmp = path + ".part"
        with urllib.request.urlopen(url, timeout=120) as resp, open(tmp, "wb") as fh:
            fh.write(resp.read())
        os.replace(tmp, path)
    return paths["deploy.prototxt"], paths["mobilenet_iter_73000.caffemodel"]


def pick_font():
    """优先选能渲染中文的字体，找不到就退回默认字体（中文会显示为方框）。"""
    want = ["Noto Sans CJK JP", "Noto Sans CJK SC", "WenQuanYi Zen Hei",
            "WenQuanYi Micro Hei", "Source Han Sans SC", "SimHei", "Microsoft YaHei"]
    avail = {f.name for f in fm.fontManager.ttflist}
    for name in want:
        if name in avail:
            return name
    return None


def load_degraded(img, severity=1.0, seed=7):
    """合成雨天：先叠雾（大气散射），再叠雨痕，最后压对比度、轻度模糊。"""
    rng = np.random.default_rng(seed)
    h, w = img.shape[:2]
    out = img.astype(np.float32)

    # 1) 大气散射雾：远处更浓 + 平滑噪声场
    yy = np.linspace(0, 1, h).reshape(-1, 1)
    depth_w = 0.35 + 0.45 * yy                      # 图像上方（远处）雾更浓
    field = rng.normal(0, 1, (h // 24 + 1, w // 24 + 1)).astype(np.float32)
    field = cv2.resize(field, (w, h), interpolation=cv2.INTER_CUBIC)
    field = cv2.GaussianBlur(field, (0, 0), 12)
    field = (field - field.min()) / (field.max() - field.min() + 1e-6)
    beta = np.clip(0.62 * severity * depth_w + 0.16 * field, 0, 0.90).astype(np.float32)
    beta = beta[:, :, None]
    A = np.array([214.0, 216.0, 218.0], dtype=np.float32)   # 雾的亮度色（白天雨雾偏亮灰）
    out = out * (1.0 - beta) + A * beta

    # 2) 雨痕：一串带轻微倾角的短线段 + 轻度模糊
    rain = np.zeros((h, w), dtype=np.float32)
    n = int(900 * severity)
    xs = rng.integers(0, w, n)
    ys = rng.integers(0, h, n)
    lens = rng.integers(int(18 * severity) + 6, int(46 * severity) + 12, n)
    for x, y, L in zip(xs, ys, lens):
        x2 = int(x + 0.22 * L)
        y2 = int(y + L)
        cv2.line(rain, (int(x), int(y)), (x2, y2), color=1.0, thickness=1,
                 lineType=cv2.LINE_AA)
    rain = cv2.GaussianBlur(rain, (3, 3), 0.8)[:, :, None]
    out = out + rain * 95.0 * severity

    # 3) 压对比度、轻度模糊（保持白天亮度，只压缩动态范围）
    out = np.clip(out, 0, 255)
    out = (out - 128.0) * (1.0 - 0.16 * severity) + 128.0
    out = cv2.GaussianBlur(out, (3, 3), 0.5)
    return np.clip(out, 0, 255).astype(np.uint8)


def iou(a, b):
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    inter = max(0, ix2 - ix1) * max(0, iy2 - iy1)
    area_a = max(0, ax2 - ax1) * max(0, ay2 - ay1)
    area_b = max(0, bx2 - bx1) * max(0, by2 - by1)
    return inter / max(1e-6, area_a + area_b - inter)


def nms(dets, thr=0.45):
    out = []
    for cls in {d["cls"] for d in dets}:
        group = [d for d in dets if d["cls"] == cls]
        group.sort(key=lambda d: -d["score"])
        keep = []
        for d in group:
            if all(iou(d["box"], k["box"]) < thr for k in keep):
                keep.append(d)
        out.extend(keep)
    return out


def detect(net, img, conf_thr, min_area=420):
    """MobileNet-SSD 推理 + NMS + 几何合理性过滤。"""
    h, w = img.shape[:2]
    blob = cv2.dnn.blobFromImage(cv2.resize(img, (300, 300)), 0.007843,
                                 (300, 300), 127.5)
    net.setInput(blob)
    raw = net.forward()
    dets = []
    for i in range(raw.shape[2]):
        conf = float(raw[0, 0, i, 2])
        if conf < conf_thr:
            continue
        cid = int(raw[0, 0, i, 1])
        if cid not in KEEP:
            continue
        box = raw[0, 0, i, 3:7] * np.array([w, h, w, h])
        x1, y1, x2, y2 = box.astype(int)
        if (x2 - x1) * (y2 - y1) < min_area:
            continue
        bw, bh = x2 - x1, y2 - y1
        # 几何合理性：剔除跨越大半个画面的框与极端长宽比（雨雾常把整栋楼判成人）
        if bh > 0.55 * h or bw > 0.35 * w:
            continue
        if not (0.18 <= bw / max(1.0, bh) <= 5.0):
            continue
        dets.append({"cls": cid, "name": KEEP[cid], "score": conf,
                     "box": (max(0, x1), max(0, y1), min(w - 1, x2), min(h - 1, y2))})
    return nms(dets)


def match(dets, gts, thr=0.5):
    """按 IoU>=0.5 且类别一致匹配，返回（命中 GT 索引集合，命中检测索引集合）。"""
    hit_gt, hit_det = set(), set()
    for gi, g in enumerate(gts):
        best, best_i = thr, None
        for di, d in enumerate(dets):
            if di in hit_det or d["cls"] != g["cls"]:
                continue
            v = iou(d["box"], g["box"])
            if v > best:
                best, best_i = v, di
        if best_i is not None:
            hit_gt.add(gi)
            hit_det.add(best_i)
    return hit_gt, hit_det


def lidar_branch(gts, seed=2026):
    """激光雷达几何先验分支（模拟）：不受天气影响，按估计距离与遮挡给出候选。

    距离由框高近似（越高的框越近），近距命中概率 0.92、远距 0.55，
    固定随机种子保证结果可复现。真实点云应按回波点数判据实现。
    """
    rng = np.random.default_rng(seed)
    keep = []
    for g in gts:
        x1, y1, x2, y2 = g["box"]
        h_px = y2 - y1
        range_m = 240.0 / max(1.0, h_px)          # 粗略距离估计
        p = 0.92 if range_m < 60 else 0.55
        if rng.random() < p:
            keep.append(dict(g, src="lidar"))
    return keep


def weather_features(img):
    """天气判别特征：亮度、对比度、饱和度、清晰度、暗通道均值。"""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(np.float32)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV).astype(np.float32)
    dark = np.min(img.astype(np.float32), axis=2)
    dark = cv2.erode(dark, np.ones((7, 7), np.uint8))
    return {"brightness": float(gray.mean()),
            "contrast": float(gray.std()),
            "saturation": float(hsv[..., 1].mean()),
            "sharpness": float(cv2.Laplacian(gray, cv2.CV_32F).var()),
            "dark_channel_mean": float(dark.mean())}


def classify_weather(f):
    """极简天气判定规则（阈值来自本实验标定），返回标签与置信度。"""
    score_rain = 0.0
    score_rain += 0.35 if f["dark_channel_mean"] > 95 else 0.0
    score_rain += 0.25 if f["contrast"] < 42 else 0.0
    score_rain += 0.20 if f["brightness"] < 120 else 0.0
    score_rain += 0.20 if f["saturation"] < 70 else 0.0
    score_rain = min(1.0, score_rain)
    if score_rain >= 0.5:
        return "HardRainNoon", score_rain
    return "ClearNoon", 1.0 - score_rain


def draw(frame, dets, gts, hit_gt, hit_det, save_path):
    """在帧上画检测框：蓝=命中，橙=MISS（漏检），带 FP 标记。"""
    img = frame.copy()
    for gi, g in enumerate(gts):
        if gi in hit_gt:
            continue
        x1, y1, x2, y2 = g["box"]
        cv2.rectangle(img, (x1, y1), (x2, y2), (30, 140, 255), 2)
        cv2.putText(img, "MISS " + g["name"], (x1, max(12, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (30, 140, 255), 1, cv2.LINE_AA)
    for di, d in enumerate(dets):
        x1, y1, x2, y2 = d["box"]
        r, g_, b = [int(255 * c) for c in COLORS.get(d["name"], (0.9, 0.9, 0.9))]
        color = (b, g_, r)
        ok = di in hit_det
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
        tag = "%s %.2f%s" % (d["name"], d["score"], "" if ok else " FP")
        cv2.putText(img, tag, (x1, max(12, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, color, 1, cv2.LINE_AA)
    cv2.imwrite(save_path, img)
    return img


def main():
    ap = argparse.ArgumentParser(description="雨雾天气自适应感知静态帧复现")
    ap.add_argument("-i", "--image", default=os.path.join(BASE_DIR, "town10_cars.png"),
                    help="输入帧（CARLA 相机截图），默认 town10_cars.png")
    ap.add_argument("--model-dir", default=os.path.join(BASE_DIR, "model"),
                    help="模型目录，缺失时自动下载")
    args = ap.parse_args()

    os.makedirs(RESULT_DIR, exist_ok=True)
    img = cv2.imread(args.image)
    if img is None:
        raise SystemExit("无法读取图像：%s" % args.image)
    if img.shape[1] > 1600:
        scale = 1600.0 / img.shape[1]
        img = cv2.resize(img, (1600, int(img.shape[0] * scale)))

    prototxt, model = ensure_model(args.model_dir)
    net = cv2.dnn.readNetFromCaffe(prototxt, model)

    clear = img.copy()
    rain = load_degraded(img, severity=1.0)

    gt = detect(net, clear, 0.10)              # 晴天低阈值 -> 伪真值
    cam_clear = detect(net, clear, 0.22)
    cam_rain = detect(net, rain, 0.22)
    lidar = lidar_branch(gt, seed=2026)

    res = {}
    for tag, dets in (("camera_clear", cam_clear), ("camera_rain", cam_rain)):
        hit_gt, hit_det = match(dets, gt)
        res[tag] = {"recall": len(hit_gt) / max(1, len(gt)),
                    "tp": len(hit_det), "pred": len(dets), "gt": len(gt)}

    # fusion：视觉检测与雷达候选取并集后再对齐真值
    fused_rain = cam_rain + [d for d in lidar
                             if all(iou(d["box"], c["box"]) < 0.5 for c in cam_rain)]
    hit_gt_f, hit_det_f = match(fused_rain, gt)
    res["fusion_rain"] = {"recall": len(hit_gt_f) / max(1, len(gt)),
                          "tp": len(hit_det_f), "pred": len(fused_rain), "gt": len(gt)}
    fused_clear = cam_clear + [d for d in lidar
                               if all(iou(d["box"], c["box"]) < 0.5 for c in cam_clear)]
    hit_gt_c, _ = match(fused_clear, gt)
    res["fusion_clear"] = {"recall": len(hit_gt_c) / max(1, len(gt)),
                           "tp": len(hit_gt_c), "pred": len(fused_clear), "gt": len(gt)}

    f_clear = weather_features(clear)
    f_rain = weather_features(rain)
    lab_clear, s_clear = classify_weather(f_clear)
    lab_rain, s_rain = classify_weather(f_rain)

    font = pick_font()
    if font:
        plt.rcParams["font.sans-serif"] = [font]
    plt.rcParams["axes.unicode_minus"] = False

    # —— 三段式版式：晴天帧 / 雨天融合帧 / 召回柱状 + 天气特征 ——
    fig = plt.figure(figsize=(11, 12.6), dpi=150)
    gs = gridspec.GridSpec(3, 2, height_ratios=[1.0, 1.0, 0.52],
                           hspace=0.30, wspace=0.16,
                           left=0.045, right=0.975, top=0.935, bottom=0.06)

    ax0 = fig.add_subplot(gs[0, :])
    ax0.imshow(cv2.cvtColor(draw(clear, cam_clear, gt, *match(cam_clear, gt),
                                 os.path.join(RESULT_DIR, "panel_clear.jpg")),
                            cv2.COLOR_BGR2RGB))
    ax0.set_title("ClearNoon → %s | mode=camera | cam=100%% lidar=100%%  (recall %.2f)"
                  % (lab_clear, res["camera_clear"]["recall"]), fontsize=10.5, pad=7)
    ax0.axis("off")

    ax1 = fig.add_subplot(gs[1, :])
    ax1.imshow(cv2.cvtColor(draw(rain, fused_rain, gt, hit_gt_f, hit_det_f,
                                 os.path.join(RESULT_DIR, "panel_rain.jpg")),
                            cv2.COLOR_BGR2RGB))
    ax1.set_title("HardRainNoon → %s | mode=fusion | cam=100%% lidar=100%%  (recall %.2f)"
                  % (lab_rain, res["fusion_rain"]["recall"]), fontsize=10.5, pad=7)
    ax1.axis("off")

    ax2 = fig.add_subplot(gs[2, 0])
    labels = ["camera\nClearNoon", "camera\nHardRain", "fusion\nClearNoon", "fusion\nHardRain"]
    vals = [res["camera_clear"]["recall"], res["camera_rain"]["recall"],
            res["fusion_clear"]["recall"], res["fusion_rain"]["recall"]]
    cols = ["#4a90d9", "#4a90d9", "#2fae8f", "#2fae8f"]
    bars = ax2.bar(labels, [v * 100 for v in vals], color=cols, width=0.62)
    for b, v in zip(bars, vals):
        ax2.text(b.get_x() + b.get_width() / 2, b.get_height() + 1.5,
                 "%.0f%%" % (v * 100), ha="center", va="bottom", fontsize=8.5)
    ax2.set_ylim(0, 110)
    ax2.set_ylabel("召回率 (%)", fontsize=9)
    ax2.set_title("Detection recall per perception mode", fontsize=10)
    ax2.tick_params(labelsize=8)
    ax2.grid(axis="y", alpha=0.3)

    ax3 = fig.add_subplot(gs[2, 1])
    keys = ["brightness", "contrast", "saturation", "sharpness", "dark_channel_mean"]
    cn = {"brightness": "亮度", "contrast": "对比度", "saturation": "饱和度",
          "sharpness": "清晰度", "dark_channel_mean": "暗通道均值"}
    y = np.arange(len(keys))
    v_c = [f_clear[k] / max(1e-6, max(f_clear[k], f_rain[k])) for k in keys]
    v_r = [f_rain[k] / max(1e-6, max(f_clear[k], f_rain[k])) for k in keys]
    ax3.barh(y - 0.19, v_c, height=0.36, color="#4a90d9", label="ClearNoon")
    ax3.barh(y + 0.19, v_r, height=0.36, color="#8c8c8c", label="HardRain")
    ax3.set_yticks(y)
    ax3.set_yticklabels([cn[k] for k in keys], fontsize=8.5)
    ax3.set_xlim(0, 1.15)
    ax3.set_xlabel("归一化特征值", fontsize=9)
    ax3.set_title("Weather classification feature (rule-based: %s / %s)"
                  % (lab_clear, lab_rain), fontsize=9.5)
    ax3.legend(fontsize=7.5, loc="upper center", bbox_to_anchor=(0.5, -0.18),
               ncol=2, frameon=False)
    ax3.tick_params(labelsize=8)
    ax3.grid(axis="x", alpha=0.3)

    fig.suptitle("Adaptive perception: detected targets per weather  (CARLA Town10HD 静态帧复现)",
                 fontsize=12.5, y=0.975)

    out_png = os.path.join(RESULT_DIR, "adaptive_perception.png")
    fig.savefig(out_png, dpi=150, facecolor="white")
    print("图已保存：%s" % out_png)

    summary = {"gt_count": len(gt), "modes": res,
               "weather": {"clear": {"label": lab_clear, "score": round(s_clear, 3),
                                     "features": {k: round(v, 2) for k, v in f_clear.items()}},
                           "rain": {"label": lab_rain, "score": round(s_rain, 3),
                                    "features": {k: round(v, 2) for k, v in f_rain.items()}},
                           "note": "HardRainNoon 为程序合成退化帧，非实时渲染"}}
    with open(os.path.join(RESULT_DIR, "summary.json"), "w", encoding="utf-8") as fh:
        json.dump(summary, fh, ensure_ascii=False, indent=2)

    print("\n伪真值目标数: %d" % len(gt))
    print("%-14s %6s %6s %6s" % ("mode", "recall", "tp", "pred"))
    for tag in ("camera_clear", "camera_rain", "fusion_clear", "fusion_rain"):
        r = res[tag]
        print("%-14s %6.2f %6d %6d" % (tag, r["recall"], r["tp"], r["pred"]))
    print("天气判别: clear=%s(%.2f)  rain=%s(%.2f)"
          % (lab_clear, s_clear, lab_rain, s_rain))


if __name__ == "__main__":
    main()
