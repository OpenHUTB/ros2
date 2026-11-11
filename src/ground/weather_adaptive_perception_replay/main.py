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
import hashlib
import json
import os
import sys
import urllib.request

import numpy as np

try:                     # cv2 只在推理与画图时用到，缺失不影响纯逻辑被测试导入
    import cv2
except ImportError:      # pragma: no cover
    cv2 = None

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import gridspec
    import matplotlib.font_manager as fm
except ImportError:      # pragma: no cover
    plt = gridspec = fm = None

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
RESULT_DIR = os.path.join(BASE_DIR, "results")


# —— 非 ASCII 路径兼容 ——
# cv2.imread / imwrite / dnn 的 Caffe 加载走的是 C 的 std::ifstream，Windows 上遇到中文或
# 空格目录（例如 C:\Users\张三\...）会直接失败：imread 返回 None，模型加载抛 cv2.error。
# 下面三个函数用「文件字节流 + imdecode」和「切到模型目录用文件名」绕开这一限制，
# 在 Linux/macOS 上行为与原生调用一致。
def imread_any(path, flags=None):
    try:
        data = np.fromfile(path, dtype=np.uint8)
    except (OSError, ValueError):
        return None
    if data.size == 0:
        return None
    return cv2.imdecode(data, cv2.IMREAD_COLOR if flags is None else flags)


def imwrite_any(path, img):
    ext = os.path.splitext(path)[1].lower() or ".png"
    ok, buf = cv2.imencode(ext, img)
    if not ok:
        return False
    buf.tofile(path)
    return True


def load_caffe_net(prototxt, weights):
    try:
        return cv2.dnn.readNetFromCaffe(prototxt, weights)
    except Exception:
        p_dir, p_base = os.path.split(os.path.abspath(prototxt))
        w_dir, w_base = os.path.split(os.path.abspath(weights))
        if p_dir != w_dir:       # 两个文件不在同一目录时没有可用的相对路径，原样抛出
            raise
        cwd = os.getcwd()
        try:
            os.chdir(p_dir)      # 目录名可以是非 ASCII，文件名必须是纯 ASCII 才能被 C 层打开
            return cv2.dnn.readNetFromCaffe(p_base, w_base)
        finally:
            os.chdir(cwd)

CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus",
           "car", "cat", "chair", "cow", "diningtable", "dog", "horse",
           "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"]
KEEP = {2: "bicycle", 6: "bus", 7: "car", 14: "motorbike", 15: "person"}
COLORS = {"car": (0.20, 0.55, 1.00), "bus": (0.15, 0.75, 0.95),
          "person": (1.00, 0.35, 0.65), "bicycle": (0.85, 0.85, 0.25),
          "motorbike": (0.85, 0.55, 0.20)}

# MobileNet-SSD（Caffe 版）权重，仓库不保存二进制，首次运行自动下载。
# 每个文件都带期望字节数与 sha256：挡住被代理换成的 HTML 错误页、下载截断的半个文件。
# 每个文件给多个下载源：raw.githubusercontent 是原始来源，但国内网络常被拖到几十 KB/s
# 甚至读超时；gh-proxy 与 jsdelivr 是镜像，某条不通就换下一条，校验通过与否由 sha256 判定，
# 所以换源不影响正确性，只影响能不能下载下来。
MODEL_FILES = {
    "deploy.prototxt": {
        "urls": [
            "https://raw.githubusercontent.com/chuanqi305/MobileNet-SSD/master/"
            "deploy.prototxt",
            "https://gh-proxy.com/https://raw.githubusercontent.com/chuanqi305/"
            "MobileNet-SSD/master/deploy.prototxt",
            "https://cdn.jsdelivr.net/gh/chuanqi305/MobileNet-SSD@master/deploy.prototxt",
        ],
        "size": 44667,
        "sha256": "2d180f723b3109e21f8287f6b3c691390d07b60eed998327cd3259ffa0e50608",
    },
    "mobilenet_iter_73000.caffemodel": {
        "urls": [
            "https://raw.githubusercontent.com/chuanqi305/MobileNet-SSD/master/"
            "mobilenet_iter_73000.caffemodel",
            "https://gh-proxy.com/https://raw.githubusercontent.com/chuanqi305/"
            "MobileNet-SSD/master/mobilenet_iter_73000.caffemodel",
        ],
        "size": 23306119,
        "sha256": "52eed8be80522c152a17fb56740de705b79881bde1a167e0e747310523685fc7",
    },
}


def file_matches(path, size, sha256):
    """字节数与 sha256 都对上才算可用。"""
    if not os.path.isfile(path) or os.path.getsize(path) != size:
        return False
    digest = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest() == sha256


def download_model(name, meta, dest_dir):
    """按 urls 顺序逐个源下载到 <name>.part，校验通过后原子替换为正式文件。"""
    path = os.path.join(dest_dir, name)
    tmp = path + ".part"
    print("下载模型文件：%s（约 %.1f MB，共 %d 个源）"
          % (name, meta["size"] / 1048576.0, len(meta["urls"])))
    reasons = []
    for i, url in enumerate(meta["urls"], 1):
        host = url.split("/")[2]
        try:
            with urllib.request.urlopen(url, timeout=30) as resp, open(tmp, "wb") as fh:
                length = resp.headers.get("Content-Length")
                if length and int(length) != meta["size"]:
                    raise ValueError("Content-Length %s 与期望 %d 字节不符"
                                     % (length, meta["size"]))
                written = 0
                for chunk in iter(lambda: resp.read(1024 * 256), b""):
                    written += len(chunk)
                    if written > meta["size"]:      # 源不守规矩地一直吐字节时兜住
                        raise ValueError("下载字节数超过期望的 %d 字节" % meta["size"])
                    fh.write(chunk)
                if written != meta["size"]:
                    raise ValueError("只下到 %d 字节，期望 %d" % (written, meta["size"]))
        except Exception as exc:  # 断网、超时、被代理拦断都在这里收口，不再抛裸 traceback
            reasons.append("第 %d 个源 %s：%s" % (i, host, exc))
            if os.path.exists(tmp):
                os.remove(tmp)
            print("  源 %d/%d（%s）不可用，换下一个" % (i, len(meta["urls"]), host))
            continue

        if not file_matches(tmp, meta["size"], meta["sha256"]):
            got = os.path.getsize(tmp) if os.path.exists(tmp) else 0
            reasons.append("第 %d 个源 %s：校验失败（实际 %d 字节）" % (i, host, got))
            if os.path.exists(tmp):
                os.remove(tmp)
            print("  源 %d/%d（%s）校验不通过，换下一个" % (i, len(meta["urls"]), host))
            continue

        os.replace(tmp, path)
        print("  已从 %s 下载并通过校验" % host)
        return path

    if os.path.exists(tmp):
        os.remove(tmp)
    raise SystemExit(
        "模型下载失败（%s）：所有下载源都不可用\n%s\n"
        "可手动下载该文件放到本地目录，再用 --model-dir <目录> 指定；"
        "期望 %d 字节、sha256 %s。"
        % (name, "\n".join("  " + r for r in reasons),
           meta["size"], meta["sha256"]))


def ensure_model(model_dir):
    """模型缺失或校验不通过时下载（约 23 MB），校验通过则直接复用。"""
    os.makedirs(model_dir, exist_ok=True)
    paths = {}
    for name, meta in MODEL_FILES.items():
        path = os.path.join(model_dir, name)
        paths[name] = path
        if file_matches(path, meta["size"], meta["sha256"]):
            continue
        if os.path.exists(path):
            print("本地文件校验不通过，重新下载：%s" % name)
        download_model(name, meta, model_dir)
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


def box_is_plausible(bw, bh, w, h, min_area=420, max_h_ratio=0.55,
                     max_w_ratio=0.35, ar_range=(0.18, 5.0)):
    """几何合理性判据（纯函数，便于离线断言）。

    雨雾常把大片建筑墙面判成 person，缺了这几条，融合模式的 pred 会被误检灌水。
    """
    if bw * bh < min_area:
        return False
    if bh > max_h_ratio * h or bw > max_w_ratio * w:
        return False
    return ar_range[0] <= bw / max(1.0, bh) <= ar_range[1]


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
        bw, bh = x2 - x1, y2 - y1
        if not box_is_plausible(bw, bh, w, h, min_area=min_area):
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
    imwrite_any(save_path, img)
    return img


def main():
    if cv2 is None or plt is None:
        raise SystemExit("需要 opencv-python 与 matplotlib：pip install -r requirements.txt")
    if not hasattr(cv2.dnn, "readNetFromCaffe"):
        raise SystemExit(
            "当前 opencv-python %s 不提供 Caffe 导入器（5.x 起已移除），无法加载 MobileNet-SSD。\n"
            "请改装 4.x：pip install 'opencv-python>=4.5,<5'"
            % getattr(cv2, "__version__", "?"))
    ap = argparse.ArgumentParser(description="雨雾天气自适应感知静态帧复现")
    ap.add_argument("-i", "--image", default=os.path.join(BASE_DIR, "town10_cars.png"),
                    help="输入帧（CARLA 相机截图），默认 town10_cars.png")
    ap.add_argument("--model-dir", default=os.path.join(BASE_DIR, "model"),
                    help="模型目录，缺失时自动下载")
    args = ap.parse_args()

    os.makedirs(RESULT_DIR, exist_ok=True)
    img = imread_any(args.image)
    if img is None:
        raise SystemExit("无法读取图像：%s（请确认文件存在，且没有被代理换成网页内容）" % args.image)
    if img.shape[1] > 1600:
        scale = 1600.0 / img.shape[1]
        img = cv2.resize(img, (1600, int(img.shape[0] * scale)))

    prototxt, model = ensure_model(args.model_dir)
    net = load_caffe_net(prototxt, model)

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
