# -*- coding: utf-8 -*-
"""检测节点：车载相机图像 -> 限速值。

流水线（经典 CV 定位 + 神经网络识别）：
1. HSV 红色分割 + 圆度/半径筛选，定位限速牌圆形 ROI；
2. 在 ROI 内按暗色连通域切分 1~2 个数字字符；
3. DigitCNN 逐字符分类（0-9），按位组合成限速值；
4. 滑窗防抖后发布 /speed_limit（Int32，-1 表示视野内无牌）与置信度：
   稳定值取窗口内票数最多的非负读数（>=2 票，置信度均值决胜），
   上锁/改判同规则；整窗无有效读数才解除。容忍退化场景下的散乱误读。
"""
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Float32

from .model import load_model, preprocess_char


def find_sign_roi(bgr):
    """HSV 红色分割定位圆形标志牌，返回 (cx, cy, r) 或 None。"""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m1 = cv2.inRange(hsv, (0, 90, 60), (8, 255, 255))
    m2 = cv2.inRange(hsv, (172, 90, 60), (180, 255, 255))
    mask = cv2.morphologyEx(m1 | m2, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for c in contours:
        area = cv2.contourArea(c)
        if area < 150:
            continue
        per = cv2.arcLength(c, True)
        if per <= 0:
            continue
        circularity = 4.0 * np.pi * area / (per * per)
        if circularity < 0.6:      # 圆形度筛选
            continue
        (cx, cy), r = cv2.minEnclosingCircle(c)
        if not (15 <= r <= 110):   # 合理的牌面半径范围（px）
            continue
        if best is None or area > best[3]:
            best = (int(cx), int(cy), int(r), area)
    if best is None:
        return None
    return best[0], best[1], best[2]


def split_chars(bgr, cx, cy, r):
    """在圆形 ROI 内切分数字字符，返回按 x 排序的灰度图列表。"""
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    H, W = gray.shape
    side = 2 * r
    x0, y0 = max(0, cx - r), max(0, cy - r)
    x1, y1 = min(W, cx + r), min(H, cy + r)
    roi = gray[y0:y1, x0:x1]
    if roi.size == 0:
        return []
    # 只保留圆内暗色（数字）像素
    h, w = roi.shape
    yy, xx = np.mgrid[0:h, 0:w]
    inside = (xx + x0 - cx) ** 2 + (yy + y0 - cy) ** 2 <= (r * 0.92) ** 2
    dark = ((roi < 110) & inside).astype(np.uint8) * 255
    n, labels, stats, _ = cv2.connectedComponentsWithStats(dark, 8)
    boxes = []
    for i in range(1, n):
        x, y, bw, bh, area = stats[i]
        if area < 20 or not (0.18 * r <= bh <= 0.95 * r) or bw > 0.9 * r:
            continue
        if bw < 2 or bh < 6:
            continue
        boxes.append((x + x0, y + y0, bw, bh))
    boxes.sort(key=lambda b: b[0])
    boxes = boxes[:2]              # 最多两位数
    chars = []
    for x, y, bw, bh in boxes:
        pad = max(2, int(0.12 * bh))
        x0c, y0c = max(0, x - pad), max(0, y - pad)
        x1c, y1c = min(W, x + bw + pad), min(H, y + bh + pad)
        chars.append(gray[y0c:y1c, x0c:x1c])
    return chars


class DetectorNode(Node):
    def __init__(self):
        super().__init__("detector_node")
        self.declare_parameter("conf_threshold", 0.60)   # 字符置信度门限
        self.declare_parameter("debounce_n", 5)          # 防抖滑窗长度（帧）
        self.declare_parameter("debug_image", True)
        self.model, self.device = load_model()
        self.conf_th = float(self.get_parameter("conf_threshold").value)
        self.debounce_n = int(self.get_parameter("debounce_n").value)
        self.debug = bool(self.get_parameter("debug_image").value)
        self.window = deque(maxlen=self.debounce_n)   # (读数, 置信度) 滑窗
        self.stable = -1
        self.stable_conf = 0.0
        self.sub = self.create_subscription(Image, "camera/image_raw",
                                            self.on_image, 10)
        self.pub_val = self.create_publisher(Int32, "speed_limit", 10)
        self.pub_conf = self.create_publisher(Float32, "speed_limit_conf", 10)
        self.pub_dbg = self.create_publisher(Image, "detector/debug", 10)
        self.infer_ms = 0.0
        self.get_logger().info(
            f"detector 就绪: conf>={self.conf_th}, debounce={self.debounce_n}, device={self.device}")

    def classify(self, chars):
        """逐字符 CNN 分类，返回 (限速值, 平均置信度)；无效返回 (-1, 0)。"""
        if not chars:
            return -1, 0.0
        import torch
        vals, confs = [], []
        with torch.no_grad():
            for c in chars:
                t = preprocess_char(c)
                if t is None:
                    return -1, 0.0
                prob = torch.softmax(self.model(t.to(self.device)), dim=1)[0]
                conf, idx = float(prob.max()), int(prob.argmax())
                if conf < self.conf_th:
                    return -1, conf
                vals.append(idx)
                confs.append(conf)
        if len(vals) == 1:
            value = vals[0]
        else:
            value = vals[0] * 10 + vals[1]
        # 合理限速值域过滤（10~99 且非个位 0/1 的组合直接由模型置信度约束）
        if value < 1:
            return -1, min(confs)
        return value, float(np.mean(confs))

    def on_image(self, msg):
        import time
        t0 = time.perf_counter()
        buf = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.width, 3)
        bgr = buf[:, :, ::-1].copy()
        roi = find_sign_roi(bgr)
        dbg = bgr.copy() if self.debug else None
        if roi is None:
            value, conf = -1, 0.0
        else:
            cx, cy, r = roi
            chars = split_chars(bgr, cx, cy, r)
            value, conf = self.classify(chars)
            if dbg is not None:
                cv2.circle(dbg, (cx, cy), r, (60, 220, 60), 2)
        # 滑窗防抖（对称多数票）：稳定值 = 窗口内票数最多的非负读数
        # （>=2 票，同票数时置信度均值决胜）；整窗无有效读数才解除。
        # 对上锁与改判一视同仁，散乱误读既无法上锁也无法长期霸占稳定值。
        self.window.append((value, conf))
        pos = [(v, c) for v, c in self.window if v > 0]
        if len(self.window) == self.window.maxlen and not pos:
            self.stable = -1                 # 整窗无牌：解除
            self.stable_conf = 0.0
        elif pos:
            groups = {}
            for v, c in pos:
                groups.setdefault(v, []).append(c)
            best = max(groups, key=lambda v: (len(groups[v]),
                                              float(np.mean(groups[v]))))
            if len(groups[best]) >= 2 and best != self.stable:
                self.stable = best
            if self.stable == best:
                self.stable_conf = float(np.mean(groups[best]))
        self.infer_ms = (time.perf_counter() - t0) * 1000.0
        self.pub_val.publish(Int32(data=self.stable))
        self.pub_conf.publish(Float32(data=self.stable_conf))
        if dbg is not None:
            tag = "no sign" if self.stable < 0 else f"limit {self.stable} km/h"
            cv2.rectangle(dbg, (0, 0), (150, 22), (30, 30, 30), -1)
            cv2.putText(dbg, tag, (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (120, 255, 120), 1, cv2.LINE_AA)
            out = Image()
            out.header.stamp = msg.header.stamp
            out.height, out.width = dbg.shape[:2]
            out.encoding = "bgr8"
            out.step = out.width * 3
            out.data = dbg.tobytes()
            self.pub_dbg.publish(out)
        self.get_logger().debug(
            f"roi={roi} value={value} conf={conf:.2f} stable={self.stable} {self.infer_ms:.1f}ms")


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
