# -*- coding: utf-8 -*-
"""
图像预处理工具：utils

把 CARLA ros-bridge 发布的 `sensor_msgs/Image`（RGB，通常 800x600 或 1280x720）
统一缩放到策略网络的输入尺寸（默认 160x80，窄长条贴合前视车道），再归一化到
[0,1] 并转为 CHW 张量。端到端模型只吃图像，因此这里不引入激光/点云等额外模态。

计算原理：
  1) 缩放：用双线性插值把 H0xW0 缩到 HxW，保留车道与前方车辆的纵向结构；
  2) 裁剪：可裁掉引擎盖与天空等无关区域（ROI），聚焦可驾驶区域；
  3) 归一化：x' = x / 255.0；通道序由 HWC 转为网络期望的 CHW。
"""

import numpy as np

try:
    import cv2
except Exception:  # pragma: no cover - 允许无 cv 环境下仅做语法检查
    cv2 = None


def preprocess_image(img_bgr, target_h=80, target_w=160, roi_top=0.30, roi_bottom=1.0):
    """输入 BGR ndarray (H,W,3)，返回 CHW float32 归一化张量。

    roi_top/roi_bottom 是按高度比例的行裁剪区间：默认丢掉上方 30%（天空），
    引擎盖一般在最底部，由用户按需调小 roi_bottom。
    """
    if cv2 is None:
        raise RuntimeError('需要 opencv-python (cv2) 进行图像预处理')

    h = img_bgr.shape[0]
    y0 = int(h * roi_top)
    y1 = int(h * roi_bottom)
    crop = img_bgr[y0:y1, :, :]

    rgb = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)
    rgb = cv2.resize(rgb, (target_w, target_h), interpolation=cv2.INTER_LINEAR)

    # HWC -> CHW，归一化到 [0,1]
    tensor = rgb.astype('float32') / 255.0
    tensor = np.transpose(tensor, (2, 0, 1))
    return tensor
