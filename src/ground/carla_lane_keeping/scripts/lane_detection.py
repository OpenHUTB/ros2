#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""车道线检测核心算法（纯 numpy 实现，不依赖 OpenCV / CARLA / ROS）。

为什么不用 OpenCV
-----------------
透视变换、Sobel 梯度、多项式拟合这几件事的数学都很简单，纯 numpy 实现
只有几十行，好处是：
  * 任何环境（Windows / Ubuntu、有无 OpenCV）都能跑
  * 每一步都能单独单元测试，出问题能定位到具体环节
  * 不引入大依赖，虚拟机里安装成本最低

处理流程
--------
    彩色图
      -> 灰度
      -> 方向梯度阈值 + 颜色阈值  -> 二值车道像素掩码
      -> 透视变换（鸟瞰图）
      -> 底部直方图找车道线起点
      -> 滑动窗口 + 多项式拟合
      -> 输出：左右车道线系数、横向偏差、航向偏差、曲率

坐标约定
--------
  * 图像坐标：u 向右，v 向下，原点左上角
  * 鸟瞰图（warped）坐标同样是 u 向右、v 向下，但 v 方向对应「车前方向」：
    v 越大离车越近。因此像素 -> 米 的换算系数为 ``meters_per_pixel_y``。
  * 车辆横向偏差 ``lateral_offset``：正值表示**车辆偏在车道中心右侧**。
  * 航向偏差 ``heading_error``：正值表示**车头相对车道方向偏右**。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from typing import Optional, Sequence, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# 基础图像运算
# ---------------------------------------------------------------------------


def to_grayscale(image: np.ndarray) -> np.ndarray:
    """BGR/BGRA 或灰度图 -> float32 灰度图（0~255）。

    按 BGR 顺序加权（OpenCV 的 BGR2GRAY 权重），确保与常见图像来源一致。
    """
    img = np.asarray(image)
    if img.ndim == 2:
        return img.astype(np.float32)
    if img.ndim != 3 or img.shape[2] < 3:
        raise ValueError(f"不支持的图像形状: {img.shape}")
    # 输入按 BGR 排列（CARLA / OpenCV 默认）
    b = img[..., 0].astype(np.float32)
    g = img[..., 1].astype(np.float32)
    r = img[..., 2].astype(np.float32)
    return 0.114 * b + 0.587 * g + 0.299 * r


def sobel_gradients(gray: np.ndarray, ksize: int = 3) -> Tuple[np.ndarray, np.ndarray]:
    """Sobel 梯度 (gx, gy)，使用边缘复制填充，输出与输入同尺寸。

    gx 对应 u（水平）方向导数，gy 对应 v（垂直）方向导数。
    """
    if gray.ndim != 2:
        raise ValueError("sobel_gradients 需要二维灰度图")
    if ksize != 3:
        raise ValueError("当前只实现了 3x3 Sobel 核")
    g = gray.astype(np.float32)
    pad = np.pad(g, 1, mode="edge")
    # 3x3 Sobel 核
    kx = np.array([[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]], dtype=np.float32)
    ky = np.array([[-1, -2, -1], [0, 0, 0], [1, 2, 1]], dtype=np.float32)

    gx = np.zeros_like(g, dtype=np.float32)
    gy = np.zeros_like(g, dtype=np.float32)
    for dy in range(3):
        for dx in range(3):
            window = pad[dy : dy + g.shape[0], dx : dx + g.shape[1]]
            gx += kx[dy, dx] * window
            gy += ky[dy, dx] * window
    return gx, gy


def gradient_magnitude_direction(
    gray: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """返回 (幅值, 方向弧度, gx)。

    角度定义为 ``atan2(|gy|, |gx|)``，范围 [0, π/2]：

    * **竖直边缘**（车道线的典型形态）：梯度几乎全在水平方向，``|gy|≈0``，
      因此角度 **接近 0°**
    * **水平边缘**（路面接缝、阴影边界）：``|gx|≈0``，角度 **接近 90°**

    注意这与「边缘自身的方向」刚好差 90 度：竖直的车道线给出 0° 的**梯度角**。
    判据必须按梯度角写，曾把两者搞混，导致所有车道线边缘被过滤掉。
    """
    gx, gy = sobel_gradients(gray)
    magnitude = np.hypot(gx, gy)
    angle = np.arctan2(np.abs(gy), np.abs(gx))
    return magnitude.astype(np.float32), angle.astype(np.float32), gx


def color_lane_mask(
    image: np.ndarray,
    white_min: int = 180,
    yellow_b_min: int = 0,
    yellow_g_min: int = 90,
    yellow_r_min: int = 120,
) -> np.ndarray:
    """颜色阈值：提取白色与黄色车道线，返回 bool 掩码。

    输入为 BGR 三通道图（与 CARLA / OpenCV 一致）。
    白色判定：三通道都亮且彼此接近；黄色判定：R、G 明显高于 B、且整体够亮。
    """
    if image.ndim != 3 or image.shape[2] < 3:
        raise ValueError("color_lane_mask 需要三通道 BGR 图")
    b = image[..., 0].astype(np.int16)
    g = image[..., 1].astype(np.int16)
    r = image[..., 2].astype(np.int16)

    maxc = np.maximum(np.maximum(r, g), b)
    minc = np.minimum(np.minimum(r, g), b)

    white = (minc >= white_min) & ((maxc - minc) <= 40)
    yellow = (r >= yellow_r_min) & (g >= yellow_g_min) & (b <= r - 40)
    return (white | yellow).astype(bool)


def directional_gradient_mask(
    gray: np.ndarray,
    sobel_min: float = 25.0,
    sobel_max: float = 800.0,
    angle_max_deg: float = 45.0,
    auto_percentile: float = 98.0,
) -> np.ndarray:
    """基于方向梯度的车道像素掩码。

    只保留**梯度方向接近水平**的边缘，即**图像中接近竖直**的边缘——
    这正是车道线的形态。梯度角 ``atan2(|gy|, |gx|)`` 接近 0° 表示
    竖直边缘，接近 90° 表示水平边缘，因此保留 ``angle <= angle_max_deg``
    的像素。

    参数
    ----
    sobel_min : 梯度幅值下界的**保底值**
    sobel_max : 梯度幅值上界
    auto_percentile : 自动阈值分位数。实际阈值取
        ``max(sobel_min, 该分位数对应的梯度幅值)``。

        为什么需要自适应：真实相机图像（CARLA 前视）的路面纹理梯度中位数
        约 45、90 分位约 260，而合成图像的背景梯度几乎为 0。用同一个固定
        阈值（例如 25）会在真实图像上把 40% 的像素判为车道线，整个路面连成
        一个巨大连通域；反之取高阈值（160）又会让低噪声图像几乎无输出。
        按分位数自适应可以在两类图像上都得到「只有少数强边缘入选」的效果。
    """
    if not (0.0 <= angle_max_deg <= 90.0):
        raise ValueError("angle_max_deg 必须落在 [0, 90] 区间")
    if sobel_max <= 0:
        raise ValueError("sobel_max 必须为正")
    if not (0.0 < auto_percentile <= 100.0):
        raise ValueError("auto_percentile 必须落在 (0, 100] 区间")

    magnitude, angle, _ = gradient_magnitude_direction(gray)

    threshold = float(sobel_min)
    if magnitude.size:
        auto = float(np.percentile(magnitude, auto_percentile))
        threshold = max(threshold, auto)
    if threshold >= sobel_max:
        # 自适应阈值越界时，退回到保底阈值，避免全图无输出
        threshold = float(sobel_min)
        if threshold >= sobel_max:
            raise ValueError("sobel_min 必须小于 sobel_max")

    angle_deg = np.degrees(angle)
    mask = (
        (magnitude >= threshold)
        & (magnitude <= sobel_max)
        & (angle_deg <= angle_max_deg)
    )
    return mask.astype(bool)


# ---------------------------------------------------------------------------
# 透视变换（自实现，不用 cv2）
# ---------------------------------------------------------------------------


def compute_homography(
    src: Sequence[Sequence[float]], dst: Sequence[Sequence[float]]
) -> np.ndarray:
    """由 4 组对应点求 3x3 单应矩阵 H，满足 dst ~ H @ src（齐次坐标）。

    采用直接线性变换（DLT）：每组对应点提供两个方程，8 个未知数（H[2,2] 固定为 1）
    对应 8 个方程，用最小二乘求解。
    """
    src_arr = np.asarray(src, dtype=np.float64).reshape(4, 2)
    dst_arr = np.asarray(dst, dtype=np.float64).reshape(4, 2)

    a_rows = []
    b_vals = []
    for (u, v), (x, y) in zip(src_arr, dst_arr):
        a_rows.append([u, v, 1.0, 0.0, 0.0, 0.0, -x * u, -x * v])
        b_vals.append(x)
        a_rows.append([0.0, 0.0, 0.0, u, v, 1.0, -y * u, -y * v])
        b_vals.append(y)

    a_mat = np.asarray(a_rows, dtype=np.float64)
    b_vec = np.asarray(b_vals, dtype=np.float64)
    solution, *_ = np.linalg.lstsq(a_mat, b_vec, rcond=None)
    h = np.append(solution, 1.0).reshape(3, 3)
    return h


def warp_perspective(
    image: np.ndarray, h: np.ndarray, out_size: Tuple[int, int]
) -> np.ndarray:
    """按单应矩阵 H 把图像变换到目标尺寸 (width, height)。

    使用逆向映射 + 双线性插值，避免正向映射产生空洞。
    越界像素填 0。
    """
    out_w, out_h = int(out_size[0]), int(out_size[1])
    src = np.asarray(image)
    if src.ndim == 2:
        src = src[..., None]

    h_inv = np.linalg.inv(np.asarray(h, dtype=np.float64))

    # 目标网格（像素中心）
    xs, ys = np.meshgrid(
        np.arange(out_w, dtype=np.float64),
        np.arange(out_h, dtype=np.float64),
    )
    ones = np.ones_like(xs)
    coords = np.stack([xs, ys, ones], axis=0).reshape(3, -1)

    src_h = h_inv @ coords
    w = src_h[2]
    # 避免除零；w 为 0 的点标记为越界
    safe_w = np.where(np.abs(w) < 1e-12, 1.0, w)
    su = (src_h[0] / safe_w).reshape(out_h, out_w)
    sv = (src_h[1] / safe_w).reshape(out_h, out_w)

    in_bounds = (np.abs(w).reshape(out_h, out_w) >= 1e-12)
    result = _bilinear_sample(src, su, sv, in_bounds)
    if result.shape[2] == 1:
        return result[:, :, 0]
    return result


def _bilinear_sample(
    src: np.ndarray, su: np.ndarray, sv: np.ndarray, in_bounds: np.ndarray
) -> np.ndarray:
    """对 src 在浮点坐标 (su, sv) 处做双线性采样，越界处填 0。"""
    h, w = src.shape[0], src.shape[1]
    channels = src.shape[2]
    src_f = src.astype(np.float32)

    u0 = np.floor(su).astype(np.int64)
    v0 = np.floor(sv).astype(np.int64)
    du = (su - u0).astype(np.float32)
    dv = (sv - v0).astype(np.float32)

    valid = in_bounds & (u0 >= 0) & (u0 < w - 1) & (v0 >= 0) & (v0 < h - 1)

    # 夹取索引，越界位置的结果稍后被清零
    uc = np.clip(u0, 0, w - 2)
    vc = np.clip(v0, 0, h - 2)

    out = np.zeros((su.shape[0], su.shape[1], channels), dtype=np.float32)
    for c in range(channels):
        p00 = src_f[vc, uc, c]
        p01 = src_f[vc, uc + 1, c]
        p10 = src_f[vc + 1, uc, c]
        p11 = src_f[vc + 1, uc + 1, c]
        top = p00 * (1.0 - du) + p01 * du
        bottom = p10 * (1.0 - du) + p11 * du
        out[:, :, c] = top * (1.0 - dv) + bottom * dv

    out[~valid] = 0.0
    return out


def trapezoid_source_points(
    width: int,
    height: int,
    bottom_ratio: float = 1.0,
    top_ratio: float = 0.58,
    bottom_half_width_ratio: float = 0.42,
    top_half_width_ratio: float = 0.28,
) -> np.ndarray:
    """构造透视变换的源梯形（图像中有意义的车道区域）。

    返回 4 个点，顺序为：左下、左上、右上、右下。

    默认参数对应「车前方一块梯形地面」，覆盖近处宽、远处窄。

    ``bottom_ratio`` 默认为 1.0（贴到图像最底行），这一点很重要：
    车道中心与横向偏差都以「离车最近处」为基准计算，如果梯形底边抬高到
    例如 0.95h，那么算出来的其实是更远处（约 3 m 前）的横向偏差，会引入
    与距离成正比的系统性误差（实测偏右 100 px 的场景误差达 35 px）。
    """
    bottom_v = (height - 1) * bottom_ratio
    top_v = height * top_ratio
    bottom_half = width * bottom_half_width_ratio
    top_half = width * top_half_width_ratio
    return np.array(
        [
            [width / 2.0 - bottom_half, bottom_v],
            [width / 2.0 - top_half, top_v],
            [width / 2.0 + top_half, top_v],
            [width / 2.0 + bottom_half, bottom_v],
        ],
        dtype=np.float64,
    )


def birdseye_destination_points(
    width: int,
    height: int,
    margin_ratio: float = 0.20,
    vertical_margin_ratio: float = 0.05,
) -> np.ndarray:
    """透视变换的目标矩形（鸟瞰图）。

    返回 4 个点，顺序与 ``trapezoid_source_points`` 一致：
    左下、左上、右上、右下。

    垂直方向必须留出边距（``vertical_margin_ratio``）：源梯形的底边取在
    图像下方约 0.95h 处，透视外推后，该行以下的近处车道像素会映射到
    目标图 h 行之外。若目标矩形下边直接顶到 ``height-1``，这些像素全部落在
    图像外被丢弃，导致近处车道（尤其是偏在一侧的）整条消失。
    """
    margin = width * margin_ratio
    v_margin = height * vertical_margin_ratio
    v_bottom = height - 1.0 - v_margin
    v_top = v_margin
    return np.array(
        [
            [margin, v_bottom],
            [margin, v_top],
            [width - 1.0 - margin, v_top],
            [width - 1.0 - margin, v_bottom],
        ],
        dtype=np.float64,
    )


# ---------------------------------------------------------------------------
# 相机标定模型：用真实针孔投影取代手工梯形比例
# ---------------------------------------------------------------------------


@dataclass
class CameraModel:
    """前视针孔相机模型（俯仰角以**光轴向下为正**）。

    为什么需要它：``trapezoid_source_points`` 用「图像宽高的百分比」圈定车道
    区域，但百分比与地面距离没有确定关系——同样的 0.42 在 800x600 / FOV90 /
    俯仰 8° 下只覆盖车前 1.7~6.4 m，横向仅 ±2 m，一旦车辆偏移半米，车道边界
    就被裁到梯形外；同时鸟瞰图的「米/像素」也是臆测的，算出的横向偏差没有
    物理意义。

    有了相机模型，地面点 (d, y) 与像素 (u, v) 可以互相解析换算，梯形由此
    反推得到，鸟瞰图才有确定的标度。
    """

    width: int = 800
    height: int = 600
    hfov_deg: float = 90.0
    mount_height_m: float = 1.70
    pitch_deg: float = 8.0

    @property
    def focal_px(self) -> float:
        """水平视场角对应的焦距（像素）。正方形像素假设下 fy = fx。"""
        return (self.width / 2.0) / math.tan(math.radians(self.hfov_deg) / 2.0)

    def horizon_row(self) -> float:
        """地平线所在像素行（该行对应地面无穷远）。"""
        return self.height / 2.0 - self.focal_px * math.tan(
            math.radians(self.pitch_deg)
        )

    def project(self, d: float, y: float) -> Tuple[float, float]:
        """地面坐标 -> 像素。``d`` 为前向距离（米），``y`` 为横向（米，右正）。"""
        if d <= 0:
            raise ValueError("前向距离 d 必须为正")
        alpha = math.atan2(self.mount_height_m, d)  # 视线俯角（相对水平面）
        theta = alpha - math.radians(self.pitch_deg)  # 相对光轴，向下为正
        z = math.hypot(d, self.mount_height_m) * math.cos(theta)
        f = self.focal_px
        u = self.width / 2.0 + f * y / z
        v = self.height / 2.0 + f * math.tan(theta)
        return u, v

    def unproject(self, u: float, v: float) -> Tuple[float, float]:
        """像素 -> 地面坐标。落在地平线以上时返回 (inf, inf)。"""
        f = self.focal_px
        theta = math.atan2(v - self.height / 2.0, f)
        alpha = theta + math.radians(self.pitch_deg)
        if alpha <= 1e-6:
            return float("inf"), float("inf")
        d = self.mount_height_m / math.tan(alpha)
        z = math.hypot(d, self.mount_height_m) * math.cos(theta)
        y = (u - self.width / 2.0) * z / f
        return d, y


@dataclass
class GroundRegion:
    """鸟瞰图所对应的地面矩形及其标度。"""

    homography: np.ndarray
    d_near: float
    d_far: float
    half_width: float
    out_size: Tuple[int, int]
    mpp_x: float  # 米/像素（横向）
    mpp_y: float  # 米/像素（纵向，即前向距离）
    source_points: np.ndarray

    def row_of_distance(self, d: float) -> float:
        """前向距离 d 对应的鸟瞰图行号（d_far 在上、d_near 在下）。"""
        return (self.d_far - d) / self.mpp_y

    def distance_of_row(self, v: float) -> float:
        return self.d_far - v * self.mpp_y


def ground_region_homography(
    cam: CameraModel,
    d_near: float = 5.0,
    d_far: float = 20.0,
    half_width: float = 4.0,
    out_size: Optional[Tuple[int, int]] = None,
) -> GroundRegion:
    """由相机模型算出「地面矩形 -> 鸟瞰图整幅」的单应矩阵。

    地面矩形取 ``d ∈ [d_near, d_far]``、``y ∈ [-half_width, +half_width]``，
    四角投影到像素后必须仍落在图像内，否则鸟瞰图边缘会缺数据。
    """
    if not (0.0 < d_near < d_far):
        raise ValueError("需满足 0 < d_near < d_far")
    if half_width <= 0:
        raise ValueError("half_width 必须为正")

    out_w, out_h = out_size or (cam.width, cam.height)
    src = np.array(
        [
            cam.project(d_near, -half_width),
            cam.project(d_far, -half_width),
            cam.project(d_far, +half_width),
            cam.project(d_near, +half_width),
        ],
        dtype=np.float64,
    )
    dst = np.array(
        [
            [0.0, out_h - 1.0],
            [0.0, 0.0],
            [out_w - 1.0, 0.0],
            [out_w - 1.0, out_h - 1.0],
        ],
        dtype=np.float64,
    )

    eps = 0.5
    if (
        src[:, 0].min() < -eps
        or src[:, 0].max() > cam.width - 1 + eps
        or src[:, 1].min() < -eps
        or src[:, 1].max() > cam.height - 1 + eps
    ):
        raise ValueError(
            "地面矩形四角超出图像范围，请减小 d_far 或 half_width："
            " corners=%s" % np.round(src, 1).tolist()
        )

    homography = compute_homography(src, dst)
    return GroundRegion(
        homography=homography,
        d_near=d_near,
        d_far=d_far,
        half_width=half_width,
        out_size=(int(out_w), int(out_h)),
        mpp_x=2.0 * half_width / float(out_w),
        mpp_y=(d_far - d_near) / float(out_h),
        source_points=src,
    )


# ---------------------------------------------------------------------------
# 车道像素 -> 多项式
# ---------------------------------------------------------------------------


@dataclass
class LaneFitConfig:
    """车道线检测参数（与 ROS 参数一一对应）。"""

    # 二值化
    #
    # sobel_min 是**保底阈值**，实际阈值由 directional_gradient_mask 里的
    # 分位数自适应决定：取 max(sobel_min, 该图像的 98 分位梯度)。
    # 这样真实图像（路面纹理强）与合成图像（背景几乎无梯度）都能得到
    # 「只有少数强边缘入选」的效果，无需为不同图像分别调参。
    sobel_min: float = 100.0
    sobel_max: float = 900.0
    sobel_auto_percentile: float = 98.5
    angle_max_deg: float = 45.0
    use_color: bool = True
    # 白漆的「三通道最小值」阈值。实测 CARLA 前视图像里，白线 minc≈170~189，
    # 而沥青路面 minc≈128~135，取 160 能干净分开；原先的 190 会把真实白线
    # 整条漏掉（在合成图上白漆是 255，所以一直没暴露）。
    white_min: int = 160

    # 相机标定（提供 CameraModel 时生效，取代下面按图像比例写的梯形参数）
    #
    # d_near / d_far 是鸟瞰图覆盖的前向距离区间，bev_half_width_m 是横向半宽。
    # 取 5~20 m 是有依据的：800x600 / FOV90 / 高 1.7 m / 俯仰 8° 时，地平线在
    # 第 244 行，车前 20 m 对应第 278 行、5 m 对应第 376 行——整条有效地面只占
    # 约 98 行，所以远端不能再远（竖向分辨率会退化到 1 m/px 以上），近端也不能
    # 再近（横向会超出视场，四角落到图像外）。
    d_near_m: float = 3.5
    d_far_m: float = 15.0
    bev_half_width_m: float = 3.2

    # 统计直方图用的距离段（米）。跳过最远端（噪声大）与最近端（车身/阴影）
    hist_band_m: Tuple[float, float] = (5.0, 13.0)

    # 车道宽先验：鸟瞰图是等比的地面正视图，左右边界一旦配对正确，
    # 两者的列距恒等于车道宽，这是全局最强的约束。
    lane_width_m: float = 3.5
    lane_width_tol_m: float = 0.6

    # 当前帧检出失败时，最多沿用上一帧拟合多少帧（20 Hz 下 10 帧 = 0.5 s）
    max_stale_frames: int = 10

    # 稳健化重拟合的取点半宽（像素）。车道线本身宽约 15 px，取 18 px 只保留
    # 曲线附近的像素，把虚线缺口边缘、路面亮斑挡在外面。
    refine_tol_px: float = 18.0

    # 直方图两侧各裁掉多少米。透视梯形的左右边界正好压在路缘/人行道上，
    # 那里有连续的亮边，会在鸟瞰图最左/最右列堆出一个假峰（实测高度可达
    # 400/418，比真车道线还高），把配对带偏。
    hist_trim_m: float = 0.3

    # 透视变换
    # 注意：梯形必须足够宽，否则车道一旦偏移出梯形就会被整体裁掉
    # （踩过的坑：早期参数偏窄，车辆偏离车道中心时右车道完全消失）。
    # bottom_ratio 必须为 1.0，使「最近处」与图像底行对齐，横向偏差才对应车头位置。
    bottom_ratio: float = 1.0
    top_ratio: float = 0.58
    bottom_half_width_ratio: float = 0.42
    top_half_width_ratio: float = 0.28
    margin_ratio: float = 0.20
    vertical_margin_ratio: float = 0.02

    # 滑动窗口
    n_windows: int = 9
    window_margin_px: int = 100
    min_pixels_to_recenter: int = 40
    min_lane_pixels: int = 150

    # 连通域几何约束：车道线在鸟瞰图中是**细长**结构。
    #
    # 主判据用「填充率」= 像素数 / 包围盒面积，而不是包围盒宽度。
    # 原因：车道线在鸟瞰图中常常是**斜的**（车辆偏离车道中心、或相机有偏航
    # 时），此时包围盒会很宽——实测一条完全正常的倾斜车道线包围盒宽度达 114 px，
    # 但填充率只有 0.27。若按「包围盒宽度 < 90 px」判断，这条真实车道线会被
    # 误杀（踩过的坑：车辆偏右的场景里整条左车道丢失，横向偏差误差达 25 px）。
    #
    # 填充率对竖直与倾斜车道线都成立：车道线很细，填充率必然低；
    # 而被误阈值化的路面是面状结构，包围盒内几乎被填满，填充率很高。
    max_component_fill_ratio: float = 0.45  # 填充率上限（主判据）
    max_component_width_px: int = 260  # 包围盒宽度上限（仅挡极端情况）
    max_component_width_ratio: float = 0.45  # 或占图像宽度的比例上限
    max_component_area_ratio: float = 0.18  # 面积占全图比例上限

    # 物理标定（用于把像素换算成米）
    meters_per_pixel_y: float = 0.05
    meters_per_pixel_x: float = 0.05

    # 多项式拟合
    poly_degree: int = 2


@dataclass
class LaneDetectionResult:
    """一帧图像的车道线检测结果。"""

    left_fit: Optional[np.ndarray] = None  # v -> u 的多项式系数（次数由高到低）
    right_fit: Optional[np.ndarray] = None
    left_base: Optional[float] = None  # 底部起点像素 u
    right_base: Optional[float] = None
    left_pixels: int = 0  # 参与拟合的像素数
    right_pixels: int = 0
    confidence: float = 0.0  # 0~1，两侧车道线与像素量的综合置信度
    lane_center_px: Optional[float] = None  # 车辆中心相对车道中心的像素偏差
    lateral_offset_m: Optional[float] = None  # 横向偏差（米），正 = 偏右
    heading_error_rad: Optional[float] = None  # 航向偏差（弧度），正 = 车头偏右
    curvature_m: Optional[float] = None  # 车道曲率半径（米）
    v_eval: Optional[float] = None  # 几何量实际求值的行（便于排查）
    lane_width_m: Optional[float] = None  # 本帧实测车道宽（米），用于自检
    distance_m: Optional[float] = None  # 求值行对应的前向距离（米）
    stale: bool = False  # 本帧几何量沿用上一帧（当前帧检出失败）
    stale_frames: int = 0  # 连续沿用帧数

    @property
    def detected(self) -> bool:
        return self.left_fit is not None and self.right_fit is not None


def find_lane_bases(
    histogram: np.ndarray,
    min_relative_peak: float = 0.25,
    min_separation_px: float = 40.0,
):
    """由鸟瞰图底部直方图找出左右车道线的起点列。

    步骤：
      1. 取全局最强峰作为第一条车道线的起点
      2. 在距它 ``min_separation_px`` 之外找次强峰作为另一条
      3. **按列坐标大小分左右**：列号小的那侧是左车道线

    第 3 步为什么不能按「图像中线」判断（曾经的实现）：
    那同样隐含「车辆居于两条车道线正中」的假设。车辆偏在车道一侧时，
    两条车道线的峰可能都落在中线同一侧，中线判据会把两条都归为同一侧，
    另一侧恒为空，整条车道被丢弃。

    按列坐标大小分左右则与车辆位置无关：左车道线在图像中必然位于右车道线
    的左边，这一条几何事实永远成立。

    返回 (left_base, right_base)，任一侧无有效峰时为 None。
    """
    hist = np.asarray(histogram, dtype=np.float64).ravel()
    n = hist.size
    if n == 0 or hist.max() <= 0:
        return None, None

    # 显著性判据直接用**原始峰值高度**相对全局峰的比例。
    #
    # 这里刻意不做平滑：车道线的直方图峰值往往是「尖而孤立」的少数几列，
    # 任何邻域卷积都会把这类尖峰削去大半（三点核下 61 会变成 30.5），
    # 从而把真实存在的车道线误判为「该侧无车道线」。这是踩过的坑。
    threshold = min_relative_peak * float(hist.max())
    separation = int(min_separation_px)

    peak1 = int(np.argmax(hist))

    # 在 peak1 左右两侧各找次强峰
    candidates = []
    left_seg = hist[: max(0, peak1 - separation)]
    if left_seg.size:
        idx = int(np.argmax(left_seg))
        if left_seg[idx] >= threshold:
            candidates.append(idx)
    right_start = min(n, peak1 + separation + 1)
    right_seg = hist[right_start:]
    if right_seg.size:
        idx = int(np.argmax(right_seg)) + right_start
        if hist[idx] >= threshold:
            candidates.append(idx)

    # 没有次强峰 -> 只找到一条车道线，无从判断左右，返回单侧
    if not candidates:
        return peak1, None

    peak2 = max(candidates, key=lambda i: hist[i])
    left_base = min(peak1, peak2)
    right_base = max(peak1, peak2)
    return left_base, right_base


def find_lane_pair(
    histogram: np.ndarray,
    expected_px: float,
    tolerance_px: float,
    center_px: Optional[float] = None,
    center_weight: float = 0.25,
    min_relative_peak: float = 0.12,
) -> Optional[Tuple[float, float]]:
    """按「车道宽先验」直接搜索左右车道线的一对起点列。

    与 ``find_lane_bases`` 的区别：那个函数只要求两个峰「分开 40 px 以上」，
    在多车道画面里会把相邻车道的线、甚至双黄线的两条线配成左右边界，于是
    车道宽度报出 0.9 m 或 8.8 m 这种明显不合理的值，横向偏差随之崩到几十米。

    鸟瞰图是地面正视图，只要透视标定正确，**左右边界的列距恒等于车道宽**。
    因此这里不再做「找两个峰」，而是穷举所有满足 ``|Δ - expected| <= tolerance``
    的列对，取两侧较弱峰也足够高、且配对中心尽量靠近车辆中心线的那一对。

    返回 (left_base, right_base)，找不到时返回 None。
    """
    hist = np.asarray(histogram, dtype=np.float64).ravel()
    n = hist.size
    if n == 0 or hist.max() <= 0:
        return None
    if expected_px <= 0 or tolerance_px <= 0:
        raise ValueError("expected_px 与 tolerance_px 必须为正")

    # 三点平滑。这里与 find_lane_bases 刻意不同：宽度先验已经足够强，
    # 平滑削掉的那些尖峰不再致命，反而能压掉「单列噪点冒充车道线」。
    smoothed = np.convolve(hist, np.array([0.25, 0.5, 0.25]), mode="same")

    delta_lo = max(1, int(round(expected_px - tolerance_px)))
    delta_hi = min(n - 1, int(round(expected_px + tolerance_px)))
    if delta_hi < delta_lo:
        return None

    threshold = min_relative_peak * float(smoothed.max())
    best_score = -1.0
    best_pair: Optional[Tuple[float, float]] = None
    columns = np.arange(n, dtype=np.float64)
    scale = 0.5 * expected_px

    for delta in range(delta_lo, delta_hi + 1):
        # 该间距下所有 (左列, 右列) 组合的得分 = 两侧较弱峰的高度。
        # 注意必须在**同一间距的所有候选列**里挑最优，只取全局最强的那一列
        # 会漏掉真正靠近车辆中心线的一对（曾经就是这么错的）。
        pair_score = np.minimum(smoothed[:-delta], smoothed[delta:])
        if pair_score.size == 0:
            continue

        # 间距越接近先验越好：偏离满量程时打 0.5 折
        score = pair_score * (1.0 - 0.5 * abs(delta - expected_px) / tolerance_px)

        if center_px is not None:
            center = columns[: pair_score.size] + delta / 2.0
            # 以半个车道宽为尺度做软惩罚，车辆偏到相邻车道才会明显吃亏
            score = score / (1.0 + center_weight * ((center - center_px) / scale) ** 2)

        idx = int(np.argmax(score))
        if pair_score[idx] < threshold:
            continue
        if float(score[idx]) > best_score:
            best_score = float(score[idx])
            best_pair = (float(idx), float(idx + delta))

    return best_pair


def fit_lanes_by_components(
    mask: np.ndarray,
    left_base: Optional[float],
    right_base: Optional[float],
    cfg: LaneFitConfig,
) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], int, int]:
    """用连通域分组拟合车道线（本项目主用）。

    思路：鸟瞰图中每条车道线是一个（细长的）连通域，因此先把掩码做连通域标记，
    再按「基点列落在哪个连通域里」把左右两侧分别取出来，各自拟合多项式。

    为什么不用滑动窗口（曾经的实现）：
    滑动窗口要求「窗口宽度大于单条车道线宽度，同时小于两条车道线的间距」。
    这个条件很容易被违反——窗口取得稍宽（例如 ±100 px）就会同时框住左右两条线，
    取均值后两条线都拟合到画面正中，输出完全错误却不报错。连通域分组不存在
    这个尺度矛盾，且天然把两条线分开。

    返回 (left_coeffs, right_coeffs, left_count, right_count)。
    """
    h, w = mask.shape
    labels, ids = _label_components(mask)
    if ids.size == 0:
        return None, None, 0, 0

    total_px = float(h * w)
    max_width = min(
        cfg.max_component_width_px,
        cfg.max_component_width_ratio * w,
    )
    max_area = cfg.max_component_area_ratio * total_px

    # 统计每个连通域，并施加**细长结构**约束。
    # 主判据是填充率（像素数 / 包围盒面积），它能同时适配竖直与倾斜的车道线；
    # 包围盒宽度只作为极端情况的兜底（见 DetectorConfig 里的说明）。
    stats = {}
    for lab in ids:
        ys, xs = np.nonzero(labels == lab)
        count = int(ys.size)
        if count < cfg.min_lane_pixels:
            continue
        width = float(xs.max() - xs.min() + 1)
        height_extent = float(ys.max() - ys.min() + 1)
        if width > max_width:
            continue
        if count > max_area:
            continue
        fill_ratio = count / max(1.0, width * height_extent)
        if fill_ratio > cfg.max_component_fill_ratio:
            continue
        # 竖直方向要足够长，排除零碎的小块
        if height_extent < 0.2 * h:
            continue
        stats[int(lab)] = {
            "count": count,
            "u_min": float(xs.min()),
            "u_max": float(xs.max()),
            "u_mean": float(xs.mean()),
        }

    result = {}
    counts = {}
    chosen_labels = {}
    for side, base in (("left", left_base), ("right", right_base)):
        if base is None:
            result[side] = None
            counts[side] = 0
            continue

        # 选基点附近、且面积足够大的连通域作为该侧车道线
        best_label = None
        best_score = None
        for lab, st in stats.items():
            if st["count"] < cfg.min_lane_pixels:
                continue
            # 基点应落在该连通域的列范围内（允许少量外扩）
            slack = 0.15 * (st["u_max"] - st["u_min"] + 1.0) + 5.0
            if not (st["u_min"] - slack <= base <= st["u_max"] + slack):
                continue
            score = abs(st["u_mean"] - base) - st["count"] * 1e-6
            if best_score is None or score < best_score:
                best_score = score
                best_label = lab

        chosen_labels[side] = best_label

    # 左右分到同一个连通域 => 两条车道线被误连成一块（细窄车道、脏像素粘连都会
    # 导致）。此时按「两基点之间的中线」把该连通域的像素重新切成左右两半再拟合。
    merged = (
        chosen_labels.get("left") is not None
        and chosen_labels.get("left") == chosen_labels.get("right")
        and left_base is not None
        and right_base is not None
    )
    if merged:
        lab = chosen_labels["left"]
        ys, xs = np.nonzero(labels == lab)
        boundary = 0.5 * (left_base + right_base)
        for side, mask_side in (("left", xs < boundary), ("right", xs >= boundary)):
            ys_side = ys[mask_side]
            xs_side = xs[mask_side]
            counts[side] = int(xs_side.size)
            if (
                xs_side.size < cfg.min_lane_pixels
                or np.unique(ys_side).size <= cfg.poly_degree
            ):
                result[side] = None
                continue
            result[side] = np.polyfit(
                ys_side.astype(np.float64), xs_side.astype(np.float64), cfg.poly_degree
            )
        return result["left"], result["right"], counts["left"], counts["right"]

    for side, base in (("left", left_base), ("right", right_base)):
        best_label = chosen_labels.get(side)
        if best_label is None:
            result[side] = None
            counts[side] = 0
            continue

        ys, xs = np.nonzero(labels == best_label)
        counts[side] = int(xs.size)
        if xs.size < cfg.min_lane_pixels or np.unique(ys).size <= cfg.poly_degree:
            result[side] = None
            continue
        result[side] = np.polyfit(
            ys.astype(np.float64), xs.astype(np.float64), cfg.poly_degree
        )

    return result["left"], result["right"], counts["left"], counts["right"]


def _label_components(mask: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """对布尔掩码做 8-邻域连通域标记，返回 (labels, 非零标签)。

    纯 numpy + BFS 实现；图像较大时可接受（鸟瞰图分辨率通常不超过 800x600）。
    """
    m = np.asarray(mask, dtype=bool)
    h, w = m.shape
    labels = np.zeros((h, w), dtype=np.int32)
    next_label = 0

    # 8 邻域偏移
    neighbors = [
        (-1, -1), (-1, 0), (-1, 1),
        (0, -1), (0, 1),
        (1, -1), (1, 0), (1, 1),
    ]

    stack = []
    for start_v in range(h):
        row = m[start_v]
        for start_u in range(w):
            if not row[start_u] or labels[start_v, start_u] != 0:
                continue
            next_label += 1
            labels[start_v, start_u] = next_label
            stack.append((start_v, start_u))
            while stack:
                v, u = stack.pop()
                for dv, du in neighbors:
                    nv, nu = v + dv, u + du
                    if 0 <= nv < h and 0 <= nu < w:
                        if m[nv, nu] and labels[nv, nu] == 0:
                            labels[nv, nu] = next_label
                            stack.append((nv, nu))

    ids = np.arange(1, next_label + 1, dtype=np.int32)
    return labels, ids


def sliding_window_fit(
    mask: np.ndarray,
    left_base: Optional[float],
    right_base: Optional[float],
    cfg: LaneFitConfig,
) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], int, int]:
    """滑动窗口搜索并按列拟合多项式。

    参数
    ----
    mask : 鸟瞰图二值掩码（True 表示车道像素）
    left_base, right_base : 起始列（来自直方图）；为 None 表示该侧不搜索

    返回
    ----
    (left_coeffs, right_coeffs, left_count, right_count)
    多项式以 v（行）为自变量拟合 u（列），系数按次数从高到低（np.polyfit 约定）。
    """
    h, w = mask.shape
    nonzero = np.nonzero(mask)
    nz_v, nz_u = nonzero[0], nonzero[1]

    result = {}
    counts = {}
    for side, base in (("left", left_base), ("right", right_base)):
        if base is None:
            result[side] = None
            counts[side] = 0
            continue

        current = float(base)
        win_h = h / cfg.n_windows
        collected_u = []
        collected_v = []

        for win in range(cfg.n_windows):
            v_lo = h - (win + 1) * win_h
            v_hi = h - win * win_h
            u_lo = current - cfg.window_margin_px
            u_hi = current + cfg.window_margin_px

            inside = (nz_v >= v_lo) & (nz_v < v_hi) & (nz_u >= u_lo) & (nz_u < u_hi)
            if int(inside.sum()) >= cfg.min_pixels_to_recenter:
                current = float(nz_u[inside].mean())
            collected_u.append(nz_u[inside])
            collected_v.append(nz_v[inside])

        all_u = np.concatenate(collected_u) if collected_u else np.array([], dtype=int)
        all_v = np.concatenate(collected_v) if collected_v else np.array([], dtype=int)
        counts[side] = int(all_u.size)

        if all_u.size < cfg.min_lane_pixels or np.unique(all_v).size <= cfg.poly_degree:
            result[side] = None
            continue

        coeffs = np.polyfit(all_v.astype(np.float64), all_u.astype(np.float64), cfg.poly_degree)
        result[side] = coeffs

    return result["left"], result["right"], counts["left"], counts["right"]


def refine_poly_fit(
    mask: np.ndarray,
    fit: np.ndarray,
    tol_px: float = 18.0,
    degree: int = 2,
    iterations: int = 2,
) -> Tuple[Optional[np.ndarray], int]:
    """以初拟合为中心收窄取点范围，剔除离群像素后重拟合。

    为什么需要：滑动窗口按固定列宽（``window_margin_px``）收集像素，
    虚线缺口的边缘、路面上零散的亮斑一旦落入窗口就会被算进同一个多项式，
    把整条线的位置拉动十几个像素。实测表现为横向偏差每隔两三秒跳 0.1 m，
    而真值几乎不动。

    做法是经典的「拟合—剔点—再拟合」：只保留距离当前曲线 ``tol_px`` 以内的
    像素，重新拟合，重复两次。车道线本身是连续细线，正常像素的残差只有
    几个像素，被剔掉的必然是杂散点。

    返回 (新系数, 参与拟合的像素数)；像素不足时返回 (None, n)。
    """
    if fit is None:
        return None, 0
    current = np.asarray(fit, dtype=np.float64)
    ys, xs = np.nonzero(mask)
    if ys.size == 0:
        return None, 0

    kept = 0
    for _ in range(max(1, iterations)):
        residual = np.abs(xs - np.polyval(current, ys))
        inside = residual <= tol_px
        kept = int(inside.sum())
        if kept <= degree + 1 or np.unique(ys[inside]).size <= degree:
            return None, kept
        current = np.polyfit(
            ys[inside].astype(np.float64), xs[inside].astype(np.float64), degree
        )
    return current, kept


def evaluate_poly(coeffs: np.ndarray, v: np.ndarray) -> np.ndarray:
    """按多项式系数求 u。coeffs 按次数从高到低。"""
    return np.polyval(coeffs, v)


def warp_horizontal_scale(h: np.ndarray, width: int, height: int, v: float) -> float:
    """返回「源图列」到「warped 图列」的横向比例。

    透视变换并非等比例：源图一条水平线段映射到 warped 图后长度会变化。
    做标定或与真值比对时必须用这个比例换算，否则会出现看似系统误差的偏差
    （实测在 800x600 默认参数下约为 0.71）。

    做法：取同一源行上的两个点，分别映射到 warped 图，用列差求比例。
    """
    src_y = float(np.clip(v, 0.0, height - 1.0))
    p_a = h @ np.array([0.0, src_y, 1.0])
    p_b = h @ np.array([float(width - 1), src_y, 1.0])
    warped_span = abs(p_b[0] / p_b[2] - p_a[0] / p_a[2])
    return warped_span / float(width - 1)


def warp_vertical_row(h: np.ndarray, width: int, height: int, src_v: float) -> float:
    """返回源图行 ``src_v`` 在 warped 图中的对应行号（取图像中线处的映射）。"""
    p = h @ np.array([width / 2.0, float(src_v), 1.0])
    return float(p[1] / p[2])


def lane_geometry_with_width(
    left_fit: np.ndarray,
    right_fit: np.ndarray,
    width: int,
    height: int,
    cfg: LaneFitConfig,
    v_eval: Optional[float] = None,
) -> Tuple[float, float, float, float]:
    """计算 (车道中心像素列, 横向偏差米, 航向偏差弧度, 曲率米)。

    约定：
      * 车辆在鸟瞰图中的横向位置取图像水平中心 ``width / 2``
      * 横向偏差 = (车辆列 - 车道中心列) * meters_per_pixel_x
        正值表示**车辆偏在车道中心右侧**
      * 航向偏差：由车道中心线斜率 du/dv 换算成角度，
        正值表示**车头相对车道方向偏右**

    ``v_eval`` 指定求值行。**务必传入有车道像素的行**（调用方从掩码统计得到），
    否则多项式会在无数据区域外推，给出错误的中心位置。
    """
    if v_eval is None:
        v_eval = float(height - 1)

    u_left = float(np.polyval(left_fit, v_eval))
    u_right = float(np.polyval(right_fit, v_eval))
    lane_center = 0.5 * (u_left + u_right)

    vehicle_u = width / 2.0
    lateral_m = (vehicle_u - lane_center) * cfg.meters_per_pixel_x

    # 车道中心线（左右平均）在求值行的斜率
    center_fit = 0.5 * (np.asarray(left_fit) + np.asarray(right_fit))
    dv = 1.0
    u_at = float(np.polyval(center_fit, v_eval))
    u_prev = float(np.polyval(center_fit, v_eval - dv))
    du_dv = (u_at - u_prev) / dv

    # 航向偏差的符号推导（这里踩过坑，实测与真值相关系数为 -0.97）：
    #
    # 设车头相对车道方向偏右 psi，则在车体系里车道中心线满足 y = -d*tan(psi)。
    # 鸟瞰图中 d = d_far - v*my、y = (u-u0)*mx，代入得
    #     u = u0 + (tan(psi)*my/mx) * v + const
    # 即 du/dv = tan(psi) * my/mx，所以 psi = atan(du/dv * mx / my)。
    # 正号：psi > 0 表示车头偏右，与控制器里 steering = -Kpsi*e_psi 的约定一致。
    heading = math.atan2(du_dv * cfg.meters_per_pixel_x, cfg.meters_per_pixel_y)

    curvature = estimate_curvature_m(center_fit, v_eval, cfg)
    return lane_center, lateral_m, heading, curvature


def estimate_curvature_m(
    fit: np.ndarray, v_eval: float, cfg: LaneFitConfig
) -> float:
    """用多项式的一阶、二阶导数估计曲率半径（米）。

    对 u = f(v)，物理坐标 x = u * mx，y = v * my（y 为车前方距离），
    则曲率半径
        R = (1 + (dx/dy)^2)^(3/2) / |d2x/dy2|
    其中 dx/dy = (du/dv) * mx / my，d2x/dy2 = (d2u/dv2) * mx / my^2。
    """
    coeffs = np.asarray(fit, dtype=np.float64)
    # 一阶、二阶导数多项式
    d1 = np.polyder(coeffs, 1)
    d2 = np.polyder(coeffs, 2)
    du_dv = float(np.polyval(d1, v_eval)) if d1.size else 0.0
    d2u_dv2 = float(np.polyval(d2, v_eval)) if d2.size else 0.0

    mx, my = cfg.meters_per_pixel_x, cfg.meters_per_pixel_y
    dx_dy = du_dv * mx / my
    d2x_dy2 = d2u_dv2 * mx / (my * my)

    if abs(d2x_dy2) < 1e-9:
        return float("inf")
    return float((1.0 + dx_dy**2) ** 1.5 / abs(d2x_dy2))


def compute_confidence(
    left_pixels: int, right_pixels: int, min_lane_pixels: int
) -> float:
    """由两侧像素量给出 0~1 的置信度。

    每一侧按其像素量相对阈值计 0~0.5 分：达到阈值即得满 0.5，
    因此两侧都达到阈值时为 1.0，只检出一侧时约为 0.5。
    """
    if min_lane_pixels <= 0:
        raise ValueError("min_lane_pixels 必须为正")

    def side_score(n: int) -> float:
        if n <= 0:
            return 0.0
        return 0.5 * min(1.0, n / float(min_lane_pixels))

    score = side_score(left_pixels) + side_score(right_pixels)
    return float(max(0.0, min(1.0, score)))


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------


class LaneDetector:
    """把上面的步骤串起来的有状态检测器。

    需要持有状态的原因：帧间可以用上一帧的拟合结果做搜索起点，提升稳定性。
    单帧调用 ``detect`` 也能工作（退化到直方图找起点）。

    两种工作模式：

    * **标定模式**（传入 ``camera``）：按针孔投影把地面矩形映射成鸟瞰图，
      米/像素由相机参数决定，左右边界用车道宽先验配对。真实相机图像必须用
      这个模式，否则算出来的横向偏差没有物理意义。
    * **比例模式**（不传 ``camera``）：沿用手工梯形比例，只适合合成图像与
      单元测试。
    """

    def __init__(
        self,
        cfg: Optional[LaneFitConfig] = None,
        image_size: Tuple[int, int] = None,
        camera: Optional[CameraModel] = None,
    ):
        self.cfg = cfg or LaneFitConfig()
        self.image_size = image_size  # (width, height)
        self.camera = camera
        self._h: Optional[np.ndarray] = None
        self._region: Optional[GroundRegion] = None
        self._metric_cfg: Optional[LaneFitConfig] = None
        self._prev_left: Optional[np.ndarray] = None
        self._prev_right: Optional[np.ndarray] = None
        self._stale_frames = 0

    @property
    def region(self) -> Optional[GroundRegion]:
        return self._region

    def reset(self):
        """清空帧间状态（换场景、重连 CARLA 时调用）。"""
        self._prev_left = None
        self._prev_right = None
        self._stale_frames = 0

    def _ensure_homography(self, width: int, height: int):
        if (
            self._h is not None
            and self.image_size == (width, height)
            and (self.camera is None or self._region is not None)
        ):
            return
        self.image_size = (width, height)

        if self.camera is not None:
            cam = replace(self.camera, width=width, height=height)
            self._region = ground_region_homography(
                cam,
                d_near=self.cfg.d_near_m,
                d_far=self.cfg.d_far_m,
                half_width=self.cfg.bev_half_width_m,
                out_size=(width, height),
            )
            self._h = self._region.homography
            self._metric_cfg = replace(
                self.cfg,
                meters_per_pixel_x=self._region.mpp_x,
                meters_per_pixel_y=self._region.mpp_y,
            )
            self._prev_left = self._prev_right = None
            self._stale_frames = 0
            return

        src = trapezoid_source_points(
            width,
            height,
            bottom_ratio=self.cfg.bottom_ratio,
            top_ratio=self.cfg.top_ratio,
            bottom_half_width_ratio=self.cfg.bottom_half_width_ratio,
            top_half_width_ratio=self.cfg.top_half_width_ratio,
        )
        dst = birdseye_destination_points(
            width,
            height,
            self.cfg.margin_ratio,
            self.cfg.vertical_margin_ratio,
        )
        self._h = compute_homography(src, dst)
        self._region = None
        self._metric_cfg = None

    def binary_lane_mask(self, image: np.ndarray) -> np.ndarray:
        """彩色/灰度图 -> 车道像素二值掩码（原图尺寸）。"""
        gray = to_grayscale(image)
        mask = directional_gradient_mask(
            gray,
            sobel_min=self.cfg.sobel_min,
            sobel_max=self.cfg.sobel_max,
            angle_max_deg=self.cfg.angle_max_deg,
            auto_percentile=self.cfg.sobel_auto_percentile,
        )
        if self.cfg.use_color and image.ndim == 3:
            mask = mask | color_lane_mask(image, white_min=self.cfg.white_min)
        return mask

    def warped_mask(self, image: np.ndarray) -> np.ndarray:
        """直接得到鸟瞰图二值掩码。"""
        h_img, w_img = image.shape[0], image.shape[1]
        self._ensure_homography(w_img, h_img)
        mask = self.binary_lane_mask(image)
        warped = warp_perspective(
            mask.astype(np.uint8) * 255, self._h, (w_img, h_img)
        )
        return warped > 0

    def _pair_is_plausible(
        self, left_fit, right_fit, v_near: float, v_far: float, width: int
    ) -> bool:
        """用几何先验否决明显错误的配对。

        三条硬约束，任何一条不满足就判定本帧无效：

        1. 求值行处的车道宽必须落在 ``lane_width_m ± 1.5 * tol`` 内；
        2. 左右线在求值行都不能跑出图像；
        3. 在统计带内左右线不能交叉（交叉说明把同一条线当成了两条）。
        """
        if left_fit is None or right_fit is None:
            return False
        mpp_x = self._region.mpp_x
        u_l = float(np.polyval(left_fit, v_near))
        u_r = float(np.polyval(right_fit, v_near))
        width_m = (u_r - u_l) * mpp_x
        if abs(width_m - self.cfg.lane_width_m) > 1.5 * self.cfg.lane_width_tol_m:
            return False
        if not (0.0 <= u_l < width and 0.0 <= u_r < width):
            return False
        for v in np.linspace(min(v_far, v_near), max(v_far, v_near), 5):
            if float(np.polyval(left_fit, v)) >= float(np.polyval(right_fit, v)):
                return False
        return True

    def _detect_calibrated(
        self, mask: np.ndarray, w_img: int, h_img: int
    ) -> LaneDetectionResult:
        """标定模式：车道宽先验配对 + 滑动窗口拟合 + 帧间保持。"""
        region = self._region
        cfg_m = self._metric_cfg

        # 直方图只统计中距离段：太远（接近地平线）一个像素跨 1 m 以上，
        # 太近则是车身、阴影与路面污渍。
        d_lo, d_hi = self.cfg.hist_band_m
        d_lo = min(max(d_lo, region.d_near), region.d_far)
        d_hi = min(max(d_hi, region.d_near), region.d_far)
        v_far = int(round(region.row_of_distance(d_hi)))
        v_near = int(round(region.row_of_distance(d_lo)))
        v_far = max(0, min(v_far, h_img - 2))
        v_near = max(v_far + 1, min(v_near, h_img))
        trim = int(round(self.cfg.hist_trim_m / region.mpp_x))
        u_lo, u_hi = trim, max(trim + 1, w_img - trim)
        histogram = mask[v_far:v_near, u_lo:u_hi].sum(axis=0).astype(np.float64)

        expected_px = self.cfg.lane_width_m / region.mpp_x
        tolerance_px = self.cfg.lane_width_tol_m / region.mpp_x
        pair = find_lane_pair(
            histogram, expected_px, tolerance_px, center_px=w_img / 2.0 - u_lo
        )
        if pair is not None:
            pair = (pair[0] + u_lo, pair[1] + u_lo)

        left_fit = right_fit = None
        n_left = n_right = 0
        if pair is not None:
            left_fit, right_fit, n_left, n_right = sliding_window_fit(
                mask, pair[0], pair[1], self.cfg
            )
            # 稳健化：以初拟合为中心剔掉离群像素再拟合一次
            if left_fit is not None:
                left_fit, n_left = refine_poly_fit(
                    mask, left_fit, tol_px=self.cfg.refine_tol_px,
                    degree=self.cfg.poly_degree,
                )
            if right_fit is not None:
                right_fit, n_right = refine_poly_fit(
                    mask, right_fit, tol_px=self.cfg.refine_tol_px,
                    degree=self.cfg.poly_degree,
                )
            v_eval_guess = float(min(h_img - 1, v_near - 1))
            if not self._pair_is_plausible(
                left_fit, right_fit, v_eval_guess, float(v_far), w_img
            ):
                left_fit = right_fit = None
                n_left = n_right = 0

        stale = False
        if left_fit is None or right_fit is None:
            if (
                self._prev_left is not None
                and self._prev_right is not None
                and self._stale_frames < self.cfg.max_stale_frames
            ):
                left_fit, right_fit = self._prev_left, self._prev_right
                self._stale_frames += 1
                stale = True
            else:
                self._stale_frames = 0
        else:
            self._stale_frames = 0
            self._prev_left, self._prev_right = left_fit, right_fit

        v_eval = float(min(h_img - 1, v_near - 1))
        result = LaneDetectionResult(
            left_fit=left_fit,
            right_fit=right_fit,
            left_base=None if pair is None else pair[0],
            right_base=None if pair is None else pair[1],
            left_pixels=n_left,
            right_pixels=n_right,
            confidence=(
                0.0
                if (stale or left_fit is None)
                else compute_confidence(n_left, n_right, self.cfg.min_lane_pixels)
            ),
            v_eval=v_eval,
            distance_m=region.distance_of_row(v_eval),
            stale=stale,
            stale_frames=self._stale_frames,
        )

        if left_fit is not None and right_fit is not None:
            u_l = float(np.polyval(left_fit, v_eval))
            u_r = float(np.polyval(right_fit, v_eval))
            result.lane_width_m = (u_r - u_l) * region.mpp_x
            center, lateral, heading, curvature = lane_geometry_with_width(
                left_fit, right_fit, w_img, h_img, cfg_m, v_eval=v_eval
            )
            result.lane_center_px = float(w_img / 2.0 - center)
            result.lateral_offset_m = lateral
            result.heading_error_rad = heading
            result.curvature_m = curvature

        return result

    def detect(self, image: np.ndarray) -> LaneDetectionResult:
        """对一帧图像做完整车道线检测。"""
        h_img, w_img = image.shape[0], image.shape[1]
        self._ensure_homography(w_img, h_img)

        mask = self.warped_mask(image)
        if self._region is not None:
            return self._detect_calibrated(mask, w_img, h_img)
        # 直方图只统计鸟瞰图底部（离车最近处），那里车道线最宽、最能定出起点
        band_start = int(h_img * 0.7)
        histogram = mask[band_start:, :].sum(axis=0)

        left_base, right_base = find_lane_bases(histogram)

        # 主路径：连通域分组拟合（不依赖窗口宽度，鲁棒）
        left_fit, right_fit, n_left, n_right = fit_lanes_by_components(
            mask, left_base, right_base, self.cfg
        )

        # 兜底：连通域断裂（虚线车道线、局部遮挡）时退回滑动窗口
        if left_fit is None or right_fit is None:
            sw_left, sw_right, sw_nl, sw_nr = sliding_window_fit(
                mask, left_base, right_base, self.cfg
            )
            if left_fit is None and sw_left is not None:
                left_fit, n_left = sw_left, sw_nl
            if right_fit is None and sw_right is not None:
                right_fit, n_right = sw_right, sw_nr

        # 用于几何求值的行：必须有实际车道像素，否则多项式会外推出发散的垃圾值。
        # 踩过的坑：目标矩形底边略微高于图像底行，若固定用 height-1 求值，
        # 会在没有任何数据的行上外推，横向偏差出现十几到几十像素的系统误差。
        rows_with_lane = np.nonzero(mask.any(axis=1))[0]
        if rows_with_lane.size:
            v_eval = float(min(h_img - 1, rows_with_lane.max() - 1))
            v_eval = float(max(v_eval, rows_with_lane.min()))
        else:
            v_eval = float(h_img - 1)

        result = LaneDetectionResult(
            left_fit=left_fit,
            right_fit=right_fit,
            left_base=left_base,
            right_base=right_base,
            left_pixels=n_left,
            right_pixels=n_right,
            confidence=compute_confidence(
                n_left, n_right, self.cfg.min_lane_pixels
            ),
            v_eval=v_eval,
        )

        if left_fit is not None and right_fit is not None:
            center, lateral, heading, curvature = lane_geometry_with_width(
                left_fit, right_fit, w_img, h_img, self.cfg, v_eval=v_eval
            )
            # lane_center_px 语义：车辆列与车道中心列的差（像素，正 = 偏右）
            result.lane_center_px = float(w_img / 2.0 - center)
            result.lateral_offset_m = lateral
            result.heading_error_rad = heading
            result.curvature_m = curvature

        self._prev_left, self._prev_right = left_fit, right_fit
        return result


def make_synthetic_lane_image(
    width: int = 800,
    height: int = 600,
    left_x_bottom: float = 260.0,
    left_x_top: float = 350.0,
    right_x_bottom: float = 560.0,
    right_x_top: float = 460.0,
    lane_half_width_px: float = 6.0,
    background: Tuple[int, int, int] = (70, 70, 70),
    lane_color: Tuple[int, int, int] = (235, 235, 235),
    noise_std: float = 6.0,
    seed: int = 0,
) -> np.ndarray:
    """生成一张合成的车道线图像（BGR），用于单元测试。

    左右车道线都是直线（便于解析验证），背景为灰色并叠加高斯噪声。
    车道线由下往上线性收窄，``*_x_bottom`` 是最下行的列坐标，
    ``*_x_top`` 是最上行的列坐标。

    返回 uint8 的 (height, width, 3) 数组。
    """
    rng = np.random.default_rng(seed)
    img = np.zeros((height, width, 3), dtype=np.float32)
    img[:, :] = np.asarray(background, dtype=np.float32)

    # tt: 0 在图像顶部，1 在图像底部
    v = np.arange(height, dtype=np.float32)
    tt = (height - 1 - v) / max(1.0, height - 1.0)

    for x_bottom, x_top in ((left_x_bottom, left_x_top), (right_x_bottom, right_x_top)):
        centers = x_top + (x_bottom - x_top) * tt  # 每行的车道线中心列
        u = np.arange(width, dtype=np.float32)[None, :]  # (1, width)
        dist = np.abs(u - centers[:, None])  # (height, width)
        line = dist <= lane_half_width_px
        img[line] = np.asarray(lane_color, dtype=np.float32)

    if noise_std > 0:
        img += rng.normal(0.0, noise_std, img.shape).astype(np.float32)
    return np.clip(img, 0, 255).astype(np.uint8)


def render_detection_overlay(
    image: np.ndarray,
    result: LaneDetectionResult,
    cfg: LaneFitConfig,
    color_left: Tuple[int, int, int] = (255, 80, 80),
    color_right: Tuple[int, int, int] = (80, 80, 255),
) -> np.ndarray:
    """把检测结果画回原图（纯 numpy 画线），返回 BGR 叠加图。

    注意：这里画的是**鸟瞰图坐标**下的拟合曲线，为可视化方便，
    调用方通常会先把它变换回原图；本函数提供的是「直接把 warped 曲线画到
    传入图像上」的简单版本，用于测试与调试。
    """
    out = np.array(image, dtype=np.uint8, copy=True)
    h, w = out.shape[0], out.shape[1]

    def draw(coeffs, color):
        if coeffs is None:
            return
        for v in range(h):
            u = int(round(float(np.polyval(coeffs, v))))
            if 0 <= u < w:
                u0, u1 = max(0, u - 2), min(w, u + 3)
                out[v, u0:u1] = color

    draw(result.left_fit, color_left)
    draw(result.right_fit, color_right)
    return out
