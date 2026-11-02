"""视觉检测：颜色分割 → 路标观测；深度图 → 墙体点。

只依赖 RGB-D 两张图，不接触仿真真值。路标用唯一的颜色编码做数据
关联（相当于一个视觉标签系统），墙点用于建图。

灰度墙面/地板的饱和度很低，先用饱和度门限筛出候选像素，再在候选像素
上做归一化色度最近邻 —— 这样既快（只有少量像素参与分类），又不会被
灰色表面误判成某种路标颜色。
"""

import numpy as np

from . import params as P
from .world import PALETTE

SAT_TH = 0.34        # 饱和度门限：墙面/地板 ≈0.06，最低的粉色路标 ≈0.39
CHROMA_TH = 0.10     # 归一化色度距离门限
MIN_BRIGHT = 0.16    # 最大通道亮度门限
MIN_CHROMA_DIFF = 0.13   # 通道极差门限：暗光下噪声会把灰墙的极差抬到 0.1 以上
MIN_PIXELS = 8
MAX_COL_GAP = 2      # 同一路标的列允许的最大间断

_PALETTE_CHROMA = PALETTE / PALETTE.sum(axis=1, keepdims=True)


def _column_bearing(cols):
    """图像列 → 车体方位角（左正右负）。"""
    col = np.asarray(cols, dtype=np.float64)
    return P.CAM_FOV / 2.0 - col * P.CAM_FOV / (P.CAM_W - 1)


def label_map(rgb):
    """逐像素颜色分类，返回 label（-1 表示无效）。"""
    img = rgb.astype(np.float32) / 255.0
    mx = img.max(axis=2)
    mn = img.min(axis=2)
    sat = (mx - mn) / (mx + 1e-6)

    cand = (sat > SAT_TH) & (mx > MIN_BRIGHT) & ((mx - mn) > MIN_CHROMA_DIFF)
    lab = np.full(img.shape[:2], -1, dtype=np.int32)
    ys, xs = np.nonzero(cand)
    if ys.size == 0:
        return lab

    px = img[ys, xs]
    chroma = px / (px.sum(axis=1, keepdims=True) + 1e-6)
    d2 = ((chroma[:, None, :] - _PALETTE_CHROMA[None, :, :]) ** 2).sum(axis=2)
    k = np.argmin(d2, axis=1)
    best = d2[np.arange(len(k)), k]
    ok = best < CHROMA_TH ** 2
    lab[ys[ok], xs[ok]] = k[ok]
    return lab


def _column_segments(cols, gap=MAX_COL_GAP):
    """把出现过的列号切成若干连续段，返回最长的一段。"""
    u = np.unique(cols)
    if u.size == 0:
        return None
    splits = np.nonzero(np.diff(u) > gap)[0]
    segs = np.split(u, splits + 1)
    return max(segs, key=len)


def detect(rgb, depth, min_pixels=MIN_PIXELS):
    """从一帧 RGB-D 中提取路标与墙体点。

    返回
    ----
    landmarks : list[(id, range, bearing)]  按 id 升序
    wall_pts  : (K, 2) 每行 (bearing, range)
    """
    lab = label_map(rgb)
    H = depth.shape[0]
    landmarks = []

    for lid in range(len(PALETTE)):
        rows, cols = np.nonzero(lab == lid)
        if rows.size < min_pixels:
            continue
        seg = _column_segments(cols)
        if seg is None or len(seg) < 2:
            continue
        col_lo, col_hi = int(seg[0]), int(seg[-1])
        keep = (cols >= col_lo) & (cols <= col_hi)
        if int(keep.sum()) < min_pixels:
            continue
        # 竖直柱子在每一列上都是连续的一整段像素；噪声散点每列只有一两个。
        # 这个"每列像素密度"判据是除掉夜间误检最有效的一条。
        if keep.sum() / float(len(seg)) < 3.0:
            continue
        rws = rows[keep]
        # 完全不接触图像边界的才认为完整可见（被裁切会让测距偏大）
        if col_lo <= 0 or col_hi >= P.CAM_W - 1:
            continue
        # 用 3/97 分位判边界：单个噪声散点不该让整根柱子被丢掉（夜间尤其明显）
        r_lo, r_hi = np.percentile(rws, [3.0, 97.0])
        if r_lo <= 0 or r_hi >= H - 1:
            continue
        c = 0.5 * (col_lo + col_hi)
        rng_m = float(np.median(depth[:, int(round(c))]))
        if rng_m >= P.CAM_RANGE_MAX - 1e-3:
            continue
        landmarks.append((lid, rng_m, float(_column_bearing(c))))

    # 深度点：全部列都返回（路标本身也是障碍物，必须进地图），每 2 列取一个
    wall_pts = []
    for c in range(0, P.CAM_W, 2):
        r = float(depth[0, c])
        if r >= P.CAM_RANGE_MAX - 1e-3:
            continue
        wall_pts.append((float(_column_bearing(c)), r))

    landmarks.sort(key=lambda t: t[0])
    return landmarks, np.asarray(wall_pts, dtype=np.float64).reshape(-1, 2)


def detection_rate(landmarks, truth_visible):
    """检测率：检出数 / 视野内真实路标数，用于夜间场景评价。"""
    if not truth_visible:
        return 1.0
    got = len({l[0] for l in landmarks} & set(truth_visible))
    return got / float(len(truth_visible))
