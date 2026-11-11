"""视觉检测：颜色分割 → 应答器观测；橙色带 → 管道观测。

只依赖 RGB-D 两张图，不接触仿真真值。应答器用唯一颜色编码做数据
关联（相当于海底的声学应答器编号）；管道用橙色涂装分割，给出
最近点距离与方位 —— 这就是"感知"层的几何基准，浑水下它不可靠，
学习式感知（CNN）的标签也来自这里对应的真值几何。
"""

import math

import numpy as np

from . import params as P
from .world import PALETTE, PIPE_COLOR

SAT_TH = 0.34        # 饱和度门限：水色/沉积物 ≈0.1，标记色都 >0.4
CHROMA_TH = 0.10     # 归一化色度距离门限
MIN_BRIGHT = 0.16
MIN_CHROMA_DIFF = 0.13
MIN_PIXELS = 8
MAX_LM_RANGE = 8.0   # 应答器观测距离上限（米）：立杆设计的可见范围
MAX_COL_GAP = 2      # 管道橙列段允许的最大列间隙

# 管道橙的判定：色度距离 + 亮度门限。不能只用通道关系 —— 红色
# 应答器（R 高 G 低）恰好也能通过 "R 大 G 中 B 小" 的组合，在起点
# 附近会被当成管道，给出完全错误的切向与横向估计。色度对光衰减
# 不敏感（各通道等比衰减），近处亮管道判定非常稳。
PIPE_CHROMA_TH = 0.13
PIPE_R_BRIGHT = 0.30
_PIPE_CHROMA = PIPE_COLOR / PIPE_COLOR.sum()

_PALETTE_CHROMA = PALETTE / PALETTE.sum(axis=1, keepdims=True)

# 背景色（上层水 / 下层水）的归一化色度：这两类颜色的像素一律不参与
# 标记分类。水色是蓝绿的，与任何标记色都留了 >=0.20 的余量，但浑水雾化
# 会把远处标记的颜色拉向水色 —— 显式排除比提高阈值更干净。
_BG_CHROMA = np.array([[0.05, 0.14, 0.19], [0.10, 0.24, 0.28]])
_BG_CHROMA = _BG_CHROMA / _BG_CHROMA.sum(axis=1, keepdims=True)
BG_CHROMA_TH = 0.11


def _column_bearing(cols):
    """图像列 → 车体方位角（左正右负）。"""
    col = np.asarray(cols, dtype=np.float64)
    return P.CAM_FOV / 2.0 - col * P.CAM_FOV / (P.CAM_W - 1)


def label_map(rgb):
    """逐像素应答器颜色分类，返回 label（-1 表示无效）。"""
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
    # 先剔背景水色，再判最近的标记色（顺序不能反：水色像素离某几个
    # 标记色本来就只有 0.1 左右，先分类会把整幅背景标成应答器）
    dbg = np.linalg.norm(chroma[:, None, :] - _BG_CHROMA[None, :, :],
                         axis=2).min(axis=1)
    keep0 = dbg >= BG_CHROMA_TH
    px, chroma = px[keep0], chroma[keep0]
    ys, xs = ys[keep0], xs[keep0]
    if ys.size == 0:
        return lab
    d2 = ((chroma[:, None, :] - _PALETTE_CHROMA[None, :, :]) ** 2).sum(axis=2)
    k = np.argmin(d2, axis=1)
    best = d2[np.arange(len(k)), k]
    ok = best < CHROMA_TH ** 2
    lab[ys[ok], xs[ok]] = k[ok]
    return lab


def pipe_mask(rgb):
    """管道橙色像素掩码（与应答器色度分类独立，互不污染）。"""
    img = rgb.astype(np.float32) / 255.0
    s = img.sum(axis=2) + 1e-6
    chroma = img / s[:, :, None]
    d2 = ((chroma - _PIPE_CHROMA[None, None, :]) ** 2).sum(axis=2)
    return (d2 < PIPE_CHROMA_TH ** 2) & (img[:, :, 0] > PIPE_R_BRIGHT)


def _column_segments(cols, gap=MAX_COL_GAP):
    """把出现过的列号切成若干连续段，返回最长的一段。"""
    u = np.unique(cols)
    if u.size == 0:
        return None
    splits = np.nonzero(np.diff(u) > gap)[0]
    segs = np.split(u, splits + 1)
    return max(segs, key=len)


def detect(rgb, depth, min_pixels=MIN_PIXELS):
    """从一帧 RGB-D 中提取应答器与管道点。

    返回
    ----
    landmarks : list[(id, range, bearing)]  按 id 升序
    pipe_pts  : (K, 2) 每行 (bearing, range)，管道上采到的点
    """
    lab = label_map(rgb)
    H = depth.shape[0]
    landmarks = []

    for lid in range(len(PALETTE)):
        rows, cols = np.nonzero(lab == lid)
        if rows.size < min_pixels:
            continue
        # 立杆是实心竖条：严格连续的列段、每列至少 2 个像素、段内像素
        # 密度足够。散点噪声（每列一两个像素、列间有大空隙）在这些
        # 判据下全被滤掉 —— 早期版本只用"最长段 + 平均密度"，一次
        # 噪声凑巧就能造出 11 m 外的假标记，把 EKF 的初始化毒化。
        u = np.unique(cols)
        splits = np.nonzero(np.diff(u) > 1)[0]
        best = None
        for sg in np.split(u, splits + 1):
            if len(sg) < 2 or sg[0] <= 0 or sg[-1] >= P.CAM_W - 1:
                continue
            keep = (cols >= sg[0]) & (cols <= sg[-1])
            if int(keep.sum()) < min_pixels:
                continue
            cnt = np.bincount(cols[keep] - sg[0], minlength=len(sg))
            if cnt.min() < 2 or keep.sum() / float(len(sg)) < 2.2:
                continue
            if best is None or keep.sum() > best[0]:
                best = (int(keep.sum()), sg, np.nonzero(keep)[0])
        if best is None:
            continue
        _, sg, idx = best
        rws = rows[idx]
        # 完全不接触图像边界的才认为完整可见（被裁切会让测距偏大）
        r_lo, r_hi = np.percentile(rws, [3.0, 97.0])
        if r_lo <= 0 or r_hi >= H - 1:
            continue
        c = 0.5 * (sg[0] + sg[-1])
        rng_m = float(np.median(depth[:, int(round(c))]))
        # 物理先验：只关心 8 m 以内的应答器。没有命中目标的射线深度
        # 是量程附近的噪声值（11~12 m），不加这个上限的话，散点噪声
        # 偶尔凑齐判据就会造出一个超远"标记"。
        if rng_m >= MAX_LM_RANGE:
            continue
        landmarks.append((lid, rng_m, float(_column_bearing(c))))

    # 管道点：橙色列按列取深度，每 2 列一个，喂给建图
    pm = pipe_mask(rgb)
    cols = np.nonzero(pm.any(axis=0))[0]
    pts = []
    for c in cols[::2]:
        r = float(depth[0, int(c)])
        if r >= P.CAM_RANGE_MAX - 1e-3:
            continue
        pts.append((float(_column_bearing(c)), r))

    landmarks.sort(key=lambda t: t[0])
    return landmarks, np.asarray(pts, dtype=np.float64).reshape(-1, 2)

def pipeline_observation(rgb, depth):
    """几何感知：从一帧图像估计管道跟踪偏差。

    返回 (found, dx_err, dtheta)：
    - dx_err  ：横向误差（米）。= FOLLOW_OFFSET - ROV 相对管道的有符号
                横向位置（沿管道绝对左法向，左正）。>0 表示还要向左挪。
    - dtheta  ：艏向与管道巡检切向的夹角（弧度，左正）。
    浑水/管道出视野时 found=False。
    """
    pm = pipe_mask(rgb)
    cols = np.nonzero(pm.any(axis=0))[0]
    if cols.size < 4:
        return False, 0.0, 0.0

    # 每列取管道深度（用第 0 行 —— 列内深度一致）
    ranges = depth[0, cols]
    ok = ranges < P.CAM_RANGE_MAX - 1e-3
    cols, ranges = cols[ok], ranges[ok]
    if cols.size < 4:
        return False, 0.0, 0.0
    rng_by_col = {int(c): float(r) for c, r in zip(cols, ranges)}

    i_near = int(np.argmin(ranges))
    r_near = float(ranges[i_near])
    c_near = int(cols[i_near])
    b_near = float(_column_bearing(c_near))

    # 切向估计：同一连续段内取离最近列足够远的另一列，两点连线即局部切向
    u = np.unique(cols)
    splits = np.nonzero(np.diff(u) > MAX_COL_GAP)[0]
    segs = np.split(u, splits + 1)
    seg = None
    for sg in segs:
        if c_near >= sg[0] and c_near <= sg[-1]:
            seg = sg
            break
    if seg is None:
        seg = np.array([c_near])
    offs = np.abs(seg.astype(int) - c_near)
    ok2 = (offs >= 4) & (offs <= 45)
    if ok2.any():
        c2 = int(seg[ok2][np.argmax(offs[ok2])])
    else:
        c2 = int(seg[int(np.argmax(offs))])
    r2 = rng_by_col.get(c2, float(depth[0, c2]))
    b2 = float(_column_bearing(c2))

    # 车体系极坐标 → 笛卡尔，两点连线方向即切向（两个朝向中取朝前的）
    p1 = np.array([r_near * math.cos(b_near), r_near * math.sin(b_near)])
    p2 = np.array([r2 * math.cos(b2), r2 * math.sin(b2)])
    tang = p2 - p1
    if float(np.hypot(*tang)) < 0.12:
        return False, 0.0, 0.0
    ang = math.atan2(tang[1], tang[0])
    if abs(ang) > math.pi / 2.0:
        ang = ang - math.pi * (1.0 if ang > 0 else -1.0)   # 取朝前的那支
    dtheta = float(ang)

    # 横向位置：最近点的有符号横向坐标（管道在左为正 → ROV 在管道右侧）
    lat_rov = -r_near * math.sin(b_near)
    dx_err = P.FOLLOW_OFFSET - lat_rov
    return True, float(dx_err), dtheta
