"""路径后处理：视线捷径（string pulling）与梯度平滑。

栅格 A* 给出的路径被限制在 8 个方向上，转弯生硬、长度偏大，直接喂给
移动机器人会导致频繁加减速。这里做两级后处理：

1. **视线捷径**：若两点之间的直线不穿越障碍，就用直线段替换中间所有点。
   这一步能大幅缩短路径、减少拐点，且**严格不降低安全性**（每段都做了
   超采样碰撞检查）。
2. **梯度平滑**：对剩余折点做迭代优化，目标函数 = 平滑项 + 障碍斥力项，
   把贴着墙角走的折线推离障碍，同时保持路径顺滑。

所有函数都工作在**世界坐标**上（单位 m），与栅格索引解耦，因此平滑结果
可以直接作为控制器（如 ``ros2_pure_pursuit``）的参考路径。
"""

from __future__ import annotations

import math

import numpy as np

from .gridmap import GridMap


# ---------------- 碰撞检查 ----------------


def segment_is_free(gm: GridMap, a: np.ndarray, b: np.ndarray, step: float | None = None) -> bool:
    """检查世界坐标下线段 ``a -> b`` 是否全程可通行。

    采用超采样：采样间距取分辨率的 1/4，且每个采样点都查询其所在栅格，
    比标准 Bresenham 更保守，能避免"斜着擦过格角"的漏检。
    """
    s = gm.spec
    ds = step if step is not None else s.resolution * 0.25
    length = float(np.linalg.norm(b - a))
    n = max(2, int(math.ceil(length / max(ds, 1e-9))) + 1)
    ts = np.linspace(0.0, 1.0, n)
    pts = a[None, :] + ts[:, None] * (b - a)[None, :]
    for x, y in pts:
        if not _cell_free(gm, float(x), float(y)):
            return False
    return True


def _cell_free(gm: GridMap, x: float, y: float) -> bool:
    """越界或占用都返回 False。"""
    i, j = gm.to_grid(x, y)
    return gm.is_free(i, j)


# ---------------- 视线捷径 ----------------


def shortcut(
    gm: GridMap,
    points: np.ndarray,
    max_iter: int = 200,
    rng: np.random.Generator | None = None,
    check_free: bool = True,
) -> np.ndarray:
    """随机视线捷径（string pulling）。

    Args:
        gm: 膨胀后的地图。
        points: ``(N, 2)`` 世界坐标折线。
        max_iter: 尝试次数。
        rng: 随机源，固定种子可复现。
        check_free: 是否做碰撞检查（仅用于测试/诊断，正常必须为 True）。

    Returns:
        缩短后的折线，首尾点保持不变。
    """
    pts = np.asarray(points, dtype=float).copy()
    if len(pts) < 3:
        return pts
    rand = rng or np.random.default_rng(0)
    n = len(pts)
    for _ in range(max_iter):
        if n <= 2:
            break
        i = int(rand.integers(0, n - 2))
        j = int(rand.integers(i + 2, n))
        if check_free and not segment_is_free(gm, pts[i], pts[j]):
            continue
        pts = np.concatenate([pts[: i + 1], pts[j:]], axis=0)
        n = len(pts)
    return pts


# ---------------- 梯度平滑 ----------------


def gradient_smooth(
    gm: GridMap,
    points: np.ndarray,
    w_smooth: float = 0.35,
    w_obstacle: float = 0.25,
    alpha: float = 0.12,
    iterations: int = 120,
    clearance: float | None = None,
) -> np.ndarray:
    """梯度下降平滑（首尾固定）。

    目标函数（对每个内部点 :math:`p_i`）：

    .. math::
        E_i = w_s \\|p_i - \\tfrac{1}{2}(p_{i-1}+p_{i+1})\\|^2
              + w_o \\max(0, d_{clear} - d(p_i))^2

    第一项把折线拉直，第二项只在点距离障碍小于 ``clearance`` 时产生斥力，
    把点推离墙面。梯度可直接解析求出，无需自动微分。

    Args:
        clearance: 期望离障碍的最小距离 (m)，默认取 2 倍分辨率。
    """
    pts = np.asarray(points, dtype=float).copy()
    if len(pts) < 3:
        return pts
    s = gm.spec
    d_clear = clearance if clearance is not None else 2.0 * s.resolution
    res = s.resolution

    for _ in range(iterations):
        new = pts.copy()
        for i in range(1, len(pts) - 1):
            p = pts[i]
            # 平滑项梯度：2 * w_s * (p - midpoint)
            midpoint = 0.5 * (pts[i - 1] + pts[i + 1])
            grad = 2.0 * w_smooth * (p - midpoint)

            # 障碍斥力：查询周围 3x3 邻格的占用情况，估计最近障碍方向
            ci, cj = gm.to_grid(p[0], p[1])
            nearest = None
            best = math.inf
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    ni, nj = ci + di, cj + dj
                    if not gm.in_bounds(ni, nj) or gm.grid[ni, nj] == 0:
                        continue
                    ox, oy = gm.to_world(ni, nj)
                    d = math.hypot(p[0] - ox, p[1] - oy)
                    if d < best:
                        best, nearest = d, np.array([ox, oy])
            if nearest is not None and best < d_clear:
                diff = p - nearest
                norm = max(float(np.linalg.norm(diff)), 1e-6)
                # 斥力方向：从障碍指向当前点，大小正比于间隙缺口
                grad -= 2.0 * w_obstacle * (d_clear - best) * (diff / norm)

            new[i] = p - alpha * grad
            # 单步位移夹紧，防止数值发散
            delta = new[i] - p
            dn = float(np.linalg.norm(delta))
            if dn > res:
                new[i] = p + delta * (res / dn)
        pts = new
    return pts


# ---------------- 组合流程与度量 ----------------


def smooth_path(
    gm: GridMap,
    points: np.ndarray,
    use_gradient: bool = True,
    seed: int = 0,
) -> tuple[np.ndarray, dict]:
    """捷径 + 梯度平滑的完整流程，并返回前后对比指标。"""
    raw = np.asarray(points, dtype=float)
    before_len = polyline_length(raw)
    before_turns = count_turns_world(raw)

    cut = shortcut(gm, raw, rng=np.random.default_rng(seed))
    after_cut_len = polyline_length(cut)

    out = gradient_smooth(gm, cut) if use_gradient else cut
    # 平滑后逐段复核碰撞；若某段被推入障碍，回退到捷径结果，保证输出安全
    if use_gradient and not all(
        segment_is_free(gm, out[i], out[i + 1]) for i in range(len(out) - 1)
    ):
        out = cut

    metrics = {
        "raw_length": before_len,
        "after_shortcut_length": after_cut_len,
        "final_length": polyline_length(out),
        "raw_turns": before_turns,
        "final_turns": count_turns_world(out),
        "raw_points": len(raw),
        "final_points": len(out),
        "safe": all(
            segment_is_free(gm, out[i], out[i + 1]) for i in range(len(out) - 1)
        ),
    }
    return out, metrics


def polyline_length(points: np.ndarray) -> float:
    """折线的世界坐标长度 (m)。"""
    pts = np.asarray(points, dtype=float)
    if len(pts) < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))


def count_turns_world(points: np.ndarray, tol_deg: float = 3.0) -> int:
    """统计折线中大于阈值的转角个数。"""
    pts = np.asarray(points, dtype=float)
    if len(pts) < 3:
        return 0
    d = np.diff(pts, axis=0)
    ang = np.arctan2(d[:, 1], d[:, 0])
    delta = np.abs(np.diff(ang))
    delta = np.minimum(delta, 2 * math.pi - delta)
    return int(np.sum(delta > math.radians(tol_deg)))
