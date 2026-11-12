"""2.5D 高度图建图与"可降落性"规则评分。

分工：

- :class:`HeightMap` 把激光雷达/深度点云累积成固定世界网格上的表面高度（2.5D），
  用来回答"这块地方有多高、平不平、周围有没有障碍"。
- :func:`rule_landing_score` 是**规则老师**：把高度图变成 0~1 的可降落性评分。
  它只在本模块训练网络时用，部署时只跑网络（这样评分标准可以被复现，也不用在机上跑规则）。

网格固定在世界系原点周围（默认 24 m × 24 m、0.5 m 一格）。选固定网格而不是"以机体为中心
滚动"的原因是：迫降过程中机体会转、会被单桨失效带着自旋，滚动网格会让同一块地面在
不同时刻落到不同格子，地图对不上，评分也就抖。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

#: 认为"这个格子不是地面"的高度阈值（m）：树、楼房、墙体都高于它
OBSTACLE_HEIGHT = 0.3
#: 单格内允许的高度标准差（m），超过就认为不平
FLATNESS_TOL = 0.15
#: 落点周围要求的净空半径（m）
MARGIN_RADIUS = 1.5
#: 距离惩罚的归一化尺度（m）
DISTANCE_SCALE = 10.0


@dataclass(frozen=True)
class GridConfig:
    """世界系固定网格。"""

    size_m: float = 24.0
    cell_m: float = 0.5
    max_height: float = 14.0

    @property
    def shape(self) -> Tuple[int, int]:
        n = int(round(self.size_m / self.cell_m))
        return (n, n)

    @property
    def origin_xy(self) -> np.ndarray:
        half = 0.5 * self.size_m
        return np.array([-half, -half], dtype=float)


#: 地面点反投影后会有 -1e-8 量级的浮点误差，低于这个高度的一律当地面
GROUND_TOLERANCE = 0.05


def world_to_cells(cfg: GridConfig, points: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """世界坐标 (N,3) → (行索引, 列索引, 是否在网格内)。

    第三个返回值是**与输入等长**的掩码：索引数组已按它过滤过，调用方要用同一个掩码
    去筛其它逐点数组（比如高度），否则长度对不上。
    """
    pts = np.asarray(points, dtype=float).reshape(-1, 3)
    n_cells = cfg.shape[0]
    if pts.size == 0:
        empty = np.zeros((0,), dtype=np.int64)
        return empty, empty, np.zeros((0,), dtype=bool)
    rel = (pts[:, :2] - cfg.origin_xy) / cfg.cell_m
    ij = np.floor(rel).astype(np.int64)
    valid = (ij[:, 0] >= 0) & (ij[:, 0] < n_cells) & (ij[:, 1] >= 0) & (ij[:, 1] < n_cells)
    return ij[valid, 0], ij[valid, 1], valid


class HeightMap:
    """累积式 2.5D 高度图。

    每个格子记三件事：见过的最高表面高度、命中次数（用来判断"看过"而不是"猜"）、
    以及是否被观测过。地面的高度固定为 0，所以"没命中任何东西的射线"不写地图——
    lidar 打不到东西说明那个方向没有障碍，不代表那里是坑。
    """

    def __init__(self, cfg: Optional[GridConfig] = None):
        self.cfg = cfg or GridConfig()
        self.height = np.zeros(self.cfg.shape, dtype=np.float32)
        self.hits = np.zeros(self.cfg.shape, dtype=np.int32)
        self.observed = np.zeros(self.cfg.shape, dtype=bool)

    # ------------------------------------------------------------------ 更新
    def update(self, points: np.ndarray) -> None:
        """把一帧世界系点云并入地图（取每格最高点）。"""
        pts = np.asarray(points, dtype=float).reshape(-1, 3)
        if pts.size == 0:
            return
        # 地面点反投影后常落在 -1e-8 这种量级的负值上，不能直接用 z >= 0 过滤
        z = np.clip(pts[:, 2], 0.0, self.cfg.max_height)
        keep = np.isfinite(pts[:, 2]) & (pts[:, 2] >= -GROUND_TOLERANCE) \
            & (pts[:, 2] <= self.cfg.max_height)
        if not np.any(keep):
            return
        ii, jj, inside = world_to_cells(self.cfg, pts[keep])
        if ii.size == 0:
            return
        heights = z[keep][inside].astype(np.float32)
        flat_idx = ii * self.cfg.shape[1] + jj
        np.maximum.at(self.height.reshape(-1), flat_idx, heights)
        np.add.at(self.hits.reshape(-1), flat_idx, 1)
        self.observed[ii, jj] = True

    def reset(self) -> None:
        self.height[:] = 0.0
        self.hits[:] = 0
        self.observed[:] = False

    # ------------------------------------------------------------------ 查询
    @property
    def shape(self) -> Tuple[int, int]:
        return self.height.shape

    def obstacle_mask(self) -> np.ndarray:
        """"这个格子上面有东西"的掩码。"""
        return self.height > OBSTACLE_HEIGHT

    def cell_center(self, row: int, col: int) -> np.ndarray:
        return self.cfg.origin_xy + (np.array([row, col], dtype=float) + 0.5) * self.cfg.cell_m

    def cell_of(self, xy: np.ndarray) -> Tuple[int, int]:
        rel = (np.asarray(xy, dtype=float)[:2] - self.cfg.origin_xy) / self.cfg.cell_m
        ij = np.floor(rel).astype(int)
        n = self.shape[0]
        return int(np.clip(ij[0], 0, n - 1)), int(np.clip(ij[1], 0, n - 1))

    def ground_fraction(self) -> float:
        """已观测格子里属于地面的比例（地形越空越接近 1）。"""
        obs = self.observed
        if not np.any(obs):
            return 0.0
        return float(np.mean(~self.obstacle_mask()[obs]))


def _local_stats(height: np.ndarray, observed: np.ndarray,
                 radius: int = 1) -> Tuple[np.ndarray, np.ndarray]:
    """在 (2r+1)² 邻域内算高度标准差与有效观测数，未观测格子不参与。"""
    h = np.where(observed, height, np.nan)
    n = 2 * radius + 1
    pad = np.pad(h, radius, constant_values=np.nan)
    stack = []
    for di in range(n):
        for dj in range(n):
            stack.append(pad[di:di + h.shape[0], dj:dj + h.shape[1]])
    stack = np.stack(stack, axis=0)                      # (k, R, C)
    finite = np.isfinite(stack)
    cnt = np.sum(finite, axis=0).astype(np.float32)
    import warnings

    with warnings.catch_warnings():
        # 邻域全是未观测格时 nanmean 会告警，下面已经用 nan_to_num 兜住了
        warnings.simplefilter("ignore", RuntimeWarning)
        mean = np.nanmean(stack, axis=0)
        var = np.nanmean((stack - mean[None, :, :]) ** 2, axis=0)
    mean = np.nan_to_num(mean, nan=0.0)
    var = np.nan_to_num(var, nan=0.0)
    # 邻域里只有一个点时标准差没有意义，按"平"处理，改由 confidence 项体现覆盖不足
    std = np.where(cnt >= 2.0, np.sqrt(np.maximum(var, 0.0)), 0.0)
    return std.astype(np.float32), cnt


def _distance_to_obstacle(obstacle: np.ndarray, cell_m: float, radius_cells: int = 8) -> np.ndarray:
    """每个格子到最近障碍格的距离（m），超过 ``radius_cells`` 的截断到上限。

    直接用欧氏距离算（障碍格个数通常几百个，网格 48²，一次计算约几毫秒），
    比逐层膨胀的写法短且不容易写错——之前那版膨胀在四邻域判断上写反了，
    导致所有格子都拿到上限值。
    """
    R, C = obstacle.shape
    limit = radius_cells * cell_m
    if not np.any(obstacle):
        return np.full((R, C), limit, dtype=np.float32)

    obs = np.argwhere(obstacle).astype(np.float64)          # (M,2)
    rows, cols = np.meshgrid(np.arange(R), np.arange(C), indexing="ij")
    cells = np.stack([rows.ravel(), cols.ravel()], axis=1).astype(np.float64)

    out = np.empty(cells.shape[0], dtype=np.float64)
    chunk = 512
    for start in range(0, cells.shape[0], chunk):
        block = cells[start:start + chunk]
        d2 = ((block[:, None, :] - obs[None, :, :]) ** 2).sum(axis=-1)
        out[start:start + chunk] = np.sqrt(d2.min(axis=1))
    return np.minimum(out.reshape(R, C) * cell_m, limit).astype(np.float32)


def rule_landing_score(height_map: HeightMap,
                       position_xy: Optional[np.ndarray] = None,
                       cfg: Optional[GridConfig] = None) -> np.ndarray:
    """规则老师给出的可降落性评分（0~1，未观测处为 0）。

    四条判据相乘，任一条不满足就压到 0 附近：

    ==========  ==================================================
    地面        格子高度 < ``OBSTACLE_HEIGHT``，且确实被观测到
    平整        3×3 邻域高度标准差 < ``FLATNESS_TOL``
    净空        到最近障碍格的距离 / ``MARGIN_RADIUS``（截断到 1）
    距离        离当前位置越近越好（``DISTANCE_SCALE`` 归一化）
    ==========  ==================================================
    """
    cfg = cfg or height_map.cfg
    observed = height_map.observed
    height = height_map.height
    obstacle = height_map.obstacle_mask()

    free = observed & ~obstacle
    std, cnt = _local_stats(height, observed, radius=1)
    flat = np.clip(1.0 - std / FLATNESS_TOL, 0.0, 1.0)
    margin_m = _distance_to_obstacle(obstacle, cfg.cell_m)
    margin = np.clip(margin_m / MARGIN_RADIUS, 0.0, 1.0)
    # 邻域观测太少时降低置信度（单帧稀疏点云不该给出满分的落点）
    conf = 0.5 + 0.5 * np.clip(cnt / 4.0, 0.0, 1.0)

    if position_xy is not None:
        R, C = height_map.shape
        rows, cols = np.meshgrid(np.arange(R), np.arange(C), indexing="ij")
        cx = cfg.origin_xy[0] + (rows + 0.5) * cfg.cell_m
        cy = cfg.origin_xy[1] + (cols + 0.5) * cfg.cell_m
        d = np.hypot(cx - float(position_xy[0]), cy - float(position_xy[1]))
        near = np.clip(1.0 - d / DISTANCE_SCALE, 0.0, 1.0)
    else:
        near = np.ones(height_map.shape, dtype=float)

    score = free.astype(float) * flat * margin * conf * (0.35 + 0.65 * near)
    return np.clip(score, 0.0, 1.0).astype(np.float32)


def best_cell(score: np.ndarray, height_map: HeightMap,
              min_score: float = 0.25) -> Optional[Tuple[int, int, float]]:
    """取评分最高的格子，返回 ``(row, col, score)``；全图都不可降落时返回 None。"""
    if score.size == 0 or float(np.max(score)) < min_score:
        return None
    idx = int(np.argmax(score))
    row, col = np.unravel_index(idx, score.shape)
    return int(row), int(col), float(score[row, col])


def height_map_input(height_map: HeightMap, cfg: Optional[GridConfig] = None) -> np.ndarray:
    """把高度图转成网络输入：1 通道，障碍高度归一化到 [0,1]，未观测处填 1（当成高度未知的障碍）。

    未观测填 1 而不是 0 很重要：填 0 等于告诉网络"这里是平地"，会让它把没看过的地方当成
    好落点——这正好是迫降时最危险的错误。
    """
    cfg = cfg or height_map.cfg
    h = height_map.height / max(cfg.max_height, 1e-6)
    h = np.where(height_map.observed, h, 1.0)
    return h[None, :, :].astype(np.float32)


# --------------------------------------------------------------------------- #
# 深度相机 → 世界系点云
# --------------------------------------------------------------------------- #
def depth_to_points(depth: np.ndarray, cam_pos: np.ndarray, cam_mat: np.ndarray,
                    fovy_deg: float, max_range: float = 20.0,
                    min_range: float = 0.2) -> np.ndarray:
    """把一张深度图反投影成世界系点云。

    MuJoCo 的深度渲染给的是**相机坐标系 z 轴方向的距离**（米），相机朝自身 -z 看，
    所以先把像素反投到相机系（``x`` 右、``y`` 上、``z`` 后），再右乘相机旋转矩阵转世界系。
    像素是正方形，故 ``fx = fy = 0.5·H / tan(fovy/2)``。
    """
    depth = np.asarray(depth, dtype=float)
    h, w = depth.shape
    fy = 0.5 * h / np.tan(0.5 * np.deg2rad(float(fovy_deg)))
    fx = fy
    cx = (w - 1) * 0.5
    cy = (h - 1) * 0.5

    valid = np.isfinite(depth) & (depth > min_range) & (depth < max_range)
    if not np.any(valid):
        return np.zeros((0, 3), dtype=float)
    v, u = np.nonzero(valid)
    z = depth[v, u]
    x = (u - cx) * z / fx
    y = -(v - cy) * z / fy
    points_cam = np.stack([x, y, -z], axis=1)
    mat = np.asarray(cam_mat, dtype=float).reshape(3, 3)
    return points_cam @ mat.T + np.asarray(cam_pos, dtype=float).reshape(3)


class DepthMapper:
    """用机体的下视深度相机建 2.5D 高度图的采样器。

    选下视相机（``down_cam``，fovy 80°）而不是激光雷达有两个原因：雷达只有 -15°/0°/+15°
    三层，飞行高度超过 3 m 时朝下的射线打不到地面；下视相机在 4 m 高度就能覆盖约 7 m 见方
    的地面，而且正好对应课程任务里"深度摄像头感知"的设定。
    """

    def __init__(self, api, camera: str = "down_cam", width: int = 96, height: int = 96,
                 max_range: float = 20.0):
        self.api = api
        self.camera = camera
        self.width = int(width)
        self.height = int(height)
        self.max_range = float(max_range)
        self._vision = None
        self._model = None
        self._cam_id = -1
        self._fovy = 45.0

    def bind(self, quad) -> "DepthMapper":
        """绑定到一台飞行器（模型变了要重新绑定）。"""
        import mujoco

        self._model = quad.model
        cfg = self.api.CameraConfig(width=self.width, height=self.height,
                                    name=self.camera, depth_max_range=self.max_range)
        self._vision = self.api.VisionSensors(quad.model, cfg)
        self._cam_id = int(mujoco.mj_name2id(quad.model, mujoco.mjtObj.mjOBJ_CAMERA,
                                             self.camera))
        self._fovy = float(quad.model.cam_fovy[self._cam_id]) if self._cam_id >= 0 else 45.0
        return self

    def scan(self, data) -> np.ndarray:
        """返回一帧世界系点云 (N,3)。调用前需要 ``mujoco.mj_forward``。"""
        if self._vision is None:
            raise RuntimeError("DepthMapper 还没 bind 到飞行器")
        depth = self._vision.depth(data, camera=self.camera)
        return depth_to_points(depth, data.cam_xpos[self._cam_id],
                               data.cam_xmat[self._cam_id], self._fovy,
                               max_range=self.max_range)
