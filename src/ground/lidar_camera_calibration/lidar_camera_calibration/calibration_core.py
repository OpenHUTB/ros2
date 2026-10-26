#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""相机-激光雷达外参标定的核心库（合成数据 + PnP，纯离线可运行）。

背景
----
仓库里只有 ROS 1 的相机检测/标定示例（``src/chap5/robot_vision``），
没有"激光雷达与相机之间外参标定"的 ROS 2 模块。本库用**已知真值的合成
场景**把标定流程完整跑通：多姿态棋盘格点云 → 加传感器噪声 → PnP 求外参
→ 与真值比较，给出可量化的误差指标。

坐标约定
--------
外参 ``(R, t)`` 表示 **LiDAR → 相机**：``p_cam = R @ p_lidar + t``。
``cv2.projectPoints`` / ``cv2.solvePnP`` 的 ``rvec`` 由 ``R`` 取 Rodrigues 得到。

为什么不直接把点云投影到真图上
------------------------------
合成场景的好处是**真值已知**，能直接回答"标定误差有多大"；本模块因此
把验证重点放在误差指标与噪声敏感性上，而不是追求照片级渲染。真实数据
采集与棋盘格检测列入文档的未验证项。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Sequence, Tuple

import cv2
import numpy as np

# ---------------- 默认相机内参（640x480，无畸变） ----------------
DEFAULT_K = np.array(
    [[600.0, 0.0, 320.0],
     [0.0, 600.0, 240.0],
     [0.0, 0.0, 1.0]], dtype=np.float64)
DEFAULT_DIST = np.zeros(5, dtype=np.float64)

# ---------------- 默认真值外参：LiDAR -> 相机 ----------------
# 相机相对雷达偏右 12 cm、下方 3 cm、上方 18 cm，并略带俯仰/偏航。
DEFAULT_RVEC_DEG = (5.0, -3.0, 0.5)
DEFAULT_T = np.array([0.12, -0.03, 0.18], dtype=np.float64)

# 标定板参数：7x5 内角点、方格 6 cm（板面 0.36 x 0.24 m）
BOARD_COLS = 7
BOARD_ROWS = 5
SQUARE_SIZE = 0.06

# 标定板在 LiDAR 坐标系下的 6 个摆放位姿（绕 x/y/z 旋转角，平移量）
# 距离约 1 m：在 640x480、fx=600 的画面中约占 200x130 像素。
# 实测（见文档第六节）板面过小会让 PnP 病态，误差可达 168°。
VIEW_POSES: Sequence[Tuple[Tuple[float, float, float], Tuple[float, float, float]]] = (
    ((0.0, 0.0, 0.0), (-0.15, -0.10, 1.00)),
    ((0.0, 40.0, 0.0), (-1.00, -0.05, 1.10)),
    ((0.0, -35.0, 0.0), (0.40, -0.05, 1.15)),
    ((15.0, 20.0, 0.0), (-0.55, 0.25, 1.20)),
    ((-15.0, -20.0, 0.0), (-0.35, -0.45, 1.05)),
    ((10.0, 10.0, 25.0), (-0.70, 0.05, 1.30)),
)


def default_rvec() -> np.ndarray:
    return np.array([math.radians(a) for a in DEFAULT_RVEC_DEG], dtype=np.float64)


def default_R() -> np.ndarray:
    return cv2.Rodrigues(default_rvec())[0]


def board_object_points(cols: int = BOARD_COLS, rows: int = BOARD_ROWS,
                        square: float = SQUARE_SIZE) -> np.ndarray:
    """标定板局部坐标下的内角点（z=0 平面）。"""
    pts = [[c * square, r * square, 0.0] for r in range(rows) for c in range(cols)]
    return np.asarray(pts, dtype=np.float64)


def board_cells(cols: int = BOARD_COLS, rows: int = BOARD_ROWS,
                square: float = SQUARE_SIZE) -> np.ndarray:
    """标定板单元格四角（用于绘制合成图像）：形状 (rows-1)*(cols-1), 4, 3。"""
    cells = []
    for r in range(rows - 1):
        for c in range(cols - 1):
            x0, y0 = c * square, r * square
            cells.append([[x0, y0, 0.0], [x0 + square, y0, 0.0],
                          [x0 + square, y0 + square, 0.0],
                          [x0, y0 + square, 0.0]])
    return np.asarray(cells, dtype=np.float64)


def _pose(rot_deg: Sequence[float], trans: Sequence[float]) -> Tuple[np.ndarray, np.ndarray]:
    R = cv2.Rodrigues(np.radians(np.asarray(rot_deg, dtype=np.float64)))[0]
    return R, np.asarray(trans, dtype=np.float64)


def build_scene(n_views: int = 3) -> Tuple[np.ndarray, List[Tuple[np.ndarray, np.ndarray]]]:
    """按前 ``n_views`` 个位姿生成 LiDAR 坐标系下的 3D 角点点集。

    Returns:
        points: (N, 3)
        poses:  每个摆位对应的 (R_board, t_board)
    """
    if not 1 <= n_views <= len(VIEW_POSES):
        raise ValueError('n_views 必须在 1..%d 之间' % len(VIEW_POSES))
    board = board_object_points()
    chunks = []
    poses = []
    for rot_deg, trans in VIEW_POSES[:n_views]:
        R, t = _pose(rot_deg, trans)
        chunks.append((R @ board.T).T + t)
        poses.append((R, t))
    return np.vstack(chunks), poses


def project(points: np.ndarray, R: np.ndarray, t: np.ndarray,
            K: np.ndarray = DEFAULT_K, D: np.ndarray = DEFAULT_DIST) -> np.ndarray:
    """LiDAR 坐标系点 -> 像素坐标。"""
    rvec = cv2.Rodrigues(R)[0]
    uv, _ = cv2.projectPoints(points, rvec, t.reshape(3, 1), K, D)
    return uv.reshape(-1, 2)


def simulate_lidar_noise(points: np.ndarray, rng: np.random.Generator,
                         sigma_range: float = 0.01,
                         sigma_angle_deg: float = 0.1) -> np.ndarray:
    """模拟激光雷达测量噪声：距离加高斯、方向加小角度扰动。"""
    r = np.linalg.norm(points, axis=1)
    unit = points / r[:, None]
    r_noisy = r + rng.normal(0.0, sigma_range, size=r.shape)
    unit_noisy = unit + rng.normal(0.0, math.radians(sigma_angle_deg), size=unit.shape)
    unit_noisy /= np.linalg.norm(unit_noisy, axis=1, keepdims=True)
    return unit_noisy * r_noisy[:, None]


def solve_extrinsics(points_lidar: np.ndarray, uv: np.ndarray,
                     K: np.ndarray = DEFAULT_K,
                     D: np.ndarray = DEFAULT_DIST) -> Tuple[bool, np.ndarray, np.ndarray]:
    """EPnP 求初值 + 迭代精化，返回 (ok, R, t)。"""
    ok, rvec, tvec = cv2.solvePnP(points_lidar, uv, K, D, flags=cv2.SOLVEPNP_EPNP)
    if not ok:
        return False, np.eye(3), np.zeros(3)
    ok, rvec, tvec = cv2.solvePnP(
        points_lidar, uv, K, D, rvec, tvec,
        useExtrinsicGuess=True, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return False, np.eye(3), np.zeros(3)
    return True, cv2.Rodrigues(rvec)[0], tvec.reshape(3)


def rotation_error_deg(R_est: np.ndarray, R_true: np.ndarray = None) -> float:
    R_true = default_R() if R_true is None else R_true
    cos = (np.trace(R_true.T @ R_est) - 1.0) / 2.0
    return math.degrees(math.acos(float(np.clip(cos, -1.0, 1.0))))


def translation_error_m(t_est: np.ndarray, t_true: np.ndarray = None) -> float:
    t_true = DEFAULT_T if t_true is None else t_true
    return float(np.linalg.norm(t_est - t_true))


def reprojection_rmse(points_lidar: np.ndarray, uv: np.ndarray,
                      R: np.ndarray, t: np.ndarray,
                      K: np.ndarray = DEFAULT_K,
                      D: np.ndarray = DEFAULT_DIST) -> float:
    uv_hat = project(points_lidar, R, t, K, D)
    return float(np.sqrt(np.mean(np.sum((uv_hat - uv) ** 2, axis=1))))


@dataclass
class CalibrationResult:
    """一次标定的结果与指标。"""
    views: int
    n_points: int
    pixel_noise: float
    range_noise: float
    angle_noise_deg: float
    R_est: np.ndarray
    t_est: np.ndarray
    rot_error_deg: float
    trans_error_m: float
    rmse_px: float
    R_true: np.ndarray = field(default_factory=default_R)
    t_true: np.ndarray = field(default_factory=lambda: DEFAULT_T.copy())

    def to_text(self) -> str:
        return '\n'.join([
            '摆位次数: %d   点数: %d' % (self.views, self.n_points),
            '噪声设置: 像素 %.2f px / 测距 %.3f m / 角 %.2f°'
            % (self.pixel_noise, self.range_noise, self.angle_noise_deg),
            '',
            '旋转误差: %.3f °' % self.rot_error_deg,
            '平移误差: %.4f m' % self.trans_error_m,
            '重投影 RMSE: %.3f px' % self.rmse_px,
            '',
            '估计外参 (LiDAR -> 相机):',
            'R =',
            np.array2string(self.R_est, precision=4, suppress_small=True,
                            prefix='    '),
            't = [%.4f, %.4f, %.4f] m' % tuple(self.t_est),
            '真值 t = [%.4f, %.4f, %.4f] m' % tuple(self.t_true),
        ])


def calibrate(views: int = 3, pixel_noise: float = 1.0,
              range_noise: float = 0.01, angle_noise_deg: float = 0.1,
              seed: int = 0,
              K: np.ndarray = DEFAULT_K,
              D: np.ndarray = DEFAULT_DIST) -> CalibrationResult:
    """跑一次完整标定：生成场景 → 加噪声 → PnP → 计算指标。"""
    points, _ = build_scene(views)
    rng = np.random.default_rng(seed)
    points_meas = simulate_lidar_noise(points, rng, range_noise, angle_noise_deg)
    uv_clean = project(points, default_R(), DEFAULT_T, K, D)
    uv_meas = uv_clean + rng.normal(0.0, pixel_noise, size=uv_clean.shape)

    ok, R_est, t_est = solve_extrinsics(points_meas, uv_meas, K, D)
    if not ok:
        raise RuntimeError('solvePnP 失败（点数或配置异常）')

    return CalibrationResult(
        views=views,
        n_points=len(points),
        pixel_noise=pixel_noise,
        range_noise=range_noise,
        angle_noise_deg=angle_noise_deg,
        R_est=R_est,
        t_est=t_est,
        rot_error_deg=rotation_error_deg(R_est),
        trans_error_m=translation_error_m(t_est),
        rmse_px=reprojection_rmse(points_meas, uv_meas, R_est, t_est, K, D),
    )


def noise_sweep(trials: int = 200, views: int = 3,
                pixel_noises: Sequence[float] = (0.0, 0.5, 1.0, 2.0),
                range_noises: Sequence[float] = (0.0, 0.01, 0.02),
                angle_noise_deg: float = 0.1) -> List[dict]:
    """噪声敏感性扫描，每个组合跑 ``trials`` 次取中位数。"""
    rows = []
    for sigma_px in pixel_noises:
        for sigma_r in range_noises:
            rot, trans, rmse = [], [], []
            for seed in range(trials):
                res = calibrate(views, sigma_px, sigma_r, angle_noise_deg, seed)
                rot.append(res.rot_error_deg)
                trans.append(res.trans_error_m)
                rmse.append(res.rmse_px)
            rows.append(dict(
                pixel_noise=sigma_px, range_noise=sigma_r,
                angle_noise_deg=angle_noise_deg,
                rot_error_deg=float(np.median(rot)),
                trans_error_m=float(np.median(trans)),
                rmse_px=float(np.median(rmse)),
                trials=trials,
            ))
    return rows


def views_sweep(trials: int = 200, views_list: Sequence[int] = (1, 2, 3, 4, 6),
                pixel_noise: float = 1.0, range_noise: float = 0.01,
                angle_noise_deg: float = 0.1) -> List[dict]:
    """标定板摆位次数扫描：回答"需要摆几次板"。"""
    rows = []
    for n in views_list:
        rot, trans, rmse = [], [], []
        for seed in range(trials):
            res = calibrate(n, pixel_noise, range_noise, angle_noise_deg, seed)
            rot.append(res.rot_error_deg)
            trans.append(res.trans_error_m)
            rmse.append(res.rmse_px)
        rows.append(dict(
            views=n, n_points=n * BOARD_COLS * BOARD_ROWS,
            rot_error_deg=float(np.median(rot)),
            trans_error_m=float(np.median(trans)),
            rmse_px=float(np.median(rmse)),
            trials=trials,
        ))
    return rows


def render_synthetic_image(points: np.ndarray, poses: Sequence[Tuple[np.ndarray, np.ndarray]],
                           R: np.ndarray, t: np.ndarray,
                           K: np.ndarray = DEFAULT_K, D: np.ndarray = DEFAULT_DIST,
                           size: Tuple[int, int] = (480, 640)) -> np.ndarray:
    """按给定外参把标定板画成"相机看到的图像"（用于投影叠加验证）。

    Args:
        points: 标定板角点（LiDAR 坐标系）
        poses:  每个摆位的 (R_board, t_board)
        R, t:   绘制时使用的外参
        size:   (height, width)
    """
    img = np.full((size[0], size[1], 3), 255, dtype=np.uint8)
    cells_local = board_cells()
    n_cols = BOARD_COLS - 1
    for R_b, t_b in poses:
        cells = (R_b @ cells_local.reshape(-1, 3).T).T.reshape(-1, 4, 3) + t_b
        quad = project(cells.reshape(-1, 3), R, t, K, D).reshape(-1, 4, 2)
        for idx, q in enumerate(quad):
            if not np.all(np.isfinite(q)):
                continue
            # 超出画面的单元格跳过，避免 OpenCV 绘制异常
            if q.max() > max(size) * 1.2 or q.min() < -max(size) * 0.2:
                continue
            # 棋盘格：按 (行 + 列) 的奇偶交替，而不是按线性序号（那是条纹）
            row, col = divmod(idx, n_cols)
            color = (40, 40, 40) if (row + col) % 2 == 0 else (205, 205, 205)
            cv2.fillConvexPoly(img, np.round(q).astype(np.int32), color)
    return img


def overlay_projection(image: np.ndarray, points_lidar: np.ndarray,
                       R_est: np.ndarray, t_est: np.ndarray,
                       R_true: np.ndarray = None, t_true: np.ndarray = None,
                       K: np.ndarray = DEFAULT_K,
                       D: np.ndarray = DEFAULT_DIST) -> np.ndarray:
    """在图像上叠加"真值投影（绿圈）"与"估计外参投影（红点）"。"""
    R_true = default_R() if R_true is None else R_true
    t_true = DEFAULT_T if t_true is None else t_true
    out = image.copy()

    uv_true = project(points_lidar, R_true, t_true, K, D)
    uv_est = project(points_lidar, R_est, t_est, K, D)

    for u, v in uv_true:
        if np.isfinite(u) and np.isfinite(v):
            cv2.circle(out, (int(round(u)), int(round(v))), 5, (0, 180, 0), 1)
    for u, v in uv_est:
        if np.isfinite(u) and np.isfinite(v):
            cv2.circle(out, (int(round(u)), int(round(v))), 2, (0, 0, 255), -1)

    cv2.putText(out, 'green: ground truth   red: estimated extrinsics',
                (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
    return out


def matrix_to_text(R: np.ndarray, t: np.ndarray) -> str:
    """把外参整理成一行 JSON 风格的文本（ROS 话题发布用）。"""
    rvec = cv2.Rodrigues(R)[0].reshape(-1)
    return ('{"rvec":[%.6f,%.6f,%.6f],"t":[%.6f,%.6f,%.6f],"frame":"lidar->camera"}'
            % (rvec[0], rvec[1], rvec[2], t[0], t[1], t[2]))
