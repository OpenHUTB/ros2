#!/usr/bin/env python3
"""解析式起伏地形 + MuJoCo hfield 写入。

高程函数固定（不随种子变化），因此：
  1. 车辆初始高度可以按解析式精确摆到地面上；
  2. 石头等静态道具可以离线算好高度写进 XML；
  3. 换机器复现画面完全一致。

高程单位：米。均值为 0.25 m，起伏幅度约 ±0.17 m。
"""
import numpy as np

# hfield 允许的最大高程（对应 XML 中 hfield size_z）
HFIELD_SIZE_Z = 0.60
TERRAIN_MEAN = 0.28


def elevation(x, y):
    """地形高程（米）。x/y 可为标量或 ndarray。"""
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    h = (
        TERRAIN_MEAN
        + 0.130 * np.sin(0.55 * x + 0.30) * np.cos(0.42 * y)
        + 0.075 * np.sin(1.05 * x - 0.80 * y + 1.20)
        + 0.040 * np.sin(2.20 * x + 1.60 * y + 0.40)
    )
    return h


def slope(x, y, eps=1e-3):
    """解析梯度，返回 (dh/dx, dh/dy)，用于道具姿态、坡度统计。"""
    dx = (elevation(x + eps, y) - elevation(x - eps, y)) / (2 * eps)
    dy = (elevation(x, y + eps) - elevation(x, y - eps)) / (2 * eps)
    return dx, dy


def fill_hfield(model):
    """把解析地形写进 model.hfield_data（归一化 0~1）。

    注意：MuJoCo 的 hfield_data 是归一化量，真实高程 = data * hfield_size_z，
    直接把米制数值写进去会得到厘米级起伏（地形看起来是平的）。
    """
    nrow = int(model.hfield_nrow[0])
    ncol = int(model.hfield_ncol[0])
    size_x = float(model.hfield_size[0][0])
    size_y = float(model.hfield_size[0][1])
    size_z = float(model.hfield_size[0][2])
    base_z = float(model.hfield_size[0][3])   # MuJoCo 要求 > 0，真实高程 = base_z + data*size_z

    # MuJoCo hfield 索引：row -> y，col -> x；数据按 column-major 存储
    xs = np.linspace(-size_x, size_x, ncol)
    ys = np.linspace(-size_y, size_y, nrow)
    gx, gy = np.meshgrid(xs, ys)          # (nrow, ncol)
    elev = elevation(gx, gy)
    model.hfield_data[:] = np.clip((elev - base_z) / size_z, 0.0, 1.0).ravel(order="C")

    cell = 2.0 * size_x / ncol
    return {
        "nrow": nrow,
        "ncol": ncol,
        "cell_m": cell,
        "elev_min": float(elev.min()),
        "elev_max": float(elev.max()),
        "size_z": size_z,
    }
