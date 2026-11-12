"""ROV 六自由度推力分配的纯算法实现（无 ROS 依赖）。

推力器布局：4 个垂推（heave）、2 个纵向（surge）、2 个横向（sway）。
分配矩阵 T (6xN) 把每个推力器的单位推力映射到 [surge, sway, heave, roll, pitch, yaw]。
"""

import math

THRUSTERS = [
    (0.30, 0.25, 0.0, 0.0, 0.0, 1.0),
    (0.30, -0.25, 0.0, 0.0, 0.0, 1.0),
    (-0.30, 0.25, 0.0, 0.0, 0.0, 1.0),
    (-0.30, -0.25, 0.0, 0.0, 0.0, 1.0),
    (0.35, 0.25, 0.0, 1.0, 0.0, 0.0),
    (0.35, -0.25, 0.0, 1.0, 0.0, 0.0),
    (0.30, 0.0, 0.0, 0.0, 1.0, 0.0),
    (-0.30, 0.0, 0.0, 0.0, 1.0, 0.0),
]
THRUST_LIMIT = 10.0
DOF_NAMES = ["surge", "sway", "heave", "roll", "pitch", "yaw"]


def cross(r, d):
    return (r[1] * d[2] - r[2] * d[1],
            r[2] * d[0] - r[0] * d[2],
            r[0] * d[1] - r[1] * d[0])


def build_tam(thrusters):
    """构造 6xN 分配矩阵，列为 [推力方向; r x 推力方向]。"""
    cols = []
    for (x, y, z, dx, dy, dz) in thrusters:
        torque = cross((x, y, z), (dx, dy, dz))
        cols.append([dx, dy, dz, torque[0], torque[1], torque[2]])
    return [[cols[j][i] for j in range(len(cols))] for i in range(6)]


def _solve(a, b):
    """高斯消元解 a x = b（a 为 n x n）。"""
    n = len(a)
    m = [row[:] + [b[i]] for i, row in enumerate(a)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(m[r][col]))
        if abs(m[pivot][col]) < 1e-12:
            raise ValueError("singular matrix")
        m[col], m[pivot] = m[pivot], m[col]
        pv = m[col][col]
        for j in range(col, n + 1):
            m[col][j] /= pv
        for r in range(n):
            if r == col:
                continue
            factor = m[r][col]
            if abs(factor) < 1e-15:
                continue
            for j in range(col, n + 1):
                m[r][j] -= factor * m[col][j]
    return [m[i][n] for i in range(n)]


def allocate(wrench, tam, limit=THRUST_LIMIT, damping=1e-6):
    """阻尼最小二乘分配 + 整体缩放饱和。

    推力器数量 N > 6 时 T^T T 奇异，改用 (T^T T + lambda I) x = T^T w，
    得到最小范数解；若最大推力超过 limit，则整体按比例缩放。
    """
    n = len(tam[0])
    ata = [[0.0] * n for _ in range(n)]
    atb = [0.0] * n
    for i in range(6):
        for j in range(n):
            atb[j] += tam[i][j] * wrench[i]
            for k in range(n):
                ata[j][k] += tam[i][j] * tam[i][k]
    for j in range(n):
        ata[j][j] += damping
    try:
        thrust = _solve(ata, atb)
    except ValueError:
        thrust = [0.0] * n

    peak = max(abs(t) for t in thrust) if thrust else 0.0
    saturation = peak > limit and peak > 0.0
    scale = limit / peak if saturation else 1.0
    thrust = [t * scale for t in thrust]
    achieved = [sum(tam[i][j] * thrust[j] for j in range(n)) for i in range(6)]
    utilization = min(1.0, peak / limit) if limit > 0 else 0.0
    return thrust, achieved, saturation, utilization


def fault_mask(tam, failed):
    """把失效推力器对应的列置零。"""
    out = [row[:] for row in tam]
    for j in failed:
        for i in range(6):
            out[i][j] = 0.0
    return out


def wrench_norm(w):
    return math.sqrt(sum(v * v for v in w))
