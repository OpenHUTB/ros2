"""与 ROS 解耦的四旋翼推力分配模型与单桨失效可行性分析.

坐标系按 REP-103: 机体 x 指机头, y 指左侧, z 向上, 旋翼推力沿 +z.
旋翼 i 位于 (L cos φ_i, L sin φ_i), φ = (45°, 135°, 225°, 315°), 即 X 型布局;
旋翼 0/2 与 1/3 分别同向旋转, 反扭矩记为 τz_i = σ_i κ f_i, σ = (-1, +1, -1, +1).
分配矩阵 A 满足 w = A f, w = [T, τx, τy, τz]ᵀ, f = [f0, f1, f2, f3]ᵀ:

    A = [[ 1,  1,  1,  1],
         [ y0, y1, y2, y3],
         [-x0,-x1,-x2,-x3],
         [ σ0·κ, σ1·κ, σ2·κ, σ3·κ]]

这是静态模型: 只描述推力到总推力/力矩的映射和单桨推力上限, 不含电机动态、桨叶
挥舞、气动干扰与姿态环带宽. 结论用于选型与余量评估, 不代替飞控或整机仿真.
"""
import math

import numpy as np


class QuadrotorAllocator:
    """X 型四旋翼的推力分配、偏航权限与失效后可达力矩集合."""

    ROTOR_ANGLES_DEG = (45.0, 135.0, 225.0, 315.0)
    SPIN_SIGNS = (-1.0, 1.0, -1.0, 1.0)

    def __init__(self, mass=2.0, arm_length=0.25, torque_ratio=0.016, max_thrust=None,
                 gravity=9.80665):
        """建立几何与分配矩阵.

        Args:
            mass: 整机质量 (kg).
            arm_length: 机身中心到旋翼的力臂 (m).
            torque_ratio: 反扭矩系数 κ = 反扭矩 / 推力 (m).
            max_thrust: 单桨最大推力 (N); None 时取 T/W = 2 对应的 mg/2.
            gravity: 重力加速度 (m/s²).
        """
        values = (mass, arm_length, torque_ratio, gravity)
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError('质量、力臂、反扭矩系数与重力必须为正的有限数值')
        if max_thrust is None:
            max_thrust = mass * gravity / 2.0
        if not math.isfinite(max_thrust) or max_thrust <= 0:
            raise ValueError('单桨最大推力必须为正的有限数值')

        self.mass = float(mass)
        self.gravity = float(gravity)
        self.arm_length = float(arm_length)
        self.torque_ratio = float(torque_ratio)
        self.max_thrust = float(max_thrust)

        angles = np.radians(np.array(self.ROTOR_ANGLES_DEG))
        positions = np.stack([self.arm_length * np.cos(angles),
                              self.arm_length * np.sin(angles)], axis=1)
        positions.setflags(write=False)
        self.positions = positions

        matrix = np.zeros((4, 4))
        matrix[0] = 1.0
        matrix[1] = positions[:, 1]
        matrix[2] = -positions[:, 0]
        matrix[3] = np.array(self.SPIN_SIGNS) * self.torque_ratio
        matrix.setflags(write=False)
        self._matrix = matrix
        self._inverse = np.linalg.inv(matrix)

    # ------------------------------------------------------------------ 基本量

    @property
    def hover_thrust(self):
        """悬停所需总推力 mg (N)."""
        return self.mass * self.gravity

    @property
    def thrust_to_weight(self):
        """推重比 T/W = 4·f_max / (mg)."""
        return 4.0 * self.max_thrust / self.hover_thrust

    @property
    def allocation_matrix(self):
        """分配矩阵 A 的副本."""
        return self._matrix.copy()

    def wrench(self, thrusts):
        """由四桨推力计算 [T, τx, τy, τz]."""
        thrusts = np.asarray(thrusts, dtype=float)
        if thrusts.shape != (4,):
            raise ValueError('推力向量长度必须为 4')
        return self._matrix @ thrusts

    def allocate(self, wrench):
        """四桨完好时的分配: 返回 (四桨推力, 是否满足推力上下限)."""
        wrench = np.asarray(wrench, dtype=float)
        if wrench.shape != (4,):
            raise ValueError('期望 wrench 长度必须为 4')
        thrusts = self._inverse @ wrench
        return thrusts, self._within_limits(thrusts)

    def hover_allocation(self):
        """悬停分配; 悬停时四桨对称出力, 每桨 mg/4."""
        return self.allocate([self.hover_thrust, 0.0, 0.0, 0.0])

    # ------------------------------------------------------------ 单桨失效

    def allocate_with_failure(self, wrench, failed_rotor):
        """单桨失效后优先保证总推力与滚转/俯仰力矩, 偏航力矩由剩余桨被动决定.

        一个旋翼失效后只剩 3 个执行器, 无法同时满足 4 个 wrench 分量, 因此放弃
        偏航力矩控制, 用剩余三桨精确满足 T、τx、τy, 再把产生的 τz 作为"不可控
        偏航"返回. 返回 (四桨推力, 是否可行, 实际达成的 wrench).
        """
        remaining = self._remaining_rotors(failed_rotor)
        thrusts = np.zeros(4)
        # 只保留 T、τx、τy 三行, 与剩余三列组成 3x3 方可解.
        reduced = self._matrix[:3][:, remaining]
        try:
            solved = np.linalg.solve(reduced, np.asarray(wrench, dtype=float)[:3])
        except np.linalg.LinAlgError:  # pragma: no cover - 几何退化时才会触发
            return thrusts, False, np.zeros(4)
        thrusts[remaining] = solved
        achieved = self._matrix @ thrusts
        return thrusts, self._within_limits(thrusts), achieved

    def forced_yaw_after_failure(self, failed_rotor, total_thrust=None):
        """单桨失效后、用剩余桨维持 (T, τx=0, τy=0) 时被动的偏航力矩 (N·m).

        取 total_thrust 为悬停推力 mg 时, 该值恒等于 κ·mg: 剩余两桨必在一条对角线
        上, 各出 mg/2, 反扭矩同向叠加. 符号取决于失效的是哪一个桨.
        """
        total = self.hover_thrust if total_thrust is None else float(total_thrust)
        _, _, achieved = self.allocate_with_failure([total, 0.0, 0.0, 0.0], failed_rotor)
        return achieved[3]

    def max_thrust_zero_moment(self, failed_rotor=None):
        """不产生滚转/俯仰力矩时能达到的最大总推力 (N).

        完好时四桨同时出力即可, 上限 4·f_max. 单桨失效后只剩一条对角线两桨有效,
        上限为 2·f_max, 即恰好是完好时的一半; 于是"单桨失效后仍能保持零力矩悬停"
        的条件是 2·f_max ≥ mg, 也就是 T/W ≥ 2.
        """
        if failed_rotor is None:
            return 4.0 * self.max_thrust
        remaining = self._remaining_rotors(failed_rotor)
        # τx=0、τy=0 是 2 个约束、3 个未知数, 解集是一条直线 f = t·n.
        rows = self._matrix[1:3][:, remaining]
        singular = np.linalg.svd(rows)[2]
        direction = singular[-1]
        length = np.linalg.norm(direction)
        if length < 1e-12:  # pragma: no cover - 几何退化
            return 0.0
        direction = direction / length
        low, high = -np.inf, np.inf
        for coefficient in direction:
            if abs(coefficient) < 1e-12:
                continue  # 该桨在零力矩解里恒为 0, 只要 0 ≤ f ≤ f_max 就一定可行
            first = 0.0 / coefficient
            second = self.max_thrust / coefficient
            low = max(low, min(first, second))
            high = min(high, max(first, second))
        if low > high:
            return 0.0
        total_coefficient = float(np.sum(direction))
        candidates = [low * total_coefficient, high * total_coefficient]
        return max(0.0, max(candidates))

    # ------------------------------------------------------------ 力矩集合

    def max_yaw_torque(self, total_thrust=None):
        """只约束总推力与单桨推力上限时能产生的最大偏航力矩 (N·m).

        两组旋转方向各有两桨: 把尽量多的推力放在其中一组上. 记 T 为总推力,
        c = 2·f_max 为一组两桨的推力上限, 则

            τz_max(T) = κ·(2·min(c, T) − T),  0 ≤ T ≤ 4·f_max.
        """
        total = self.hover_thrust if total_thrust is None else float(total_thrust)
        if not math.isfinite(total) or total < 0 or total > 4.0 * self.max_thrust + 1e-9:
            raise ValueError('总推力必须落在 [0, 4·f_max] 内')
        pair_cap = 2.0 * self.max_thrust
        return self.torque_ratio * (2.0 * min(pair_cap, total) - total)

    def feasible_polygon(self, total_thrust=None, yaw=0.0, failed_rotor=None):
        """固定总推力与偏航力矩时, 可达的 (τx, τy) 集合的顶点.

        完好时把 wrench 写成 (τx, τy) 的仿射函数, 单桨推力上下限给出若干半平面,
        顶点即这些半平面边界的交点. 返回按逆时针排序的 (n, 2) 数组; 集合退化时
        可能只有 1 个点, 无解时返回空数组.
        """
        total = self.hover_thrust if total_thrust is None else float(total_thrust)
        if failed_rotor is None:
            offset = self._inverse @ np.array([total, 0.0, 0.0, float(yaw)])
            slopes = self._inverse[:, 1:3]
            count = 4
        else:
            remaining = self._remaining_rotors(failed_rotor)
            reduced = np.linalg.inv(self._matrix[:3][:, remaining])
            offset = reduced @ np.array([total, 0.0, 0.0])
            slopes = reduced[:, 1:3]
            count = 3

        planes = []
        for index in range(count):
            slope = slopes[index]
            planes.append((-slope, offset[index]))
            planes.append((slope, self.max_thrust - offset[index]))
        return _intersect_half_planes(planes)

    @staticmethod
    def polygon_area(polygon):
        """鞋带公式计算多边形面积; 顶点少于 3 个时为 0."""
        polygon = np.asarray(polygon, dtype=float)
        if polygon.shape[0] < 3:
            return 0.0
        x, y = polygon[:, 0], polygon[:, 1]
        return 0.5 * abs(float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))

    @staticmethod
    def polygon_extents(polygon):
        """返回多边形在 τx、τy 方向上的最大绝对值 (对称布局时可视为滚转/俯仰权限)."""
        polygon = np.asarray(polygon, dtype=float)
        if polygon.size == 0:
            return 0.0, 0.0
        return float(np.max(np.abs(polygon[:, 0]))), float(np.max(np.abs(polygon[:, 1])))

    @staticmethod
    def polygon_contains(polygon, point, tolerance=1e-9):
        """判断点是否落在凸多边形内 (逆时针顶点, 叉积同号)."""
        polygon = np.asarray(polygon, dtype=float)
        if polygon.shape[0] < 3:
            return False
        point = np.asarray(point, dtype=float)
        scale = max(1.0, float(np.max(np.abs(polygon))))
        sign = 0
        for index in range(polygon.shape[0]):
            start = polygon[index]
            end = polygon[(index + 1) % polygon.shape[0]]
            cross = ((end[0] - start[0]) * (point[1] - start[1])
                     - (end[1] - start[1]) * (point[0] - start[0]))
            if abs(cross) <= tolerance * scale:
                continue
            current = 1 if cross > 0 else -1
            if sign == 0:
                sign = current
            elif current != sign:
                return False
        return True

    # ------------------------------------------------------------------ 内部

    def _remaining_rotors(self, failed_rotor):
        if failed_rotor is None:
            raise ValueError('failed_rotor 不能为空')
        if not isinstance(failed_rotor, (int, np.integer)) or not 0 <= failed_rotor < 4:
            raise ValueError('failed_rotor 必须是 0..3 的整数')
        return [i for i in range(4) if i != int(failed_rotor)]

    def _within_limits(self, thrusts, tolerance=1e-9):
        return bool(np.all(thrusts >= -tolerance)
                    and np.all(thrusts <= self.max_thrust + tolerance))


def _intersect_half_planes(planes, tolerance=1e-9):
    """求半平面 a·u ≤ c 交集的多边形顶点, 按逆时针排序.

    半平面数量很少 (最多 8 个), 直接枚举边界两两求交再筛可行点即可, 不需要 LP 库.
    """
    points = []
    scale = max(1e-9, max(abs(c) for _, c in planes))
    tolerance = tolerance * scale
    for i in range(len(planes)):
        a1, c1 = planes[i]
        for j in range(i + 1, len(planes)):
            a2, c2 = planes[j]
            determinant = a1[0] * a2[1] - a1[1] * a2[0]
            if abs(determinant) < 1e-12:
                continue
            point = np.array([(c1 * a2[1] - c2 * a1[1]) / determinant,
                              (a1[0] * c2 - a2[0] * c1) / determinant])
            if all(a[0] * point[0] + a[1] * point[1] <= c + tolerance for a, c in planes):
                points.append(point)
    if not points:
        return np.zeros((0, 2))
    unique = []
    for point in points:
        if not any(np.linalg.norm(point - other) <= 1e-6 * max(1.0, scale) for other in unique):
            unique.append(point)
    polygon = np.array(unique)
    if polygon.shape[0] < 3:
        return polygon
    center = polygon.mean(axis=0)
    order = np.argsort(np.arctan2(polygon[:, 1] - center[1], polygon[:, 0] - center[0]))
    return polygon[order]
