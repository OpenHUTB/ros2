"""VFH（Vector Field Histogram，向量场直方图）局部避障核心算法。

本文件是纯 Python 实现，**不依赖 ROS**，因此既可以被 ROS 2 节点调用，
也可以被 ``tools/offline_demo.py`` 的离线闭环仿真直接复用，还能被 pytest
单独测试。算法流程（Borenstein & Koren, 1991；本实现按工程可解释性做了简化）：

1. **极坐标投影**：把一帧 ``LaserScan`` 的每个有效回波按
   ``angle_min + i * angle_increment`` 投影到 ``n_sectors`` 个角度栅格中，
   记录每格的最短距离，同时按 ``(d_max - d)^2`` 累加障碍密度（仅用于诊断
   与并列时的择优，不参与阻塞判决）。
2. **阻塞判决**：某扇区的**净空**（最短距离减去机器人半径）小于
   ``block_distance`` 即判为阻塞。这里刻意用"距离"而不是"归一化密度"
   作为阈值——若按密度峰值归一化，当身边有一面 0.3 m 的墙时，2 m 外的
   另一面墙也会被算成"阻塞"，可用方向被误杀，车会在空地上打转。
3. **谷值搜索**：把连续的自由扇区合并成"谷"，谷宽代表可通行走廊的角度
   宽度；不足 ``min_gap_deg`` 的窄缝直接丢弃（机器人过不去）。
4. **方向选择**：在合格谷中选"最靠近目标方位"且"尽量朝前"的扇区，
   避免选出背后的空旷方向。
5. **速度规划**：线速度按航向差的余弦投影衰减（偏 90° 归零），
   净空越小越慢；角速度比例控制并限幅到平台最大角速度。
6. **安全停障与死锁恢复**：选定方向净空低于 ``safety_distance`` 时只转不走；
   连续多帧既不走也不转则原地旋转搜索，跳出局部极小值。

角度约定
--------
所有角度单位为弧度，机器人坐标系下 **0 表示正前方、逆时针为正**。
TurtleBot3 的 LDS-01 雷达 ``angle_min = 0.0``，即 ``ranges[0]`` 就是正前方；
本模块按 ``angle_min + i * angle_increment`` 计算每个回波的角度，
不假设"数组中间就是正前方"（这是常见实现错误，会导致车对着正后方的墙
做决策）。
"""

import math

TWO_PI = 2.0 * math.pi


def normalize_angle(angle):
    """把角度规范到 ``[-pi, pi)`` 区间。

    :param angle: 任意弧度值
    :return: 规范化后的弧度值
    """
    return (angle + math.pi) % TWO_PI - math.pi


def polar_points(ranges, angle_min, angle_increment,
                 range_min=0.0, range_max=3.5):
    """把一帧扫描数据转换为有效极坐标点列表。

    过滤 ``nan`` / ``inf`` / 非正距离 / 超出量程的回波。

    :param ranges: 距离序列（``LaserScan.ranges``）
    :param angle_min: 第 0 个回波的角度（``LaserScan.angle_min``）
    :param angle_increment: 相邻回波角度增量
    :param range_min: 有效最小距离，单位米
    :param range_max: 有效最大距离，单位米
    :return: ``[(角度, 距离), ...]``
    """
    points = []
    for index, distance in enumerate(ranges):
        if distance != distance:                      # nan
            continue
        if math.isinf(distance) or distance <= 0.0:
            continue
        if distance < range_min or distance > range_max:
            continue
        points.append((angle_min + index * angle_increment, distance))
    return points


class VfhCommand:
    """一帧决策的输出结果。"""

    __slots__ = ('linear', 'angular', 'state', 'heading', 'clearance',
                 'valley_width', 'density')

    def __init__(self, linear=0.0, angular=0.0, state='cruise', heading=0.0,
                 clearance=float('inf'), valley_width=0, density=0.0):
        self.linear = linear
        self.angular = angular
        self.state = state
        self.heading = heading
        self.clearance = clearance
        self.valley_width = valley_width
        self.density = density

    def __repr__(self):
        return ('VfhCommand(linear=%.3f, angular=%.3f, state=%s, '
                'heading=%.1f deg, clearance=%.2f m, valley=%d)'
                % (self.linear, self.angular, self.state,
                   math.degrees(self.heading), self.clearance,
                   self.valley_width))


class VfhPlanner:
    """VFH 局部避障规划器（输入单帧扫描，内部维护航向与死锁状态）。"""

    def __init__(self, n_sectors=72, max_range=3.5, block_distance=0.50,
                 min_gap_deg=25.0, robot_radius=0.22, safety_distance=0.40,
                 max_linear=0.22, max_angular=1.82, turn_gain=1.6,
                 smoothing=0.55, deadlock_frames=20, recovery_frames=15,
                 clearance_half_deg=20.0, body_margin=0.06,
                 block_range=2.0):
        """
        :param n_sectors: 极坐标直方图扇区数，默认 72（每扇区 5°）
        :param max_range: 雷达有效量程，单位米
        :param block_distance: 阻塞判决阈值（净空，单位米）
        :param min_gap_deg: 可通行谷的最小角宽（度）
        :param robot_radius: 机器人等效半径，用于净空计算，单位米
        :param safety_distance: 选定方向净空低于该值时只转不走，单位米
        :param max_linear: 最大线速度，单位米/秒（Waffle 约 0.26）
        :param max_angular: 最大角速度，单位弧度/秒（Waffle 约 1.82）
        :param turn_gain: 航向误差到角速度的比例增益
        :param smoothing: 航向一阶滞后系数，抑制等宽谷之间的来回切换
        :param deadlock_frames: 连续多少帧"不走也不转"判定为死锁
        :param recovery_frames: 死锁恢复时原地旋转的帧数
        :param clearance_half_deg: 净空判定的半窗角（度）
        :param body_margin: 车身膨胀的额外余量，单位米
        :param block_range: 只考虑该前向距离内的障碍，单位米
        """
        if n_sectors < 8:
            raise ValueError('n_sectors 至少为 8')
        if block_distance <= 0.0:
            raise ValueError('block_distance 必须为正')

        self.n_sectors = int(n_sectors)
        self.max_range = float(max_range)
        self.block_distance = float(block_distance)
        self.min_gap_sectors = max(
            1, int(math.ceil(math.radians(min_gap_deg) / (TWO_PI / n_sectors))))
        self.robot_radius = float(robot_radius)
        self.safety_distance = float(safety_distance)
        self.max_linear = float(max_linear)
        self.max_angular = float(max_angular)
        self.turn_gain = float(turn_gain)
        self.smoothing = float(smoothing)
        self.deadlock_frames = int(deadlock_frames)
        self.recovery_frames = int(recovery_frames)
        self.clearance_half_deg = float(clearance_half_deg)
        self.body_margin = float(body_margin)
        self.block_range = float(block_range)

        self.sector_width = TWO_PI / self.n_sectors
        self._stuck_count = 0
        self._recovery_left = 0
        self._recovery_sign = 1.0
        self._last_heading = None

    # ------------------------------------------------------------------
    # 1~2. 极坐标投影与阻塞判决
    # ------------------------------------------------------------------
    def build_histogram(self, points):
        """由极坐标点构建障碍密度直方图与每扇区最短距离。

        :param points: ``[(角度, 距离), ...]``
        :return: ``(hist, sector_min)``；``hist`` 为按峰值归一化的密度
                 （仅用于诊断/并列择优），``sector_min`` 为每扇区最短距离
        """
        n = self.n_sectors
        raw = [0.0] * n
        sector_min = [float('inf')] * n

        for angle, distance in points:
            index = int((angle % TWO_PI) / self.sector_width) % n
            if distance < sector_min[index]:
                sector_min[index] = distance
            if distance < self.max_range:
                raw[index] += (self.max_range - distance) ** 2

        peak = max(raw) if raw else 0.0
        if peak > 0.0:
            hist = [value / peak for value in raw]
        else:
            hist = raw
        return hist, sector_min

    def blocked_sectors(self, sector_min):
        """按"车身能否沿该方向通过"判定每个扇区是否阻塞。

        这是 VFH 里"障碍物放大（enlargement）"步骤的正确做法：把机器人当成
        一个半径为 ``robot_radius`` 的圆，对每个候选方向 θ，若存在障碍点
        （角度 a、距离 d）满足

        - 在 θ 的前方半平面内（``|a - θ| < 90°``），且
        - 该障碍到 θ 这条射线的**侧向偏移** ``d · sin|a - θ|`` 小于车身半径
          加余量，且
        - 前向距离 ``d · cos(a - θ)`` 在 ``block_range`` 以内（远处障碍不
          影响当下决策，否则一路直路也会被判"阻塞"）

        则该方向被阻塞。只按"该方向上的最近距离"判定是不够的——侧向擦碰
        根本不出现在那条射线上（实测绕外凸角时车从 0.056 m 处擦过并穿模）。

        :param sector_min: 每扇区最短距离
        :return: 布尔列表，``True`` 表示阻塞
        """
        n = self.n_sectors
        limit = self.robot_radius + self.body_margin
        obstacles = [(self._sector_center(index), distance)
                     for index, distance in enumerate(sector_min)
                     if math.isfinite(distance)]

        blocked = [False] * n
        for index in range(n):
            theta = self._sector_center(index)
            for angle, distance in obstacles:
                delta = normalize_angle(angle - theta)
                if abs(delta) >= math.pi / 2.0:        # 障碍在后方，不挡路
                    continue
                if distance * math.cos(delta) > self.block_range:
                    continue                            # 太远，暂不影响
                if distance * math.sin(abs(delta)) < limit:
                    blocked[index] = True
                    break
        return blocked

    def find_valleys(self, sector_min):
        """找出所有连续自由扇区组成的谷。

        :param sector_min: 每扇区最短距离
        :return: ``[(起始扇区, 宽度扇区数), ...]``
        """
        n = self.n_sectors
        blocked = self.blocked_sectors(sector_min)
        free = [not flag for flag in blocked]
        if all(free):
            return [(0, n)]
        if not any(free):
            return []

        # 从某个阻塞扇区之后开始扫描，避免一个跨 0 度的谷被切成两段
        start_offset = 0
        for index in range(n):
            if blocked[index]:
                start_offset = (index + 1) % n
                break

        valleys = []
        run_start = None
        for step in range(n):
            index = (start_offset + step) % n
            if free[index]:
                if run_start is None:
                    run_start = index
            elif run_start is not None:
                valleys.append((run_start, (index - run_start) % n))
                run_start = None
        if run_start is not None:
            valleys.append((run_start, (start_offset - run_start) % n))
        return valleys

    def _sector_center(self, sector):
        """扇区中心角度（机器人坐标系）。"""
        return normalize_angle((sector + 0.5) * self.sector_width)

    # ------------------------------------------------------------------
    # 3~4. 方向选择
    # ------------------------------------------------------------------
    def choose_heading(self, valleys, target_angle=0.0):
        """在候选谷中选择行进方向。

        代价 = |扇区方向 - 目标方位| + 0.25 × |扇区方向|
        第二项是对"偏离正前方"的惩罚，避免选出机器人背后的空旷方向
        （那会让车掉头而不是绕过障碍）。

        :param valleys: :meth:`find_valleys` 的结果
        :param target_angle: 目标方位（机器人坐标系，正前方为 0）
        :return: ``(heading, valley_width)``；无可行方向返回 ``(None, 0)``
        """
        best = None
        for start, width in valleys:
            if width < self.min_gap_sectors:
                continue
            for offset in range(width):
                sector = (start + offset) % self.n_sectors
                angle = self._sector_center(sector)
                cost = (abs(normalize_angle(angle - target_angle))
                        + 0.25 * abs(angle))
                if best is None or cost < best[0]:
                    best = (cost, angle, width)
        if best is None:
            return None, 0
        return best[1], best[2]

    def _clearance_in(self, sector_min, start, width):
        """指定扇区范围内的最短距离（无回波返回量程）。"""
        values = [sector_min[(start + offset) % self.n_sectors]
                  for offset in range(width)]
        valid = [value for value in values if math.isfinite(value)]
        return min(valid) if valid else self.max_range

    def _window_clearance(self, sector_min, center_angle, half_width=None):
        """以 ``center_angle`` 为中心、``±half_width`` 范围内的最短距离。

        :param center_angle: 窗口中心角（机器人坐标系）
        :param half_width: 半窗宽（弧度），默认 10°
        """
        if half_width is None:
            half_width = math.radians(10.0)
        half = max(1, int(half_width / self.sector_width))
        center = int(center_angle / self.sector_width + 0.5) % self.n_sectors
        return self._clearance_in(sector_min,
                                  (center - half) % self.n_sectors,
                                  2 * half + 1)

    # ------------------------------------------------------------------
    # 5~6. 速度规划与死锁恢复
    # ------------------------------------------------------------------
    def compute(self, ranges, angle_min, angle_increment, target_angle=0.0,
                range_min=0.0, debug=None):
        """由一帧扫描数据计算速度指令。

        :param ranges: 距离序列
        :param angle_min: 第 0 个回波角度
        :param angle_increment: 相邻回波角度增量
        :param target_angle: 目标方位（机器人坐标系），默认正前方
        :param range_min: 有效最小距离
        :param debug: 传入 dict 时回填 ``hist`` / ``sector_min`` /
                      ``valleys`` / ``blocked`` 便于可视化
        :return: :class:`VfhCommand`
        """
        points = polar_points(ranges, angle_min, angle_increment,
                              range_min=range_min, range_max=self.max_range)
        hist, sector_min = self.build_histogram(points)
        valleys = self.find_valleys(sector_min)

        if debug is not None:
            debug['hist'] = hist
            debug['sector_min'] = sector_min
            debug['valleys'] = valleys
            debug['blocked'] = self.blocked_sectors(sector_min)
            debug['points'] = points

        # ---------- 死锁恢复优先 ----------
        if self._recovery_left > 0:
            self._recovery_left -= 1
            return VfhCommand(
                linear=0.0,
                angular=self._recovery_sign * self.max_angular * 0.7,
                state='recovery',
                heading=self._recovery_sign * math.pi / 2,
                clearance=self._window_clearance(sector_min, 0.0),
                valley_width=0,
                density=max(hist))

        heading, width = self.choose_heading(valleys, target_angle)

        # ---------- 全阻塞：原地转向密度更低的一侧 ----------
        if heading is None:
            quarter = self.n_sectors // 4
            left = sum(hist[1:quarter + 1])
            right = sum(hist[-quarter:])
            sign = 1.0 if left <= right else -1.0
            self._recovery_sign = sign
            self._recovery_left = self.recovery_frames
            return VfhCommand(
                linear=0.0,
                angular=sign * self.max_angular * 0.6,
                state='blocked',
                heading=sign * math.pi / 2,
                clearance=self._window_clearance(sector_min, 0.0),
                valley_width=0,
                density=max(hist))

        # ---------- 航向一阶滞后 ----------
        # 两个等宽谷的代价可能只差零点几度，逐帧直接取最优会让航向来回
        # 抖动（实测在 U 形角落表现为原地缩着往前蹭）。一阶滞后等价于
        # 给方向选择加惯性，让航向连续变化。
        #
        # 但滞后本身有风险：平滑值可能落在两个谷之间的阻塞扇区里，等于
        # "为了平滑而把车开向墙"（实测贴着 U 形外墙蹭到 0.07 m）。因此
        # 平滑后必须复核净空，不安全就退回本帧的最优方向。
        raw_heading = heading
        if self._last_heading is not None:
            delta = normalize_angle(heading - self._last_heading)
            heading = normalize_angle(self._last_heading
                                      + self.smoothing * delta)
            if (self._window_clearance(
                    sector_min, heading,
                    math.radians(self.clearance_half_deg))
                    < self.safety_distance):
                heading = raw_heading
        self._last_heading = heading

        # ---------- 近距保护 ----------
        # 净空检查只看"选定方向 ±20°"窗口，管不住车身侧向擦碰：实测绕 U 形
        # 外墙外凸角时角点落在窗口外，车以 0.22 m/s 从 0.056 m 处擦过去，
        # 车身直接穿模。这里补一层：若**行驶方向前方半平面内**有障碍比车身
        # 半径更近，就停车并朝远离它的方向转。
        # 注意不能对所有方向生效——原地旋转并不会拉开距离，全覆盖会让车
        # 在离墙 0.28 m 时永久停车（实测 400 s 原地打转）。
        nearest = min(range(self.n_sectors), key=lambda i: sector_min[i])
        if sector_min[nearest] < self.robot_radius + self.body_margin:
            bearing = self._sector_center(nearest)
            if abs(normalize_angle(bearing - heading)) < math.radians(90.0):
                sign = 1.0 if bearing <= 0.0 else -1.0
                self._recovery_sign = sign
                return VfhCommand(
                    linear=0.0,
                    angular=sign * self.max_angular * 0.6,
                    state='guard',
                    heading=bearing,
                    clearance=sector_min[nearest],
                    valley_width=0,
                    density=max(hist))

        # 选定方向 ±12° 内的最短距离 == 沿该方向行驶时的实际通行余量
        clearance = self._window_clearance(sector_min, heading,
                                           math.radians(self.clearance_half_deg))

        if clearance < self.safety_distance:
            # 前方太近：只转不走，先把车头转开
            linear = 0.0
            angular = 2.0 * self.turn_gain * heading
            angular = max(-self.max_angular, min(self.max_angular, angular))
            state = 'blocked'
        elif abs(heading) > math.radians(100.0):
            # 需要大角度掉头：同样先原地转
            linear = 0.0
            angular = 2.0 * self.turn_gain * heading
            angular = max(-self.max_angular, min(self.max_angular, angular))
            state = 'turn'
        else:
            # 线速度按航向差的余弦投影衰减：偏 60° 只剩一半，偏 90° 归零。
            # 若用线性衰减，偏 90° 仍有 0.7 倍速度，实测会一边满舵一边
            # 往前开，绕着目标画圈永远到不了。
            align = max(0.0, math.cos(heading))
            clearance_ratio = ((clearance - self.robot_radius)
                               / (0.5 * self.max_range))
            clearance_ratio = max(0.12, min(1.0, clearance_ratio))
            linear = self.max_linear * align * clearance_ratio
            angular = max(-self.max_angular,
                          min(self.max_angular, self.turn_gain * heading))
            if linear < 0.02 and abs(heading) > math.radians(25.0):
                linear = 0.0        # 还没对准就先转，别用极小速度蹭
            state = 'cruise' if abs(heading) < math.radians(15.0) else 'avoid'

        # ---------- 死锁计数："不走也不转" 才算卡住 ----------
        # 若用 linear < 0.02 判定，则"以 0.026 m/s 龟速蹭墙"永远触发不了
        # 恢复（实测在 U 形角落卡死 500+ 帧）。这里同时要求角速度也很小。
        if linear < 0.03 and abs(angular) < 0.30:
            self._stuck_count += 1
            if self._stuck_count >= self.deadlock_frames:
                self._stuck_count = 0
                self._recovery_sign = 1.0 if heading >= 0.0 else -1.0
                self._recovery_left = self.recovery_frames
        else:
            self._stuck_count = 0

        return VfhCommand(linear=linear, angular=angular, state=state,
                          heading=heading, clearance=clearance,
                          valley_width=width, density=max(hist))

    def reset(self):
        """清空内部航向与死锁状态（换场景或重连仿真时调用）。"""
        self._stuck_count = 0
        self._recovery_left = 0
        self._recovery_sign = 1.0
        self._last_heading = None
