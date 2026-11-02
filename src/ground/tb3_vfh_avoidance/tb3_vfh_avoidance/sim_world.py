"""离线仿真环境：二维迷宫 + 射线投射激光雷达 + 差速小车运动学。

用途：在没有安装 Gazebo 的机器上，验证 :mod:`vfh_core` 的避障逻辑是否真的
能把小车从起点开到终点。仿真本身是确定性的（无噪声），因此同一份代码在
任何平台跑出的轨迹完全一致，便于复现与对比。

坐标系：世界坐标系为平面直角坐标，``x`` 向右、``y`` 向上，角度逆时针为正，
``theta = 0`` 指向 ``x`` 正方向。
"""

import math

TWO_PI = 2.0 * math.pi


def normalize_angle(angle):
    """把角度规范到 ``[-pi, pi)``。"""
    return (angle + math.pi) % TWO_PI - math.pi


class World:
    """由线段（墙）与矩形（障碍箱）构成的二维世界。"""

    def __init__(self):
        self.segments = []

    # ------------------------------------------------------------------
    # 构建
    # ------------------------------------------------------------------
    def add_wall(self, x0, y0, x1, y1):
        """添加一段墙（线段）。"""
        self.segments.append(((float(x0), float(y0)), (float(x1), float(y1))))

    def add_box(self, x0, y0, x1, y1):
        """添加一个矩形障碍（四条边）。"""
        self.add_wall(x0, y0, x1, y0)
        self.add_wall(x1, y0, x1, y1)
        self.add_wall(x1, y1, x0, y1)
        self.add_wall(x0, y1, x0, y0)

    def add_room(self, side=8.0):
        """添加一个正方形房间的四壁，房间范围为 ``[0, side] × [0, side]``。"""
        self.add_box(0.0, 0.0, side, side)

    # ------------------------------------------------------------------
    # 感知
    # ------------------------------------------------------------------
    def ray_cast(self, ox, oy, angle, max_range=3.5):
        """投射一条射线，返回最近命中距离。

        :param ox: 射线起点 x
        :param oy: 射线起点 y
        :param angle: 射线方向（弧度）
        :param max_range: 量程，超出则返回 ``max_range``
        :return: 命中距离（米），未命中返回 ``max_range``
        """
        dx = math.cos(angle)
        dy = math.sin(angle)
        best = max_range

        for (ax, ay), (bx, by) in self.segments:
            sx = bx - ax
            sy = by - ay
            denominator = dx * sy - dy * sx
            if abs(denominator) < 1e-12:      # 射线与线段平行
                continue
            ax_ = ax - ox
            ay_ = ay - oy
            t = (ax_ * sy - ay_ * sx) / denominator      # 射线参数
            u = (ax_ * dy - ay_ * dx) / denominator      # 线段参数
            if t >= 0.0 and 0.0 <= u <= 1.0 and t < best:
                best = t
        return best

    def scan(self, x, y, theta, n_rays=360, max_range=3.5, angle_min=0.0):
        """模拟一圈激光扫描。

        默认参数对应 TurtleBot3 的 LDS-01：``angle_min = 0``、
        360 条射线、量程 3.5 m。**因此 ``ranges[0]`` 就是正前方**。

        :param x: 机器人 x
        :param y: 机器人 y
        :param theta: 机器人朝向（弧度）
        :param n_rays: 射线数量
        :param max_range: 量程
        :param angle_min: 第 0 条射线的角度（机器人坐标系）
        :return: ``(ranges, angle_min, angle_increment)``
        """
        increment = TWO_PI / n_rays
        ranges = []
        for index in range(n_rays):
            angle = theta + angle_min + index * increment
            ranges.append(self.ray_cast(x, y, angle, max_range=max_range))
        return ranges, angle_min, increment


class DiffDriveRobot:
    """差速轮小车（单轨模型）。"""

    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x = float(x)
        self.y = float(y)
        self.theta = normalize_angle(theta)
        self.path = [(self.x, self.y)]

    def step(self, linear, angular, dt=0.05):
        """按速度指令推进一个时间步（中点积分，比欧拉法更接近真实轨迹）。"""
        theta_mid = self.theta + angular * dt * 0.5
        self.x += linear * math.cos(theta_mid) * dt
        self.y += linear * math.sin(theta_mid) * dt
        self.theta = normalize_angle(self.theta + angular * dt)
        self.path.append((self.x, self.y))
        return self.x, self.y, self.theta


def make_maze():
    """构建一个带 U 形陷阱的测试迷宫。

    场景说明：房间 8 m × 8 m，U 形障碍**开口朝下**，正对起点到终点的直线路径。
    小车冲进 U 形内部后前向被堵死，只能退出再绕行——这正是"局部极小值"陷阱：
    朴素的三扇区策略会顶在尽头左右摇摆，而 VFH 的谷值搜索能找到唯一的出口方向。
    """
    world = World()
    world.add_room(8.0)

    # U 形陷阱（开口朝 -y 方向，即朝向起点）
    world.add_box(3.0, 2.2, 3.2, 4.5)
    world.add_box(5.0, 2.2, 5.2, 4.5)
    world.add_box(3.0, 4.3, 5.2, 4.5)

    # 两处散布障碍，模拟走廊里的箱子（刻意避开终点附近，保证终点可达）
    world.add_box(1.5, 5.6, 2.3, 6.4)
    world.add_box(5.4, 2.2, 6.2, 3.0)

    return world
