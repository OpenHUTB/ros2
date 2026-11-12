"""Stanley 前视路径跟踪控制器（阿克曼转向），纯标准库。

被控对象：运动学自行车模型

    x' = v cos(psi)
    y' = v sin(psi)
    psi' = v tan(delta) / L        （L 为轴距）

Stanley 控制律（Stanford 无人车，Thrun 等, 2006）

    delta = psi_e + arctan(k * e / v)

- ``e``：前轴（此处简化为车体中心）到参考路径的**横向偏差**，正值表示偏左/外。
- ``psi_e``：参考路径切向与车头朝向的**航向偏差**（已做 ±pi 包裹）。
- ``k``：横向增益。第二项在大偏差时提供"打回来"的转向，在高速时自动减弱
  （除以 v），这正是它比固定前视距离的纯跟踪更稳的原因。

与纯跟踪（Pure Pursuit）的区别：Stanley 用"航向误差 + 横向误差"直接算转角，
没有前视距离这个参数；它在低速大偏差时收敛更快，但对航向噪声更敏感。
"""

import math

L_WHEELBASE = 2.5      # 轴距 (m)
MAX_STEER = 0.6        # 前轮最大转角 (rad)


def wrap(a):
    """把角度包裹到 (-pi, pi]。"""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


class CirclePath:
    """半径 R 的圆，逆时针行驶。中心在原点。"""

    def __init__(self, radius=8.0):
        self.R = radius

    def curvature(self, x=None, y=None):
        """圆上任意点的曲率恒为 1/R（逆时针为正）。"""
        return 1.0 / self.R

    def nearest(self, x, y):
        """返回 (最近点, 该点切向角 psi_ref, 有符号横向偏差 e)。"""
        r = math.hypot(x, y)
        if r < 1e-9:
            return (0.0, self.R), math.pi / 2.0, -self.R
        theta = math.atan2(y, x)
        px, py = self.R * math.cos(theta), self.R * math.sin(theta)
        psi_ref = wrap(theta + math.pi / 2.0)      # 逆时针切向
        e = r - self.R                              # >0 表示在圆外
        return (px, py), psi_ref, e

    def point_at(self, s):
        """按弧长 s 取点，用于画参考轨迹。"""
        theta = s / self.R
        return (self.R * math.cos(theta), self.R * math.sin(theta))


def stanley_steer(e, psi_e, v, k=1.2, max_steer=MAX_STEER,
                  curvature=0.0, L=L_WHEELBASE, feedforward=True):
    """Stanley 控制律（可选曲率前馈）。"""
    delta = psi_e + math.atan2(k * e, max(v, 0.1))
    if feedforward:
        delta += math.atan(L * curvature)
    return max(-max_steer, min(max_steer, delta))


def simulate(path=None, v=2.0, k=1.2, dt=0.01, duration=25.0,
             x0=None, psi0=math.pi / 2.0, L=L_WHEELBASE, feedforward=True):
    """闭环仿真。默认从圆外 1.5 m 处、朝向正确的情况下出发。"""
    path = path or CirclePath(8.0)
    if x0 is None:
        x0 = (path.R + 1.5, 0.0)
    x, y, psi = x0[0], x0[1], psi0
    ts, xs, ys, psis, es, deltas = [], [], [], [], [], []
    n = int(round(duration / dt))
    for i in range(n):
        t = i * dt
        _, psi_ref, e = path.nearest(x, y)
        psi_e = wrap(psi_ref - psi)
        kappa = path.curvature(x, y)
        delta = stanley_steer(e, psi_e, v, k, curvature=kappa, L=L,
                              feedforward=feedforward)
        ts.append(t)
        xs.append(x)
        ys.append(y)
        psis.append(psi)
        es.append(e)
        deltas.append(delta)
        # RK4 太兴师动众，控制周期内用欧拉足够（dt=10 ms）
        x += v * math.cos(psi) * dt
        y += v * math.sin(psi) * dt
        psi = wrap(psi + v * math.tan(delta) / L * dt)
    return {"ts": ts, "x": xs, "y": ys, "psi": psis, "e": es, "delta": deltas}


def reference_polyline(path=None, n=400):
    path = path or CirclePath(8.0)
    circumference = 2.0 * math.pi * path.R
    return [path.point_at(circumference * i / n) for i in range(n + 1)]
