#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tracker.py

基于规则（rule-based）的轨迹跟踪控制器：
- Pure Pursuit 几何跟踪参考路径；
- 档位（前进/倒车）由参考路径上 gear 字段决定，倒车时转向方向自动反号；
- 沿路径方向推进最近点索引，避免卡在起点；
- 到达终点（路径最后一点）后停车。
"""

import math

from collision_check import VehicleFootprint


def _densify(path, ds=0.1):
    """把路径加密到约 ds 米间距（线性插值）。"""
    out = [path[0]]
    for i in range(len(path) - 1):
        x1, y1, t1, g1 = path[i]
        x2, y2, t2, g2 = path[i + 1]
        seg_len = math.hypot(x2 - x1, y2 - y1)
        n = max(1, int(seg_len / ds))
        for k in range(1, n + 1):
            a = k / n
            out.append((x1 + a * (x2 - x1),
                        y1 + a * (y2 - y1),
                        t1 + a * (t2 - t1),
                        g1))
    return out


class DiffRCSim:
    """自行车模型模拟器（可视化用）。"""

    def __init__(self, vp: VehicleFootprint):
        self.vp = vp
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = 0.0
        self.steer = 0.0

    def reset(self, x, y, yaw):
        self.x, self.y, self.yaw = x, y, yaw
        self.v = 0.0
        self.steer = 0.0

    def step(self, v_cmd, steer_cmd, dt):
        self.v += max(-3.0 * dt, min(3.0 * dt, v_cmd - self.v))
        self.steer += max(-4.0 * dt, min(4.0 * dt, steer_cmd - self.steer))
        self.yaw += self.v / self.vp.wheel_base * math.tan(self.steer) * dt
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt


class PurePursuit:
    """Pure Pursuit 路径跟踪器（支持前进/倒车）。"""

    def __init__(self, path, vp, lookahead=1.0, speed=0.6):
        self.path = path
        self.vp = vp
        self.lookahead = lookahead
        self.speed = speed
        self.ld = lookahead
        self.target_idx = 0
        self.finished = False
        self._last_gear = path[0][3]
        self._stopping = False  # 正在换向停车中

    def run(self, car: DiffRCSim, dt):
        if self.finished:
            return 0.0, 0.0, {"at_goal": True}

        # 1. 沿路径方向找最近点：先从当前 target_idx 往后搜最近的
        idx = self._find_nearest(car.x, car.y)
        self.target_idx = idx

        # 2. 沿路径向前 lookahead 距离取目标点
        tx, ty, gear = self._lookahead_point(idx)

        # 3. Pure Pursuit 几何
        heading = car.yaw if gear > 0 else car.yaw + math.pi
        alpha = math.atan2(ty - car.y, tx - car.x) - heading
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))  # 归一化
        steer = math.atan2(2.0 * self.vp.wheel_base * math.sin(alpha), self.ld)
        max_steer = math.atan(self.vp.wheel_base / self.vp.min_turn_radius)
        steer = max(-max_steer, min(max_steer, steer))
        # 倒车时转向反号
        if gear < 0:
            steer = -steer

        v_cmd = self.speed * gear

        # 4. 到终点判断
        end_x, end_y = self.path[-1][0], self.path[-1][1]
        end_d = math.hypot(car.x - end_x, car.y - end_y)
        if end_d < 0.3 and idx >= len(self.path) - 5:
            self.finished = True
            return 0.0, 0.0, {"at_goal": True, "err": end_d}

        # 5. gear 切换：如果参考档位和上一帧不同，先把车刹停
        if gear != self._last_gear:
            self._stopping = True
            self._last_gear = gear
        if self._stopping:
            # 仍打方向对准前视点，但速度为 0
            if abs(car.v) < 0.05:
                self._stopping = False
            return 0.0, steer, {"at_goal": False, "idx": idx, "gear": gear, "stopping": True}

        return v_cmd, steer, {"at_goal": False, "idx": idx, "gear": gear}

    def _find_nearest(self, x, y):
        """从 target_idx 开始向前搜，找到路径上离 (x,y) 最近的点。
        如果当前点之后的点都越来越远，就停在最近处。"""
        best = self.target_idx
        best_d = math.hypot(self.path[best][0] - x, self.path[best][1] - y)
        # 向前搜 50 个点
        lo = max(0, self.target_idx - 5)
        hi = min(len(self.path) - 1, self.target_idx + 50)
        for i in range(lo, hi + 1):
            d = math.hypot(self.path[i][0] - x, self.path[i][1] - y)
            if d < best_d:
                best_d = d
                best = i
        return best

    def _lookahead_point(self, idx):
        """从 idx 出发，沿路径累计距离 lookahead，返回 (x, y, gear)。"""
        acc = 0.0
        i = idx
        while i < len(self.path) - 1:
            d = math.hypot(self.path[i + 1][0] - self.path[i][0],
                           self.path[i + 1][1] - self.path[i][1])
            acc += d
            i += 1
            if acc >= self.lookahead:
                break
        px, py, _, gear = self.path[i]
        return px, py, gear


if __name__ == "__main__":
    vp = VehicleFootprint()
    path = [(i * 0.3, 0.5 * math.sin(i * 0.2), 0.0, 1) for i in range(50)]
    car = DiffRCSim(vp)
    car.reset(0.0, 0.0, 0.0)
    ctrl = PurePursuit(path, vp)
    for step in range(500):
        v, s, info = ctrl.run(car, 0.05)
        car.step(v, s, 0.05)
        if info["at_goal"]:
            print("reached step", step, "pos=(%.2f, %.2f)" % (car.x, car.y))
            break
