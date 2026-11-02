"""2D 小车目标追踪纯逻辑核心（tracker_core）

旧版 main.py 的核心 bug（本模块修复的对象）：

    自动模式撞到障碍物后的处理是
        omega = copysign(MAX_STEER, self.omega if self.omega != 0 else 1.0)
    即"原地全锁转向、不记住绕行方向、不绕行"。当目标恰好在障碍物
    正后方时，小车会撞上障碍物 → 原地转一个角度 → 又朝目标撞上去 →
    再原地转……形成极限环，永远到不了目标（实测 3000 步仍卡在
    障碍物前 421px 处）。此外 trail 列表无限增长。

修复策略（本模块实现，人工势场法）：

1. 合成航向 = 目标引力 + 障碍物斥力 + 固定旋向的切向分量：
   - 引力指向目标，保证宏观上始终朝目标前进；
   - 斥力随距离靠近而增强，在障碍物影响半径内把小车推开；
   - 切向分量绕障碍物旋转 90°，专治"目标在障碍物正后方"的
     局部极小（纯引力+斥力会在这里对消卡死）；
   - 旋向（左/右）按首次受阻时"与目标射线的相对方位"选择一次
     并记忆，直到障碍物脱离影响范围，避免左右振荡。
2. 速度按前方净空自适应减速，贴障慢行、开阔处全速。
3. 真死锁退避：滑动窗口位移检测，确认完全卡住才倒车转向脱离。
4. 轨迹点封顶：trail 用定长队列，长跑不再无限占用内存。

本模块不依赖 pygame，便于单元测试与离线验证。
"""

import math
from collections import deque


def normalize_angle(a):
    """角度归一化到 (-pi, pi]"""
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def clip(x, lo, hi):
    return max(lo, min(hi, x))


class Car:
    """独轮运动学小车：势场避障 + 目标追踪"""

    #: 斥力影响半径 = 障碍物半径 + 车半径 + CLEAR_GAP
    CLEAR_GAP = 110.0
    #: 合成航向中斥力项（含切向）相对引力项的权重
    W_REPULSIVE = 2.2
    #: 切向分量相对径向斥力的权重（越大绕行越果断）
    W_TANGENT = 1.4

    def __init__(self, x, y, theta=0.0, obstacles=None,
                 max_speed=180.0, max_steer=3.5, kp_turn=2.5,
                 arrive_radius=18.0, car_radius=10.0, slow_radius=120.0,
                 trail_limit=600, bounds=None):
        self.x = x
        self.y = y
        self.theta = theta
        self.v = 0.0
        self.omega = 0.0
        self.obstacles = list(obstacles or [])
        self.max_speed = max_speed
        self.max_steer = max_steer
        self.kp_turn = kp_turn
        self.arrive_radius = arrive_radius
        self.car_radius = car_radius
        self.slow_radius = slow_radius
        self.trail = deque(maxlen=trail_limit)
        self.trail_limit = trail_limit
        # 势场绕避状态（逐障碍物记忆旋向，避免多障碍时切向分量互相抵消）
        self._swirl_map = {}          # obstacle -> (+1 / -1)
        # 卡死退避状态
        self.backoff_ticks = 0
        self._backoff_total = 18
        self._stuck_window = deque(maxlen=60)
        self.stuck_events = 0
        # 反复卡死时的临时绕行点（打破势场局部极小）
        self.detour = None
        self.detour_ticks = 0
        self.bounds = bounds or (900, 600)
        # 目标距离进展检测（绕圈时位移正常但目标距离不下降）
        self._progress_counter = 0
        self._progress_ref = None

    # ---------- 几何与碰撞 ----------

    def hit_obstacle(self, nx, ny, margin=0.0):
        """位置 (nx, ny) 是否与某个障碍物重叠，返回该障碍物或 None"""
        for (ox, oy, r) in self.obstacles:
            if math.hypot(nx - ox, ny - oy) < r + self.car_radius + margin:
                return (ox, oy, r)
        return None

    def clearance_ahead(self, heading, lookahead):
        """沿给定航向前方探测点的最小障碍净空（到障碍表面的距离）"""
        px = self.x + math.cos(heading) * lookahead
        py = self.y + math.sin(heading) * lookahead
        best = float('inf')
        for (ox, oy, r) in self.obstacles:
            best = min(best, math.hypot(px - ox, py - oy) - r - self.car_radius)
        return best

    # ---------- 势场避障 ----------

    def _swirl_for(self, obstacle, goal_dir):
        """按相对方位选择绕行旋向并逐障碍物记忆：
        障碍物在目标射线左侧(cross>0)时从右侧绕(swirl=-1)更近。"""
        if obstacle in self._swirl_map:
            return self._swirl_map[obstacle]
        ox, oy, _r = obstacle
        ux = self.x - ox
        uy = self.y - oy
        cross = goal_dir[0] * uy - goal_dir[1] * ux
        s = -1 if cross > 0 else 1
        self._swirl_map[obstacle] = s
        return s

    def steering_heading(self, tx, ty):
        """势场合成航向（弧度）"""
        dx = tx - self.x
        dy = ty - self.y
        d_goal = math.hypot(dx, dy)
        ax, ay = dx / d_goal, dy / d_goal
        rx = ry = 0.0
        active = []
        # 接近目标时衰减斥力：终点常位于障碍物影响圈内，
        # 斥力的切向分量会让小车绕着到达圈打转进不了终点，
        # 40px 内完全关闭斥力改为纯引力收敛（碰撞积分仍会兜底防撞）。
        goal_scale = clip((d_goal - 40.0) / 60.0, 0.0, 1.0)
        for (ox, oy, r) in self.obstacles:
            ddx = self.x - ox
            ddy = self.y - oy
            d = math.hypot(ddx, ddy)
            r_infl = r + self.car_radius + self.CLEAR_GAP
            if d >= r_infl:
                self._swirl_map.pop((ox, oy, r), None)
                continue
            active.append((ox, oy, r))
            w = (r_infl - d) / max(1e-6, self.CLEAR_GAP)   # 0..1
            w *= goal_scale
            ux, uy = ddx / d, ddy / d
            s = self._swirl_for((ox, oy, r), (ax, ay))
            # 径向斥力 + 固定旋向的切向分量
            rx += w * (ux - s * self.W_TANGENT * uy)
            ry += w * (uy + s * self.W_TANGENT * ux)
        hx = ax + self.W_REPULSIVE * rx
        hy = ay + self.W_REPULSIVE * ry
        return math.atan2(hy, hx), (active[0] if active else None)

    def _swirl_dir_hint(self):
        """退避时的转向参考：取当前记忆中的任一旋向，否则 +1"""
        if self._swirl_map:
            return next(iter(self._swirl_map.values()))
        return 1

    def _make_detour(self, tx, ty):
        """设置垂直于目标方向的临时绕行点，方向取当前绕行旋向"""
        gx, gy = tx - self.x, ty - self.y
        d = math.hypot(gx, gy)
        if d < 1e-6:
            self.detour = None
            return
        gx, gy = gx / d, gy / d
        s = self._swirl_dir_hint()
        off = 170.0
        dxp, dyp = -gy * s * off, gx * s * off
        w, h = self.bounds
        px = clip(self.x + dxp, 60.0, w - 60.0)
        py = clip(self.y + dyp, 60.0, h - 60.0)
        self.detour = (px, py)
        self.detour_ticks = 0

    # ---------- 单步仿真 ----------

    def auto_step(self, dt, tx, ty):
        """自动模式单步。到达目标返回 True，否则 False。"""
        dx = tx - self.x
        dy = ty - self.y
        dist = math.hypot(dx, dy)
        tx_use, ty_use = tx, ty
        if dist < self.arrive_radius:
            self.v = 0.0
            self.omega = 0.0
            self._swirl_map.clear()
            self.trail.append((self.x, self.y))
            return True

        # —— 临时绕行点：先到绕行点再回目标主线（超时自动放弃） ——
        if self.detour is not None:
            self.detour_ticks += 1
            dd = math.hypot(self.detour[0] - self.x, self.detour[1] - self.y)
            if dd < 45.0 or self.detour_ticks > 300:
                self.detour = None
            else:
                tx_use, ty_use = self.detour
                dx = tx_use - self.x
                dy = ty_use - self.y
                dist = math.hypot(dx, dy)

        # —— 真死锁退避进行中：倒车 + 定向转向 ——
        if self.backoff_ticks > 0:
            self.backoff_ticks -= 1
            self.v = -0.45 * self.max_speed
            self.omega = 0.7 * self.max_steer * (self._swirl_dir_hint())
            self._integrate(dt, ignore_collision=self.v < 0)
            if self.backoff_ticks == 0:
                self._stuck_window.clear()
                self._swirl_map.clear()   # 退避后重新选绕行方向
            self.trail.append((self.x, self.y))
            return False

        heading, active = self.steering_heading(tx_use, ty_use)
        err_h = normalize_angle(heading - self.theta)
        self.omega = clip(self.kp_turn * err_h, -self.max_steer,
                          self.max_steer)

        # 速度：目标距离减速 + 前方净空减速
        lookahead = 2.0 * self.car_radius + self.v * dt * 4.0
        clear = self.clearance_ahead(heading, lookahead)
        clear_scale = clip(clear / 60.0, 0.2, 1.0) if \
            self.obstacles else 1.0
        self.v = (self.max_speed
                  * min(1.0, dist / self.slow_radius)
                  * clear_scale
                  * max(0.35, math.cos(err_h)))

        self._integrate(dt)

        # —— 死锁检测：与 60 步前的位置比较，位移 < 1.5px 判定卡死 ——
        # 贴障慢行（约 2~7px/s，60 步 12~42px）不会误判，
        # 真正顶死（位移≈0）才触发。
        self._stuck_window.append((self.x, self.y))
        if len(self._stuck_window) > self._stuck_window.maxlen:
            old_x, old_y = self._stuck_window.popleft()
            if math.hypot(self.x - old_x, self.y - old_y) < 1.5:
                self._stuck_window.clear()
                self.stuck_events += 1
                self.backoff_ticks = self._backoff_total
                # 连续卡死 → 退出势场局部极小的临时绕行点
                if self.stuck_events >= 2 and self.detour is None:
                    self._make_detour(tx, ty)

        # —— 进展检测：每 180 步检查目标距离是否下降，
        #      绕圈极限环（位移正常但不接近目标）同样触发退避+绕行。
        #      终点临近区（< 80px）跳过：收尾接近速度只有十几 px/s，
        #      且纯引力已证明可收敛，误判反而会把车从终点踢飞。——
        self._progress_counter += 1
        if self._progress_ref is None:
            self._progress_ref = dist
        if self._progress_counter >= 180:
            self._progress_counter = 0
            d_goal = math.hypot(tx - self.x, ty - self.y)
            if d_goal > self._progress_ref - 5.0 and d_goal >= 80.0:
                self.stuck_events += 1
                self.backoff_ticks = self._backoff_total
                if self.stuck_events >= 2 and self.detour is None:
                    self._make_detour(tx, ty)
            self._progress_ref = d_goal

        self.trail.append((self.x, self.y))
        return False

    def _integrate(self, dt, ignore_collision=False):
        """按当前 v/omega 积分一步；碰撞时放弃位移返回 False"""
        new_theta = self.theta + self.omega * dt
        new_x = self.x + self.v * math.cos(new_theta) * dt
        new_y = self.y + self.v * math.sin(new_theta) * dt
        if not ignore_collision and self.hit_obstacle(new_x, new_y):
            self.v = 0.0
            return False
        self.x, self.y, self.theta = new_x, new_y, new_theta
        return True

    def manual_step(self, dt, forward, turn):
        """手动模式单步：forward ∈ {-1,0,1}，turn ∈ [-1,1]"""
        self.v = forward * 160.0
        self.omega = turn * 3.0
        self._integrate(dt)
        self.trail.append((self.x, self.y))


class MovingTarget:
    """随机游走目标：匀速直线 + 边界反弹"""

    def __init__(self, x, y, bounds=(900, 600), speed=60.0, rng=None):
        import random
        self.x = x
        self.y = y
        self.bounds = bounds
        self.speed = speed
        self._rng = rng or random
        angle = self._rng.uniform(0, 2 * math.pi)
        self.vx = speed * math.cos(angle)
        self.vy = speed * math.sin(angle)

    def step(self, dt):
        self.x += self.vx * dt
        self.y += self.vy * dt
        margin = 25
        w, h = self.bounds
        if self.x < margin or self.x > w - margin:
            self.vx *= -1
            self.x = max(margin, min(w - margin, self.x))
        if self.y < margin or self.y > h - margin:
            self.vy *= -1
            self.y = max(margin, min(h - margin, self.y))

    def set_pos(self, x, y):
        self.x = x
        self.y = y
