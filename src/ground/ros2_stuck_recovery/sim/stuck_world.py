"""离线仿真世界：差速底盘 + 墙体 + 三类可复现的"卡死"故障。

故障建模都落在轮子上，而不是直接改写位姿，这样里程计和真实运动的差异是"自然产生"的：

* ``blocked``：车顶住墙，轮子转不动 -> 编码器读数为 0，里程计也不动；
* ``slip``：地面湿滑，轮子空转 -> 编码器正常读数，车走得很少（动/静摩擦不同）；
* ``one_wheel``：单轮失去抓地（悬空/异物） -> 编码器照转，车却绕另一侧轮打转。

这三类正好对应检测器的三路证据，也是真实场景里下层控制器都不会报错的静默故障。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from ros2_stuck_recovery.robot import Limits, Twist, integrate_pose, wheel_speeds

BODY_RADIUS = 0.13       # 车体等效半径 [m]
RANGE_MAX = 3.50         # 前向测距量程 [m]


@dataclass(frozen=True)
class Segment:
    """墙体线段。"""

    x1: float
    y1: float
    x2: float
    y2: float


@dataclass(frozen=True)
class Rect:
    x0: float
    y0: float
    x1: float
    y1: float

    def contains(self, x: float, y: float) -> bool:
        return self.x0 <= x <= self.x1 and self.y0 <= y <= self.y1


@dataclass
class Scenario:
    """一个可复现场景。"""

    name: str
    title: str
    start: tuple[float, float, float] = (0.0, 0.0, 0.0)
    barriers: list[Segment] = field(default_factory=list)
    fault: str = "none"                 # none / slip / one_wheel
    slip_zone: Rect | None = None
    fault_until: float = 0.0            # 故障解除时刻 [s]
    duration: float = 24.0
    task: Twist = Twist(0.25, 0.0)      # 任务层指令（"一直往前走"）
    expect: str = "escape"              # escape / abort / no_trigger
    mu_kinetic: float = 0.12            # 动摩擦系数
    mu_static: float = 0.90             # 停顿后重新起步时的附着系数
    static_speed: float = 0.02          # 低于该速度视为"已停住" [m/s]
    slip_tau: float = 0.35              # 水膜/松土重新建立附着的衰减时间常数 [s]


@dataclass(frozen=True)
class Observation:
    t: float
    pose: tuple[float, float, float]        # 真值位姿（仅评分用，节点不消费）
    odom_pose: tuple[float, float, float]   # 里程计位姿
    v_odom: float
    w_odom: float
    w_imu: float
    range_front: float
    blocked: bool


def _dist_point_segment(px: float, py: float, seg: Segment) -> float:
    vx, vy = seg.x2 - seg.x1, seg.y2 - seg.y1
    wx, wy = px - seg.x1, py - seg.y1
    length2 = vx * vx + vy * vy
    if length2 < 1e-12:
        return math.hypot(wx, wy)
    t = max(0.0, min(1.0, (wx * vx + wy * vy) / length2))
    cx, cy = seg.x1 + t * vx, seg.y1 + t * vy
    return math.hypot(px - cx, py - cy)


def _ray_segment(px: float, py: float, yaw: float, seg: Segment, max_range: float) -> float:
    """从 (px,py) 沿 yaw 方向的射线到线段的最短交点距离。"""
    dx, dy = math.cos(yaw), math.sin(yaw)
    ax, ay = seg.x1 - px, seg.y1 - py
    bx, by = seg.x2 - px, seg.y2 - py
    denom = dx * (by - ay) - dy * (bx - ax)
    if abs(denom) < 1e-12:
        return max_range
    t = (ax * (by - ay) - ay * (bx - ax)) / denom
    u = (ax * dy - ay * dx) / denom
    if t < 0.0 or not (0.0 <= u <= 1.0):
        return max_range
    return min(t, max_range)


class StuckWorld:
    """一个场景 + 一台差速车。``step()`` 接受底盘指令，返回观测。"""

    def __init__(self, scenario: Scenario, seed: int = 0, limits: Limits | None = None,
                 noise_scale: float = 1.0) -> None:
        self.sc = scenario
        self.limits = limits or Limits()
        self.noise_scale = noise_scale
        self.rng = np.random.default_rng(seed)
        self.t = 0.0
        self.pose = scenario.start
        self.odom_pose = scenario.start
        self.step_index = 0
        self._spin = 0.0

    # ---------------------------------------------------------------- 内部
    def _collides(self, x: float, y: float) -> bool:
        return any(_dist_point_segment(x, y, s) < BODY_RADIUS for s in self.sc.barriers)

    def _slide(self, pose, dx: float, dy: float, new_yaw: float):
        """发生接触时只保留沿墙切向的位移，模拟车贴着墙滑动。"""
        best, best_d = None, 1e9
        for seg in self.sc.barriers:
            d = _dist_point_segment(pose[0] + dx, pose[1] + dy, seg)
            if d < best_d:
                best, best_d = seg, d
        if best is None:
            return None
        vx, vy = best.x2 - best.x1, best.y2 - best.y1
        norm = math.hypot(vx, vy)
        if norm < 1e-9:
            return None
        tx, ty = vx / norm, vy / norm
        along = dx * tx + dy * ty
        nx, ny = pose[0] + tx * along, pose[1] + ty * along
        if self._collides(nx, ny):
            return None
        return nx, ny, new_yaw

    def _range(self, pose) -> float:
        best = RANGE_MAX
        for seg in self.sc.barriers:
            best = min(best, _ray_segment(pose[0], pose[1], pose[2], seg, RANGE_MAX))
        return best

    # ---------------------------------------------------------------- 对外
    def step(self, cmd: Twist, dt: float = 0.05) -> Observation:
        sc = self.sc
        self.step_index += 1

        # 1) 指令 -> 轮速（限幅后再反解，和真实驱动板一致）
        wl_cmd, wr_cmd = wheel_speeds(cmd, self.limits.wheel_base)
        wl_enc, wr_enc = wl_cmd, wr_cmd
        wl_trac, wr_trac = wl_cmd, wr_cmd

        # 2) 故障注入
        slipping = False
        if sc.fault == "one_wheel" and self.t < sc.fault_until:
            # 左轮悬空：编码器照转，但不产生牵引力
            wl_trac = 0.0
        if sc.fault == "slip" and self.t < sc.fault_until and sc.slip_zone is not None:
            if sc.slip_zone.contains(self.pose[0], self.pose[1]):
                slipping = True
                v_nom = 0.5 * (wl_trac + wr_trac)
                if abs(v_nom) < sc.static_speed:
                    # 停住 -> 附着随时间恢复（水膜排开 / 松土回填）
                    self._spin = 0.0
                mu = sc.mu_kinetic + (sc.mu_static - sc.mu_kinetic) * math.exp(-self._spin / sc.slip_tau)
                self._spin += dt
                wl_trac *= mu
                wr_trac *= mu

        # 3) 真实运动（含贴墙滑动）
        v_tr = 0.5 * (wl_trac + wr_trac)
        w_tr = (wr_trac - wl_trac) / self.limits.wheel_base
        new_pose = integrate_pose(self.pose, Twist(v_tr, w_tr), dt)
        blocked = False
        if self._collides(new_pose[0], new_pose[1]):
            dx = new_pose[0] - self.pose[0]
            dy = new_pose[1] - self.pose[1]
            requested = math.hypot(dx, dy)
            slid = self._slide(self.pose, dx, dy, new_pose[2])
            if slid is None:
                new_pose = self.pose
                blocked = True
            else:
                new_pose = (slid[0], slid[1], new_pose[2])
                achieved = math.hypot(slid[0] - self.pose[0], slid[1] - self.pose[1])
                # 贴墙蹭出一点点不等于"走得动"：位移不足一半就按堵转处理
                if requested > 1e-6 and achieved < 0.5 * requested:
                    blocked = True
        if blocked:
            # 轮子被墙体别住：编码器也停在原处
            wl_enc = wr_enc = 0.0
        self.pose = new_pose

        # 4) 传感器：编码器噪声 + 陀螺仪偏差噪声 + 测距噪声
        noise = self.rng.normal(0.0, 0.004 * self.noise_scale, size=2)
        v_odom_true = 0.5 * (wl_enc + wr_enc) + noise[0]
        w_odom_true = (wr_enc - wl_enc) / self.limits.wheel_base + noise[1]
        self.odom_pose = integrate_pose(self.odom_pose, Twist(v_odom_true, w_odom_true), dt)

        w_actual = (wr_trac - wl_trac) / self.limits.wheel_base
        w_imu = w_actual + 0.02 + float(self.rng.normal(0.0, 0.01 * self.noise_scale))
        rng_front = self._range(self.pose) + float(self.rng.normal(0.0, 0.01 * self.noise_scale))

        self.t += dt
        return Observation(
            t=self.t,
            pose=self.pose,
            odom_pose=self.odom_pose,
            v_odom=v_odom_true,
            w_odom=w_odom_true,
            w_imu=w_imu,
            range_front=max(0.0, rng_front),
            blocked=blocked,
        )
