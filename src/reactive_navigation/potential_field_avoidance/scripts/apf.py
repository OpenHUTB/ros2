"""人工势场法（Artificial Potential Field）局部避障。

把"导航到目标"变成"沿着合成势场的下坡方向走"：

- 目标产生**引力势**（抛物线）：U_att(q) = 0.5 * zeta * |q - goal|^2
- 障碍产生**斥力势**（只在影响半径 d0 内生效）：
  U_rep(q) = 0.5 * eta * (1/d - 1/d0)^2      d = |q - 障碍表面|
- 合成梯度 F = -∇U，机器人每步沿 -∇U 方向前进固定步长。

势场法的经典缺陷是**局部极小**：引力与斥力在某处抵消（典型是凹形障碍
挡在目标前面），机器人卡住。本模块实现了可开关的逃逸机制（**切向滑行**：
沿负梯度的垂直方向绕障碍走，撞墙时向外偏转扫掠），并把"卡住"作为一等
结果返回，供回归测试与教学对比使用。

全部为纯函数 / 显式参数，方便离线单元测试（无 ROS 依赖）。
"""

import math
import random

# ----------------------------------------------------------------------------
# 演示地图：50 x 50，三块轴对齐矩形障碍 (x, y, w, h)
# ----------------------------------------------------------------------------
START = (5.0, 5.0)
GOAL = (45.0, 45.0)
OBSTACLES = [(10.0, 18.0, 6.0, 6.0),
             (28.0, 32.0, 5.0, 8.0),
             (36.0, 12.0, 7.0, 5.0)]

# 凹形陷阱（经典局部极小）：goal 藏在一堵厚墙正后方，机器人从正前方来。
# 引力（指向 goal，向上）与墙面斥力（向下）在墙前某处正好抵消，且横向
# 只有引力把机器人拉回中线 —— 这是一个**稳定**局部极小，纯梯度必卡死。
# 墙有限长，所以从全局看 goal 可达：随机扰动逃逸可以绕过墙端到达。
TRAP_OBSTACLES = [(20.0, 30.0, 20.0, 6.0)]     # 厚墙 20 x 6
TRAP_START = (30.0, 5.0)
TRAP_GOAL = (30.0, 41.0)      # 墙正后方 5 m


# ----------------------------------------------------------------------------
# 几何：点到矩形的距离
# ----------------------------------------------------------------------------
def clamp_to_rect(q, rect):
    """把点 q 收缩到矩形上（内部点原样返回）。"""
    x, y, w, h = rect
    return (min(max(q[0], x), x + w), min(max(q[1], y), y + h))


def dist_to_rect(q, rect):
    """点 q 到矩形表面的欧氏距离（在矩形内部为 0）。"""
    c = clamp_to_rect(q, rect)
    return math.hypot(q[0] - c[0], q[1] - c[1])


def inside_rect(q, rect):
    x, y, w, h = rect
    return x <= q[0] <= x + w and y <= q[1] <= y + h


def nearest_obstacle_distance(q, obstacles):
    """到所有障碍表面距离的最小值。"""
    return min(dist_to_rect(q, r) for r in obstacles)


# ----------------------------------------------------------------------------
# 势与梯度
# ----------------------------------------------------------------------------
def attractive_gradient(q, goal, zeta):
    """∇U_att = zeta * (q - goal)。"""
    return (zeta * (q[0] - goal[0]), zeta * (q[1] - goal[1]))


def repulsive_gradient(q, obstacles, eta, d0):
    """斥力势的梯度 ∇U_rep。d <= d0 时逐障碍累加，**方向指向障碍**
    （沿 U_rep 上升方向）；导航沿 -∇U 步进即被推离障碍。

    U_rep = 0.5*eta*(1/d - 1/d0)^2,  d = |q - c|,  c 为矩形表面最近点
    ∇U_rep = eta * (1/d0 - 1/d) / d^3 * (q - c)    （d < d0 时系数为负，
    故合成后指向障碍；d >= d0 时为 0）
    """
    gx = gy = 0.0
    for rect in obstacles:
        c = clamp_to_rect(q, rect)
        dx, dy = q[0] - c[0], q[1] - c[1]
        d = math.hypot(dx, dy)
        if d <= 1e-12 or d >= d0:
            continue                       # 在障碍内部或影响半径之外
        coef = eta * (1.0 / d0 - 1.0 / d) / (d * d * d)
        gx += coef * dx
        gy += coef * dy
    return (gx, gy)


def total_gradient(q, goal, obstacles, zeta=1.0, eta=100.0, d0=4.0):
    """合成梯度 ∇(U_att + U_rep)。"""
    ax, ay = attractive_gradient(q, goal, zeta)
    rx, ry = repulsive_gradient(q, obstacles, eta, d0)
    return (ax + rx, ay + ry)


def potential(q, goal, obstacles, zeta=1.0, eta=100.0, d0=4.0):
    """总势能 U(q)（供数值梯度校验与画等高线使用）。"""
    u_att = 0.5 * zeta * ((q[0] - goal[0]) ** 2 + (q[1] - goal[1]) ** 2)
    u_rep = 0.0
    for rect in obstacles:
        c = clamp_to_rect(q, rect)
        d = math.hypot(q[0] - c[0], q[1] - c[1])
        if 1e-12 < d < d0:
            u_rep += 0.5 * eta * (1.0 / d - 1.0 / d0) ** 2
    return u_att + u_rep


# ----------------------------------------------------------------------------
# 导航主循环
# ----------------------------------------------------------------------------
def navigate(start, goal, obstacles,
             step=0.5, zeta=1.0, eta=100.0, d0=4.0,
             tol=1.0, max_iter=4000,
             escape=True, escape_steps=60, escape_retries=12,
             seed=7, record=False):
    """沿 -∇U 步进导航。

    返回 dict：
        success     是否到达 tol 邻域
        path        轨迹 [(x, y), ...]
        steps       实际步数
        length      路径长度
        min_clear   全程到障碍的最小距离（>0 表示无碰撞）
        stuck       卡住时给出卡点，否则 None
        escapes     触发逃逸的次数
    """
    rng = random.Random(seed)
    q = (float(start[0]), float(start[1]))
    path = [q]
    escapes = 0
    clear_min = nearest_obstacle_distance(q, obstacles)
    trail = [q]                       # 震荡检测：最近 N 步的净位移

    for it in range(max_iter):
        if math.hypot(q[0] - goal[0], q[1] - goal[1]) < tol:
            return _result(True, path, it, clear_min, None, escapes)

        gx, gy = total_gradient(q, goal, obstacles, zeta, eta, d0)
        gnorm = math.hypot(gx, gy)
        if gnorm < 1e-9:
            # 梯度消失且未到达 —— 精确局部极小
            q, escaped, esc = _try_escape(q, goal, obstacles, zeta, eta, d0,
                                          step, tol, escape_steps, rng,
                                          escape if escapes < escape_retries else False)
            escapes += 1
            if not escaped:
                return _result(False, path, it, clear_min, q, escapes)
            path.extend(esc)
            trail = [q]
            continue

        # 步长自适应：离障碍越近步越小，避免一步跨过斥力平衡点造成震荡
        clear_now = nearest_obstacle_distance(q, obstacles)
        step_eff = step * max(0.1, min(1.0, clear_now / (2.0 * step)))

        # 固定步长沿负梯度方向
        nx = q[0] - step_eff * gx / gnorm
        ny = q[1] - step_eff * gy / gnorm
        cand = (nx, ny)
        clear_cand = nearest_obstacle_distance(cand, obstacles)

        # 拒绝明显往障碍里扎的步（斥力已经会推开，这里是兜底）
        if clear_cand <= 0.05:
            q, escaped, esc = _try_escape(q, goal, obstacles, zeta, eta, d0,
                                          step, tol, escape_steps, rng,
                                          escape if escapes < escape_retries else False)
            escapes += 1
            if not escaped:
                return _result(False, path, it, clear_min, q, escapes)
            path.extend(esc)
            trail = [q]
            continue

        if clear_cand < clear_min:
            clear_min = clear_cand
        q = cand
        path.append(q)

        # 震荡检测：最近 30 步净位移几乎为零 —— 卡在局部极小附近来回摆
        trail.append(q)
        if len(trail) > 30:
            trail.pop(0)
            drift = math.hypot(q[0] - trail[0][0], q[1] - trail[0][1])
            if drift < step * 1.2:
                q, escaped, esc = _try_escape(q, goal, obstacles, zeta, eta, d0,
                                              step, tol, escape_steps, rng,
                                              escape if escapes < escape_retries else False)
                escapes += 1
                if not escaped:
                    return _result(False, path, it, clear_min, q, escapes)
                path.extend(esc)
                trail = [q]

    return _result(False, path, max_iter, clear_min, None, escapes)


def _try_escape(q, goal, obstacles, zeta, eta, d0,
                step, tol, escape_steps, rng, escape):
    """从局部极小逃逸：沿切向绕障碍滑行。

    纯随机扰动在势垒前基本无效（扩散的净位移很小，走不远就被吸回
    卡点）；经典做法是**切向滑行**：沿负梯度方向的垂直方向持续走，
    贴墙时向外偏转，翻过势垒（绕过障碍端点）后净梯度自然恢复指向
    目标，回到梯度下降模式。
    """
    if not escape:
        return q, False, []
    base_d = math.hypot(q[0] - goal[0], q[1] - goal[1])
    start_clear = nearest_obstacle_distance(q, obstacles)
    gx, gy = total_gradient(q, goal, obstacles, zeta, eta, d0)
    n = math.hypot(gx, gy)
    if n < 1e-12:
        ang0 = math.atan2(goal[1] - q[1], goal[0] - q[0])
    else:
        ang0 = math.atan2(-gy, -gx)              # 负梯度方向
    side = rng.choice((1.0, -1.0))               # 绕行方向随机（左/右）
    ang = ang0 + side * math.pi / 2.0            # 负梯度转 90° 得切向
    trail = []
    for _ in range(escape_steps):
        if math.hypot(q[0] - goal[0], q[1] - goal[1]) < tol:
            return q, True, trail
        placed = False
        for da in (0.0, 0.26, -0.26, 0.52, -0.52, 0.79, -0.79, 1.05, -1.05):
            a = ang + da * side                  # 撞墙时向外偏转扫掠
            cand = (q[0] + step * math.cos(a), q[1] + step * math.sin(a))
            if nearest_obstacle_distance(cand, obstacles) > 0.05:
                q = cand
                placed = True
                break
        if not placed:
            break
        trail.append(q)
        # 墙端已过：滑出斥力影响半径（clear ≥ d0，净梯度 = 纯引力、必然
        # 指向目标），再叠加 "-∇U 与目标方向夹角 < 90°" 双保险 → 提前
        # 回到梯度模式。只按"离障碍比卡点更远"退出会被墙的侧面再次
        # 卡住（评审 #1400：旧版固定滑 60 步 = 30 m，越出墙端 20 m
        # 也越出 50 m 地图边界）。
        clear_now = nearest_obstacle_distance(q, obstacles)
        if clear_now >= d0 - 1e-9:
            g2x, g2y = total_gradient(q, goal, obstacles, zeta, eta, d0)
            if math.hypot(g2x, g2y) > 1e-9:
                dot = (-g2x) * (goal[0] - q[0]) + (-g2y) * (goal[1] - q[1])
                if dot > 0.0:
                    return q, True, trail
        # 已明显比卡点更接近目标 → 提前回到梯度模式
        if math.hypot(q[0] - goal[0], q[1] - goal[1]) < base_d - 3.0 * step:
            return q, True, trail
    return q, True, trail


def _result(success, path, steps, clear_min, stuck, escapes):
    length = sum(math.hypot(path[i + 1][0] - path[i][0],
                            path[i + 1][1] - path[i][1])
                 for i in range(len(path) - 1))
    return {
        "success": success,
        "path": path,
        "steps": steps,
        "length": length,
        "min_clear": clear_min,
        "stuck": stuck,
        "escapes": escapes,
    }


# ----------------------------------------------------------------------------
# 供 ROS 节点 / 文档演示复用的"一步决策"接口
# ----------------------------------------------------------------------------
def step_command(q, goal, obstacles, zeta=1.0, eta=100.0, d0=4.0):
    """给定当前位姿，返回负梯度方向的单位向量与势能（在线控制用）。"""
    gx, gy = total_gradient(q, goal, obstacles, zeta, eta, d0)
    n = math.hypot(gx, gy)
    if n < 1e-9:
        return None, potential(q, goal, obstacles, zeta, eta, d0)
    return (-gx / n, -gy / n), potential(q, goal, obstacles, zeta, eta, d0)
