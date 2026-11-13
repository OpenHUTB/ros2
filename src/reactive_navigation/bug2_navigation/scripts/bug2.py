"""Bug2 反应式导航：只用"撞墙—沿墙—回线"三种行为的完备导航。

与全局规划（A* / RRT）不同，Bug 系列算法**不建图**：它只假设机器人能感知
"前方多远会撞上障碍"（一根虚拟的激光笔），行为规则只有三条：

1. **走向目标（MOVE）**：沿起点到目标的直线 m-line 匀速前进；
2. **沿墙绕行（FOLLOW）**：撞到障碍后记下 hit 点，贴着障碍边界走；
3. **回到 m-line（LEAVE）**：沿墙途中若再次跨过 m-line、且此处离目标比
   hit 点更近，就离开障碍，回到行为 1。

Bug2 的核心不变量：**只有"更近的 m-line 交点"才允许离开障碍**。这保证
每个障碍最多绕行一圈内必做决策，因此只要目标可达就一定能到达（完备）；
反之若沿墙一整圈回到 hit 点，则宣告目标不可达 —— 这是 A*/RRT 这类
"全局最优但依赖建图"的算法不具备的确定性终止判据。

障碍为轴对齐矩形，边界按弧长参数化；纯函数实现，便于离线单测。
"""

import math

START = (2.0, 2.0)
GOAL = (48.0, 48.0)
OBSTACLES = [(15.0, 20.0, 8.0, 12.0),
             (30.0, 8.0, 6.0, 14.0),
             (35.0, 32.0, 10.0, 8.0)]

# 四面围墙夹一道窄门（门宽 2.0 m，机器人安全间隙需求 2*r_clear=2.4 m）：
# goal 在腔内。Bug2 沿墙发现墙体彼此相接（撞到相邻墙）
# → 确定性宣告不可达；势场法则会卡在门口的斥力垒上。
# 这是 A*/RRT 这类全局规划给不了的确定性终止判据。
WALLS = [(20.0, 30.0, 8.0, 3.0),    # 底左（门在 x∈[28,30]）
         (30.0, 30.0, 10.0, 3.0),   # 底右
         (20.0, 33.0, 3.0, 15.0),   # 左壁 y∈[33,48]
         (37.0, 33.0, 3.0, 15.0),   # 右壁
         (23.0, 45.0, 14.0, 3.0)]   # 顶
WALL_START = (29.0, 5.0)
WALL_GOAL = (29.0, 39.0)     # 腔内

MOVE, FOLLOW = "move", "follow"


# ----------------------------------------------------------------------------
# 矩形边界的弧长参数化
# ----------------------------------------------------------------------------
def inflate(rect, r):
    x, y, w, h = rect
    return (x - r, y - r, w + 2.0 * r, h + 2.0 * r)


def rect_perimeter(rect):
    _, _, w, h = rect
    return 2.0 * (w + h)


def boundary_point(rect, s):
    """膨胀矩形边界上的点，s 为弧长（逆时针：底边→右边→顶边→左边，
    从左下角起；y 向上的坐标系里这是逆时针）。"""
    x, y, w, h = rect
    s = s % rect_perimeter(rect)
    if s < w:                    # 顶边 →
        return (x + s, y)
    s -= w
    if s < h:                    # 右边 ↓
        return (x + w, y + s)
    s -= h
    if s < w:                    # 底边 ←
        return (x + w - s, y + h)
    s -= w
    return (x, y + h - s)        # 左边 ↑


def project_to_boundary(q, rect):
    """把点 q 投到矩形边界上，返回 (弧长 s, 投影点)。"""
    x, y, w, h = rect
    cx = min(max(q[0], x), x + w)
    cy = min(max(q[1], y), y + h)
    if cx == q[0] and cy == q[1]:          # 内点：推到最近边
        d_top, d_bot = cy - y, y + h - cy
        d_left, d_right = cx - x, x + w - cx
        m = min(d_top, d_bot, d_left, d_right)
        if m == d_top:
            cy = y
        elif m == d_bot:
            cy = y + h
        elif m == d_left:
            cx = x
        else:
            cx = x + w
    # 求弧长（逆时针，从左下角起）
    if abs(cy - y) < 1e-12 and x <= cx <= x + w:
        s = cx - x
    elif abs(cx - (x + w)) < 1e-12 and y <= cy <= y + h:
        s = w + (cy - y)
    elif abs(cy - (y + h)) < 1e-12 and x <= cx <= x + w:
        s = w + h + (x + w - cx)
    else:
        s = 2.0 * w + h + (y + h - cy)
    return s, (cx, cy)


# ----------------------------------------------------------------------------
# 几何工具
# ----------------------------------------------------------------------------
def dist_to_segment(q, a, b):
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    d2 = dx * dx + dy * dy
    if d2 < 1e-18:
        return math.hypot(q[0] - ax, q[1] - ay)
    t = max(0.0, min(1.0, ((q[0] - ax) * dx + (q[1] - ay) * dy) / d2))
    return math.hypot(q[0] - (ax + t * dx), q[1] - (ay + t * dy))


def on_mline(q, start, goal, eps):
    return dist_to_segment(q, start, goal) <= eps


def rect_distance(q, rect):
    c = (min(max(q[0], rect[0]), rect[0] + rect[2]),
         min(max(q[1], rect[1]), rect[1] + rect[3]))
    return math.hypot(q[0] - c[0], q[1] - c[1])


def which_obstacle(q, obstacles, r_clear):
    """返回第一个与 q 间隙小于 r_clear 的障碍索引，无则 None。"""
    for i, rect in enumerate(obstacles):
        if rect_distance(q, rect) < r_clear:
            return i
    return None


# ----------------------------------------------------------------------------
# Bug2 主循环
# ----------------------------------------------------------------------------
def navigate(start, goal, obstacles,
             step=0.5, r_clear=1.2, tol=1.0,
             mline_eps=0.6, max_iter=20000, max_rounds=1):
    """运行 Bug2，返回轨迹与事件记录。

    返回 dict：
        success    是否到达
        path       轨迹 [(x, y), ...]
        events     [(mode, point), ...]  模式切换事件（hit / leave / blocked）
        length     路径长度
        hits       撞墙转入沿墙的段数（"绕墙段数"，成功路径通常 > 0）
        detours    沿墙走满一整圈回到 hit 点的次数（>0 即判不可达）
    """
    start, goal = (float(start[0]), float(start[1])), (float(goal[0]), float(goal[1]))
    if which_obstacle(goal, obstacles, r_clear) is not None:
        return _finish(False, [start], [], 0.0, 0)      # 目标本身撞障碍

    q = start
    path = [q]
    events = []
    mode = MOVE
    detours = 0

    for _ in range(max_iter):
        if math.hypot(q[0] - goal[0], q[1] - goal[1]) < tol:
            return _finish(True, path, events, _length(path), detours)

        if mode == MOVE:
            d = math.hypot(goal[0] - q[0], goal[1] - q[1])
            cand = (q[0] + step * (goal[0] - q[0]) / d,
                    q[1] + step * (goal[1] - q[1]) / d)
            hit = which_obstacle(cand, obstacles, r_clear)
            if hit is None:
                q = cand
                path.append(q)
            else:
                # 记 hit 点（回退到障碍间隙边缘），转入沿墙
                rect = inflate(obstacles[hit], r_clear)
                q = boundary_point(rect, project_to_boundary(cand, rect)[0])
                q = (round(q[0], 9), round(q[1], 9))
                path.append(q)
                events.append(("hit", q))
                mode = FOLLOW
                wall = rect
                wall_idx = hit
                hit_dist = math.hypot(q[0] - goal[0], q[1] - goal[1])
                hit_point = q
                s = project_to_boundary(q, wall)[0]
                walked = 0.0
        else:
            # 沿墙逆时针步进
            per = rect_perimeter(wall)
            s = (s + step) % per
            walked += step
            nxt = boundary_point(wall, s)

            # 撞到相邻墙体：这堵墙不是孤立的，沿一圈走不通 → 不可达。
            # 注意（评审 #1400）：这是**只对封闭腔成立的保守启发式判据**。
            # 原始 Bug2 在这里应改跟踪"相邻障碍的联合边界"继续绕行；本
            # 模块沿墙只跟踪单块障碍的边界（教学重点在状态机而非墙体
            # 拼接），对"相邻但不封死"的障碍组会误报不可达——反例与
            # 适用范围见 docs/reactive_navigation/bug2_navigation.md 第 1
            # 节，并由 tests/test_bug2.py 锁死为已知限制。真实系统里沿墙
            # 由激光雷达实现，遇到转角/接缝时换墙跟踪联合边界。
            for j, o in enumerate(obstacles):
                if j != wall_idx and rect_distance(nxt, o) < r_clear:
                    events.append(("blocked", nxt))
                    path.append(nxt)
                    return _finish(False, path, events, _length(path), detours + 1)

            # 回到 hit 点附近且已走大半圈：目标被该障碍围死
            if (walked > per * 0.5 and
                    math.hypot(nxt[0] - hit_point[0], nxt[1] - hit_point[1]) < 2.0 * step):
                detours += 1
                path.append(nxt)
                if detours >= max_rounds:
                    return _finish(False, path, events, _length(path), detours)
                q = nxt
                path.append(q)
                continue

            q = nxt
            path.append(q)

            # 离开条件：回到 m-line 且比 hit 点更近目标
            if (on_mline(q, start, goal, mline_eps) and
                    math.hypot(q[0] - goal[0], q[1] - goal[1]) < hit_dist - 1e-9 and
                    math.hypot(q[0] - hit_point[0], q[1] - hit_point[1]) > 2.0 * step):
                # 检查离开后第一步是否可行（m-line 可能仍被同一障碍挡着）
                d = math.hypot(goal[0] - q[0], goal[1] - q[1])
                cand = (q[0] + step * (goal[0] - q[0]) / d,
                        q[1] + step * (goal[1] - q[1]) / d)
                if which_obstacle(cand, obstacles, r_clear) is None:
                    events.append(("leave", q))
                    mode = MOVE
    return _finish(False, path, events, _length(path), detours)


def _length(path):
    return sum(math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
               for i in range(len(path) - 1))


def _finish(success, path, events, length, detours):
    return {
        "success": success,
        "path": path,
        "events": events,
        "length": length,
        # hits：撞墙转入沿墙的次数（"绕墙段数"，成功路径通常 > 0）；
        # detours：一次沿墙以失败收尾的次数——绕满一整圈回到 hit 点，
        # 或撞到相邻墙体被 blocked 终止，两种"绕行走不完"都 +1
        # （成功路径恒为 0；事件类型以 events 里有无 blocked 区分，
        #   评审 #1400 次要项：detours>0 不区分两种失败原因）
        "hits": sum(1 for mode, _ in events if mode == "hit"),
        "detours": detours,
    }
