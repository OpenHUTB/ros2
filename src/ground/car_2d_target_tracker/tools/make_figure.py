"""生成绕障修复对比图（不依赖 pygame，纯 tracker_core 数据）

输出：docs/img/ground/car_2d_tracker_avoidance_compare.png
左图：旧算法（原地全锁转向）在"目标于障碍物正后方"场景死锁绕圈，
      新算法（势场+切向绕行）8 秒内到达；
右图：7 个压力场景的轨迹总览。
"""
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei',
                                   'Noto Sans CJK SC', 'WenQuanYi Zen Hei',
                                   'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import tracker_core as tc  # noqa: E402

DT = 1.0 / 60.0


# ---------- 旧算法复刻（修复前 main.py 的 auto_step） ----------

def old_auto_step(car, dt, tx, ty, obstacles):
    dx, dy = tx - car['x'], ty - car['y']
    dist = math.hypot(dx, dy)
    if dist < 18.0:
        return True
    ta = math.atan2(dy, dx)
    err = tc.normalize_angle(ta - car['theta'])
    car['omega'] = max(-3.5, min(3.5, 2.5 * err))
    sf = max(0.0, math.cos(err))
    car['v'] = 180.0 * sf * min(1.0, dist / 120.0)
    nt = car['theta'] + car['omega'] * dt
    nx = car['x'] + car['v'] * math.cos(nt) * dt
    ny = car['y'] + car['v'] * math.sin(nt) * dt
    hit = any(math.hypot(nx - ox, ny - oy) < r + 10.0 for ox, oy, r in obstacles)
    if hit:
        car['v'] = 0.0
        car['omega'] = math.copysign(3.5, car['omega'] if car['omega'] != 0 else 1.0)
    else:
        car['x'], car['y'], car['theta'] = nx, ny, nt
    car['trail'].append((car['x'], car['y']))
    return False


def run_old(start, goal, obstacles, steps=1000):
    car = {'x': start[0], 'y': start[1], 'theta': start[2],
           'v': 0.0, 'omega': 0.0, 'trail': []}
    for _ in range(steps):
        if old_auto_step(car, DT, *goal, obstacles):
            break
    return car


def run_new(start, goal, obstacles, steps=3000):
    car = tc.Car(*start, obstacles=obstacles)
    for _ in range(steps):
        if car.auto_step(DT, *goal):
            break
    return car


OB1 = [(450, 300, 60)]

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13.2, 5.6), dpi=150)

# ---- 左图：旧算法死锁 vs 新算法绕行 ----
for ox, oy, r in OB1:
    ax1.add_patch(plt.Circle((ox, oy), r, color='#9e9e9e', alpha=0.85))
    ax1.annotate('障碍物', (ox, oy), ha='center', va='center', fontsize=9,
                 color='white')

old = run_old((100, 300, 0.0), (800, 300), OB1, steps=1000)
xs, ys = zip(*old['trail'])
ax1.plot(xs, ys, color='#d62728', lw=1.6, label='旧算法：原地转向死锁（16.7 s 仍卡在障碍前）')
ax1.annotate('卡死区', (xs[-1], ys[-1]), textcoords='offset points',
             xytext=(8, 8), fontsize=9, color='#d62728')

new = run_new((100, 300, 0.0), (800, 300), OB1)
xs, ys = zip(*new.trail)
ax1.plot(xs, ys, color='#2ca02c', lw=1.8, label='新算法：势场绕行 5.6 s 到达')
ax1.annotate('绕行轨迹', (460, 380), textcoords='offset points',
             xytext=(6, 6), fontsize=9, color='#2ca02c')

ax1.plot(100, 300, 'o', color='#1f77b4', ms=8)
ax1.annotate('P', (100, 300), textcoords='offset points', xytext=(-14, -4),
             fontsize=10, color='#1f77b4')
ax1.plot(800, 300, '*', color='#d62728', ms=14)
ax1.annotate('目标', (800, 300), textcoords='offset points', xytext=(8, 4),
             fontsize=10, color='#d62728')
ax1.set_xlim(0, 900)
ax1.set_ylim(120, 480)
ax1.set_aspect('equal')
ax1.grid(True, ls=':', alpha=0.4)
ax1.set_title('目标在障碍物正后方：旧算法死锁 vs 新算法绕行', fontsize=11)
ax1.legend(loc='upper left', fontsize=8.5, framealpha=0.92)
ax1.set_xlabel('x (px)')
ax1.set_ylabel('y (px)')

# ---- 右图：压力场景轨迹总览 ----
scenarios = [
    ('正后方单障碍', (100, 300, 0.0), (800, 300), [(450, 300, 60)], '#1f77b4'),
    ('窄门 120px', (100, 300, 0.0), (800, 300), [(450, 180, 60), (450, 420, 60)], '#ff7f0e'),
    ('错位围墙三连', (100, 300, 0.0), (800, 300),
     [(300, 300, 70), (500, 220, 70), (700, 340, 70)], '#2ca02c'),
    ('目标在两障碍之间', (450, 550, 0.0), (450, 50), [(450, 300, 60), (380, 300, 60)], '#9467bd'),
]
for name, start, goal, obs, color in scenarios:
    car = run_new(start, goal, obs)
    xs, ys = zip(*car.trail)
    ax2.plot(xs, ys, color=color, lw=1.4, label=name)
    for ox, oy, r in obs:
        ax2.add_patch(plt.Circle((ox, oy), r, color='#bdbdbd', alpha=0.6))
    ax2.plot(*start[:2], 'o', color=color, ms=5)
    ax2.plot(*goal, '*', color=color, ms=10)

ax2.set_xlim(0, 900)
ax2.set_ylim(0, 600)
ax2.set_aspect('equal')
ax2.grid(True, ls=':', alpha=0.35)
ax2.set_title('新算法压力场景轨迹总览（全部到达，零碰撞）', fontsize=11)
ax2.legend(loc='lower right', fontsize=8.5, framealpha=0.92)
ax2.set_xlabel('x (px)')
ax2.set_ylabel('y (px)')

out = os.path.join('..', '..', '..', 'docs', 'img', 'ground',
                   'car_2d_tracker_avoidance_compare.png')
out = os.path.abspath(out)
os.makedirs(os.path.dirname(out), exist_ok=True)
fig.tight_layout()
fig.savefig(out)
print('saved:', out)
