"""生成禁飞区航线穿越检测文档配图

输出：docs/img/air/no_fly_zone_check_path_check.png
内容：局部平面投影下的禁飞区地图，对比三条航线：
  1. 直线航线（旧版误判安全，本版拦截）
  2. 单斜点绕行（距禁区仅 2159m，加裕量后仍不达标）
  3. 三段直角绕行（每段 >= 2245m，校验通过）
"""
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# 中文显示（Windows 自带微软雅黑；Linux 回落 Noto CJK / 文泉驿）
plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei',
                                   'Noto Sans CJK SC', 'WenQuanYi Zen Hei',
                                   'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..',
                                    '..', '..'))
sys.path.insert(0, os.path.join(REPO, 'src', 'air', 'no_fly_zone_check'))
import nfz_core as nfz  # noqa: E402

ZONES = [
    {'name': 'Airport',      'lat': 47.65, 'lon': -122.14, 'radius_m': 2000},
    {'name': 'MilitaryBase', 'lat': 47.60, 'lon': -122.20, 'radius_m': 1500},
    {'name': 'School',       'lat': 47.62, 'lon': -122.10, 'radius_m': 500},
]

ORIGIN = (47.65, -122.14)   # 以机场中心为原点做局部平面投影


def to_local(lat, lon):
    """以机场中心为原点的局部平面坐标（米，x 向东，y 向北）"""
    y = (lat - ORIGIN[0]) * nfz.M_PER_DEG_LAT
    x = (lon - ORIGIN[1]) * nfz.M_PER_DEG_LAT * math.cos(math.radians(ORIGIN[0]))
    return x, y


fig, ax = plt.subplots(figsize=(9.5, 7.5), dpi=150)

# 禁飞区
zone_colors = {'Airport': '#d62728', 'MilitaryBase': '#7f7f7f',
               'School': '#ff7f0e'}
for z in ZONES:
    x, y = to_local(z['lat'], z['lon'])
    circ = plt.Circle((x, y), z['radius_m'], color=zone_colors[z['name']],
                      alpha=0.18, zorder=1)
    ax.add_patch(circ)
    circ_b = plt.Circle((x, y), z['radius_m'], color=zone_colors[z['name']],
                        fill=False, lw=1.8, zorder=2)
    ax.add_patch(circ_b)
    ax.annotate(z['name'], (x, y), ha='center', va='center', fontsize=11,
                color=zone_colors[z['name']], fontweight='bold', zorder=3)

routes = [
    ('直线航线：被本版拦截（旧版漏报）',
     [(47.65, -122.17), (47.65, -122.11)], '#d62728', 'o', '--'),
    ('单点斜绕：距机场中心 2159m，裕量不足',
     [(47.65, -122.17), (47.72, -122.14), (47.65, -122.11)],
     '#b8860b', 's', '-.'),
    ('三段直角绕行：每段 >= 2245m，校验通过',
     [(47.65, -122.17), (47.72, -122.17), (47.72, -122.11),
      (47.65, -122.11)], '#2ca02c', '^', '-'),
]

for label, pts, color, marker, ls in routes:
    xs, ys = zip(*[to_local(a, b) for a, b in pts])
    ax.plot(xs, ys, color=color, lw=2.0, ls=ls, marker=marker, ms=7,
            label=label, zorder=4)
    ax.annotate('P', (xs[0], ys[0]), textcoords='offset points',
                xytext=(-12, -4), fontsize=10, color=color, zorder=5)
    ax.annotate('D', (xs[-1], ys[-1]), textcoords='offset points',
                xytext=(6, -4), fontsize=10, color=color, zorder=5)

# 用核心库复核三条航线的最短距离，标注真实计算结果（错开高度避免重叠）
label_dy = [600, 450, 950]   # 每条航线的标注相对其最高点的高度偏移
for (label, pts, color, marker, ls), dy in zip(routes, label_dy):
    min_d = min(
        nfz.min_dist_to_segment_m(ZONES[0]['lat'], ZONES[0]['lon'],
                                  *[c for p in (pts[i], pts[i + 1])
                                    for c in p])[0]
        for i in range(len(pts) - 1))
    xs, ys = zip(*[to_local(a, b) for a, b in pts])
    mid = (xs[0] + xs[-1]) / 2.0
    ytop = max(ys) + dy
    ax.annotate('最短距离 %.0f m' % min_d, (mid, ytop), ha='center',
                fontsize=9, color=color, zorder=5)

ax.set_xlim(-7200, 5200)
ax.set_ylim(-6800, 9900)
ax.set_aspect('equal')
ax.grid(True, ls=':', alpha=0.4)
ax.set_xlabel('相对机场中心的东向距离 (m)')
ax.set_ylabel('相对机场中心的北向距离 (m)')
ax.set_title('禁飞区航线穿越检测（buffer = 200 m）\n'
             'P = 取货点, D = 送货点；直线航线两端安全但穿越禁区 —— 旧版漏报场景',
             fontsize=11)
ax.legend(loc='lower left', fontsize=9, framealpha=0.92)

out = os.path.join(REPO, 'docs', 'img', 'air',
                   'no_fly_zone_check_path_check.png')
os.makedirs(os.path.dirname(out), exist_ok=True)
fig.tight_layout()
fig.savefig(out)
print('saved:', out)

# 打印复核数据
for label, pts, *_ in routes:
    min_d = min(
        nfz.min_dist_to_segment_m(ZONES[0]['lat'], ZONES[0]['lon'],
                                  *[c for p in (pts[i], pts[i + 1])
                                    for c in p])[0]
        for i in range(len(pts) - 1))
    v = nfz.check_path(pts, [ZONES[0]], buffer_m=200.0)
    print('%-40s min_dist=%8.1f m  buffer200=%s'
          % (label, min_d, 'VIOLATION' if v else 'safe'))
