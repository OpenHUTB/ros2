"""
轮胎滑移率-附着特性与最佳制动滑移率仿真（纯 Python 数值仿真）

依赖: numpy, matplotlib, Pillow
使用: python main.py
说明: 采用 Pacejka 魔术公式的简化形式刻画制动滑移率 s 与路面附着系数 μ
      的关系，对比干沥青、湿沥青、积雪、结冰四种路面的峰值附着系数与最佳
      滑移率区间，并按峰值附着估算不同初速度下的制动距离趋势。所有结果
      写入脚本同级 results/ 目录。
"""
import os
import csv

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image

RESULTS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'results')

G = 9.81
KMH = 1000.0 / 3600.0
SLIP = np.linspace(0.0, 1.0, 200)
BRAKE_SPEEDS = [60.0, 100.0]   # km/h

# 四种路面：名称 -> 魔术公式参数 (D 峰值附着, B 刚度因子, C 形状因子) 与颜色
SURFACES = {
    'dry_asphalt':  {'D': 0.95, 'B': 12.0, 'C': 1.6, 'color': 'tab:brown',
                    'label': 'Dry asphalt'},
    'wet_asphalt': {'D': 0.65, 'B': 10.0, 'C': 1.6, 'color': 'tab:blue',
                    'label': 'Wet asphalt'},
    'snow':         {'D': 0.30, 'B': 8.0,  'C': 1.5, 'color': 'tab:cyan',
                    'label': 'Snow'},
    'ice':          {'D': 0.15, 'B': 6.0,  'C': 1.5, 'color': 'tab:purple',
                    'label': 'Ice'},
}


def adhesion(s, D, B, C):
    """简化魔术公式附着系数 μ(s) = D·sin(C·arctan(B·s))。"""
    return D * np.sin(C * np.arctan(B * s))


def peak_slip(B, C):
    """峰值附着对应的最佳滑移率 s* = tan(π/(2C))/B。"""
    return float(np.tan(np.pi / (2.0 * C)) / B)


def brake_distance(v_kmh, mu_peak):
    """按峰值附着估算制动距离 d = v²/(2·μ·g) (m)。"""
    vms = v_kmh * KMH
    return vms * vms / (2.0 * mu_peak * G)


def summarize():
    """汇总各路面峰值附着、最佳滑移率与制动距离。"""
    rows = []
    for name, p in SURFACES.items():
        s_star = peak_slip(p['B'], p['C'])
        row = {
            'surface': name,
            'mu_peak': round(p['D'], 3),
            'optimal_slip': round(s_star, 3),
        }
        for v in BRAKE_SPEEDS:
            row[f'brake_distance_{int(v)}kmh_m'] = round(
                brake_distance(v, p['D']), 2)
        rows.append(row)
    return rows


# ---------- 绘图 ----------
def plot_curves(out_path):
    """四种路面滑移率-附着系数曲线，标注峰值点。"""
    fig, ax = plt.subplots(figsize=(9, 5))
    for name, p in SURFACES.items():
        mu = adhesion(SLIP, p['D'], p['B'], p['C'])
        ax.plot(SLIP, mu, color=p['color'], lw=1.8, label=p['label'])
        s_star = peak_slip(p['B'], p['C'])
        ax.scatter([s_star], [p['D']], color=p['color'], edgecolor='black', s=60, zorder=5)
        ax.annotate(f"s*={s_star:.2f}", (s_star, p['D']),
                    textcoords='offset points', xytext=(6, -2), fontsize=8)
    ax.set_xlabel('slip ratio s')
    ax.set_ylabel('adhesion coefficient μ')
    ax.set_title('Tire slip-adhesion curves by road surface')
    ax.legend(fontsize=8); ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)


def plot_peak_bar(rows, out_path):
    """峰值附着系数对比柱状，标注最佳滑移率。"""
    names = list(SURFACES.keys())
    mus = [r['mu_peak'] for r in rows]
    sstars = [r['optimal_slip'] for r in rows]
    colors = [SURFACES[n]['color'] for n in names]

    fig, ax = plt.subplots(figsize=(9, 4.8))
    x = np.arange(len(names))
    bars = ax.bar(x, mus, color=colors, width=0.55)
    ax.set_xticks(x)
    ax.set_xticklabels([SURFACES[n]['label'] for n in names])
    ax.set_ylabel('peak adhesion μ_max')
    ax.set_title('Peak adhesion and optimal slip by surface')
    ax.grid(alpha=0.3, axis='y')
    for bar, mu, ss in zip(bars, mus, sstars):
        ax.text(bar.get_x() + bar.get_width() / 2, mu,
                f'μ={mu:.2f}\ns*={ss:.2f}', ha='center', va='bottom', fontsize=8)
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)


def plot_brake_distance(rows, out_path):
    """各路面在两个初速度下的制动距离分组柱状。"""
    names = list(SURFACES.keys())
    labels = [SURFACES[n]['label'] for n in names]
    fig, ax = plt.subplots(figsize=(9, 4.8))
    x = np.arange(len(names))
    width = 0.36
    for i, v in enumerate(BRAKE_SPEEDS):
        col = f'brake_distance_{int(v)}kmh_m'
        vals = [r[col] for r in rows]
        ax.bar(x + (i - 0.5) * width, vals, width, label=f'{int(v)} km/h')
    ax.set_xticks(x); ax.set_xticklabels(labels)
    ax.set_ylabel('braking distance (m)')
    ax.set_title('Estimated braking distance by surface')
    ax.legend(fontsize=8); ax.grid(alpha=0.3, axis='y')
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)


def render_demo_frame(s_cur, out_size=(800, 420)):
    """渲染一帧：附着曲线 + 当前滑移率标记。"""
    fig, ax = plt.subplots(figsize=(10, 4.6))
    for name, p in SURFACES.items():
        mu = adhesion(SLIP, p['D'], p['B'], p['C'])
        ax.plot(SLIP, mu, color=p['color'], lw=1.4, label=p['label'])
        mu_cur = adhesion(s_cur, p['D'], p['B'], p['C'])
        ax.scatter([s_cur], [mu_cur], color=p['color'], edgecolor='black', s=50, zorder=5)
    ax.axvline(s_cur, color='black', ls=':', lw=1.2)
    ax.set_xlim(0, 1); ax.set_ylim(0, 1.0)
    ax.set_xlabel('slip ratio s'); ax.set_ylabel('adhesion coefficient μ')
    ax.set_title(f'Tire adhesion @ slip={s_cur:.2f}')
    ax.legend(fontsize=8); ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.canvas.draw()
    buf = np.frombuffer(fig.canvas.buffer_rgba(), dtype=np.uint8)
    img = Image.fromarray(buf.reshape(fig.canvas.get_width_height()[::-1] + (4,)))
    plt.close(fig)
    return img.convert('RGB').resize(out_size)


# ---------- MuJoCo 四路面单车场景静态帧（可选依赖） ----------
# 四块并排路板依次对应干沥青 / 湿沥青 / 积雪 / 结冰，配色与附着曲线一致；
# 单车停在干沥青板上。该图仅作路面工况示意，峰值附着与制动距离以曲线、柱状图和指标表为准。
_GRID_LINES = ''.join(
    f'<geom type="box" size="0.008 6 0.001" pos="{i} 0 0.002" rgba="0.66 0.66 0.69 1"/>'
    f'<geom type="box" size="6 0.008 0.001" pos="0 {i} 0.002" rgba="0.66 0.66 0.69 1"/>'
    for i in range(-6, 7))

_SURFACE_PADS = [
    (-2.1, '0.55 0.34 0.29 0.55'),   # dry asphalt
    (-0.7, '0.12 0.47 0.71 0.50'),   # wet asphalt
    (0.7,  '0.62 0.86 0.90 0.50'),   # snow
    (2.1,  '0.72 0.62 0.85 0.50'),   # ice
]
_PADS = ''.join(
    f'<geom type="box" size="1.3 2.6 0.004" pos="{x} 0 0.004" rgba="{c}"/>'
    for x, c in _SURFACE_PADS)

SCENE_XML = """<mujoco><compiler angle="radian"/>
<default><geom contype="0" conaffinity="0"/></default>
<worldbody>
<light name="key" dir="-0.35 -0.55 -1" diffuse="0.95 0.95 0.95" ambient="0.55 0.55 0.55"/>
<geom name="ground" type="plane" size="6 6 0.02" rgba="0.86 0.86 0.88 1"/>
""" + _GRID_LINES + _PADS + """
<camera name="cam" mode="fixed" pos="0 -7 5.2" xyaxes="1 0 0 0 0.74 0.67" fovy="45"/>
<body name="car" pos="-2.1 0 0.16"><joint type="free"/>
<geom type="box" size="0.5 0.26 0.14" rgba="0.45 0.25 0.18 1"/>
<geom type="cylinder" size="0.08 0.05" pos="0.32 -0.28 0" euler="1.5708 0 0" rgba="0.1 0.1 0.1 1"/>
<geom type="cylinder" size="0.08 0.05" pos="0.32  0.28 0" euler="1.5708 0 0" rgba="0.1 0.1 0.1 1"/>
<geom type="cylinder" size="0.08 0.05" pos="-0.32 -0.28 0" euler="1.5708 0 0" rgba="0.1 0.1 0.1 1"/>
<geom type="cylinder" size="0.08 0.05" pos="-0.32  0.28 0" euler="1.5708 0 0" rgba="0.1 0.1 0.1 1"/>
</body>
</worldbody></mujoco>"""


def render_scene_png(out_path):
    """MuJoCo 离屏渲染单帧四路面单车场景静态图（网格地面斜俯视）。

    mujoco 为可选依赖：未安装或离屏 GL 不可用时返回 None，不阻断曲线图产物。
    """
    try:
        import mujoco
    except (ImportError, OSError, RuntimeError):
        print("未安装 mujoco，跳过仿真场景静态图（曲线图与数据表不受影响）")
        return None

    os.environ.setdefault('MUJOCO_GL', 'egl')
    model = mujoco.MjModel.from_xml_string(SCENE_XML)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    renderer = mujoco.Renderer(model, 480, 640)
    renderer.update_scene(data, camera='cam')
    Image.fromarray(renderer.render()).save(out_path)
    try:
        renderer.close()
    except AttributeError:
        pass
    return os.path.getsize(out_path)


def main():
    os.makedirs(RESULTS_DIR, exist_ok=True)
    rows = summarize()

    csv_path = os.path.join(RESULTS_DIR, 'metrics.csv')
    fields = ['surface', 'mu_peak', 'optimal_slip'] + \
             [f'brake_distance_{int(v)}kmh_m' for v in BRAKE_SPEEDS]
    with open(csv_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

    plot_curves(os.path.join(RESULTS_DIR, 'slip_curves.png'))
    plot_peak_bar(rows, os.path.join(RESULTS_DIR, 'slip_peak_bar.png'))
    plot_brake_distance(rows, os.path.join(RESULTS_DIR, 'slip_brake_distance.png'))

    slips = np.linspace(0.05, 1.0, 18)
    frames = [render_demo_frame(float(s)) for s in slips]
    if frames:
        duration = max(80, int(1000 / len(frames) * 10))
        frames[0].save(os.path.join(RESULTS_DIR, 'slip_demo.gif'),
                       save_all=True, append_images=frames[1:],
                       duration=duration, loop=0, optimize=True)

    scene_png = render_scene_png(os.path.join(RESULTS_DIR, 'slip_scene.png'))
    if scene_png:
        print(f"仿真场景静态图已写入 slip_scene.png（{scene_png // 1024} KB）")

    for r in rows:
        print(f"{r['surface']:13s} μ_peak={r['mu_peak']:.2f} "
              f"s*={r['optimal_slip']:.3f} "
              f"d60={r['brake_distance_60kmh_m']}m "
              f"d100={r['brake_distance_100kmh_m']}m")
    print(f'实验完成，结果保存在 {RESULTS_DIR} 目录')


if __name__ == '__main__':
    main()
