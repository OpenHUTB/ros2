"""
巡航风阻与能耗敏感性分析（纯 Python 数值仿真）

依赖: numpy, matplotlib, Pillow
使用: python main.py
说明: 把车辆行驶阻力分解为滚动阻力、坡度阻力与空气阻力，在不同车速、
      风阻系数（车型）与道路坡度下计算总阻力、驱动功率与百公里能耗，
      量化车速、空气动力学外形和坡度对巡航能耗的影响。所有结果写入脚本
      同级的 results/ 目录。
"""
import os
import csv

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image

RESULTS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'results')

# ---------- 车辆与环境参数 ----------
MASS = 1800.0          # 满载质量 (kg)
MU_RR = 0.012          # 滚动阻力系数
G = 9.81               # 重力加速度 (m/s²)
RHO = 1.225            # 空气密度 (kg/m³)
FRONT_AREA = 2.2       # 迎风面积 (m²)
DRIVE_EFF = 0.85       # 驱动 / 传动综合效率
MS_PER_KMH = 1000.0 / 3600.0
J_PER_KWH = 3.6e6

V_KMH = np.linspace(5.0, 120.0, 120)     # 车速扫描 (km/h)
GRADE_CHECK = [0.0, 3.0, 6.0, -3.0]      # 坡度情景 (%)
SPEED_CHECK = [60.0, 90.0, 120.0]        # 指标统计车速 (km/h)

# 三种典型风阻系数车型
MODELS = {
    'sedan':  {'cd': 0.28, 'label': 'Sedan Cd=0.28', 'color': 'tab:blue'},
    'normal': {'cd': 0.35, 'label': 'Normal Cd=0.35', 'color': 'tab:green'},
    'suv':    {'cd': 0.42, 'label': 'SUV Cd=0.42',  'color': 'tab:red'},
}


def resistance(v_ms, cd, grade_pct, mass=MASS):
    """分解计算各阻力分量，返回 (滚动, 坡度, 空气, 总) 阻力 (N)。"""
    grade = grade_pct / 100.0
    sin_th = grade / np.sqrt(1.0 + grade * grade)
    cos_th = 1.0 / np.sqrt(1.0 + grade * grade)
    f_rr = MU_RR * mass * G * cos_th
    f_gr = mass * G * sin_th
    f_ad = 0.5 * RHO * cd * FRONT_AREA * v_ms * v_ms
    return f_rr, f_gr, f_ad, f_rr + f_gr + f_ad


def power_kw(v_ms, f_tot):
    """驱动功率 (kW)。"""
    return f_tot * v_ms / 1000.0


def energy_kwh_per_100km(f_tot, eff=DRIVE_EFF):
    """百公里能耗 (kWh/100km)，按驱动效率折算。"""
    return f_tot * 100_000.0 / eff / J_PER_KWH


def grade_label(g):
    return f'{g:+.0f}%'


def summarize():
    """统计各车速、各坡度下的阻力、功率与能耗。"""
    rows = []
    for g in GRADE_CHECK:
        for v in SPEED_CHECK:
            vms = v * MS_PER_KMH
            _, _, _, f_tot = resistance(vms, MODELS['normal']['cd'], g)
            rows.append({
                'grade_pct': g,
                'speed_kmh': v,
                'total_resistance_N': round(float(f_tot), 1),
                'power_kW': round(float(power_kw(vms, f_tot)), 2),
                'energy_kwh_per_100km': round(float(energy_kwh_per_100km(f_tot)), 2),
            })
    return rows


# ---------- 绘图 ----------
def plot_drag_components(out_path):
    """平路、普通车型下各阻力分量随车速变化。"""
    vms = V_KMH * MS_PER_KMH
    f_rr = np.full_like(vms, MU_RR * MASS * G)
    f_ad = 0.5 * RHO * MODELS['normal']['cd'] * FRONT_AREA * vms ** 2
    f_tot = f_rr + f_ad

    fig, ax = plt.subplots(figsize=(9, 4.8))
    ax.plot(V_KMH, f_rr, color='tab:gray', lw=1.6, label='rolling resistance')
    ax.plot(V_KMH, f_ad, color='tab:blue', lw=1.6, label='aerodynamic drag')
    ax.plot(V_KMH, f_tot, color='black', lw=2.0, label='total resistance')
    ax.fill_between(V_KMH, f_rr, f_tot, color='tab:blue', alpha=0.12)
    ax.set_xlabel('speed (km/h)'); ax.set_ylabel('resistance (N)')
    ax.set_title('Driving resistance components on flat road')
    ax.legend(fontsize=8); ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)


def plot_power_by_model(out_path):
    """三种风阻车型驱动功率随车速对比（平路）。"""
    vms = V_KMH * MS_PER_KMH
    fig, ax = plt.subplots(figsize=(9, 4.8))
    for key, m in MODELS.items():
        _, _, _, f_tot = resistance(vms, m['cd'], 0.0)
        ax.plot(V_KMH, power_kw(vms, f_tot), color=m['color'], lw=1.6, label=m['label'])
    ax.set_xlabel('speed (km/h)'); ax.set_ylabel('power (kW)')
    ax.set_title('Cruise drive power by drag coefficient (flat road)')
    ax.legend(fontsize=8); ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)


def plot_energy_by_grade(out_path):
    """不同坡度下百公里能耗随车速对比（普通车型）。"""
    vms = V_KMH * MS_PER_KMH
    fig, ax = plt.subplots(figsize=(9, 4.8))
    cmap = plt.cm.viridis(np.linspace(0.1, 0.85, len(GRADE_CHECK)))
    for g, c in zip(GRADE_CHECK, cmap):
        _, _, _, f_tot = resistance(vms, MODELS['normal']['cd'], g)
        ax.plot(V_KMH, energy_kwh_per_100km(f_tot), color=c, lw=1.6,
                label=f'grade {grade_label(g)}')
    ax.set_xlabel('speed (km/h)'); ax.set_ylabel('energy (kWh/100km)')
    ax.set_title('Energy consumption per 100 km by road grade')
    ax.legend(fontsize=8); ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)


def render_demo_frame(v_cur, out_size=(800, 420)):
    """渲染一帧：各车型功率曲线 + 当前车速标记。"""
    vms = V_KMH * MS_PER_KMH
    fig, ax = plt.subplots(figsize=(10, 4.6))
    for key, m in MODELS.items():
        _, _, _, f_tot = resistance(vms, m['cd'], 0.0)
        ax.plot(V_KMH, power_kw(vms, f_tot), color=m['color'], lw=1.4, label=m['label'])
    ax.axvline(v_cur, color='black', ls=':', lw=1.2)
    ax.set_xlim(0, 120); ax.set_ylim(0, None)
    ax.set_xlabel('speed (km/h)'); ax.set_ylabel('power (kW)')
    ax.set_title(f'Cruise power @ {v_cur:.0f} km/h')
    ax.legend(fontsize=8); ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.canvas.draw()
    buf = np.frombuffer(fig.canvas.buffer_rgba(), dtype=np.uint8)
    img = Image.fromarray(buf.reshape(fig.canvas.get_width_height()[::-1] + (4,)))
    plt.close(fig)
    return img.convert('RGB').resize(out_size)


# ---------- MuJoCo 车型场景静态帧（可选依赖） ----------
# 三车型并排斜俯视网格地面：轿车（蓝，低扁）、普通车型（绿）、SUV（红，高）；
# 车体体型仅用于可视化风阻外形差异，定量结论以阻力/功率/能耗曲线与指标表为准。
_GRID_LINES = ''.join(
    f'<geom type="box" size="0.008 6 0.001" pos="{i} 0 0.002" rgba="0.66 0.66 0.69 1"/>'
    f'<geom type="box" size="6 0.008 0.001" pos="0 {i} 0.002" rgba="0.66 0.66 0.69 1"/>'
    for i in range(-6, 7))

SCENE_XML = """<mujoco><compiler angle="radian"/>
<default><geom contype="0" conaffinity="0"/></default>
<worldbody>
<light name="key" dir="-0.35 -0.55 -1" diffuse="0.95 0.95 0.95" ambient="0.55 0.55 0.55"/>
<geom name="ground" type="plane" size="6 6 0.02" rgba="0.86 0.86 0.88 1"/>
""" + _GRID_LINES + """
<camera name="cam" mode="fixed" pos="0 -7 5.2" xyaxes="1 0 0 0 0.74 0.67" fovy="45"/>
<body name="car_sedan" pos="0 -2.0 0.14"><joint type="free"/>
<geom type="box" size="0.5 0.24 0.13" rgba="0.12 0.47 0.71 1"/>
<geom type="cylinder" size="0.08 0.05" pos="0.32 -0.26 0" euler="1.5708 0 0" rgba="0.1 0.1 0.1 1"/>
<geom type="cylinder" size="0.08 0.05" pos="0.32  0.26 0" euler="1.5708 0 0" rgba="0.1 0.1 0.1 1"/>
<geom type="cylinder" size="0.08 0.05" pos="-0.32 -0.26 0" euler="1.5708 0 0" rgba="0.1 0.1 0.1 1"/>
<geom type="cylinder" size="0.08 0.05" pos="-0.32  0.26 0" euler="1.5708 0 0" rgba="0.1 0.1 0.1 1"/>
</body>
<body name="car_normal" pos="0  0.0 0.16"><joint type="free"/>
<geom type="box" size="0.5 0.26 0.16" rgba="0.17 0.63 0.17 1"/>
<geom type="cylinder" size="0.08 0.05" pos="0.32 -0.28 0" euler="1.5708 0 0" rgba="0.1 0.1 0.1 1"/>
<geom type="cylinder" size="0.08 0.05" pos="0.32  0.28 0" euler="1.5708 0 0" rgba="0.1 0.1 0.1 1"/>
<geom type="cylinder" size="0.08 0.05" pos="-0.32 -0.28 0" euler="1.5708 0 0" rgba="0.1 0.1 0.1 1"/>
<geom type="cylinder" size="0.08 0.05" pos="-0.32  0.28 0" euler="1.5708 0 0" rgba="0.1 0.1 0.1 1"/>
</body>
<body name="car_suv" pos="0  2.0 0.22"><joint type="free"/>
<geom type="box" size="0.5 0.30 0.22" rgba="0.84 0.15 0.15 1"/>
<geom type="cylinder" size="0.08 0.05" pos="0.32 -0.32 0" euler="1.5708 0 0" rgba="0.1 0.1 0.1 1"/>
<geom type="cylinder" size="0.08 0.05" pos="0.32  0.32 0" euler="1.5708 0 0" rgba="0.1 0.1 0.1 1"/>
<geom type="cylinder" size="0.08 0.05" pos="-0.32 -0.32 0" euler="1.5708 0 0" rgba="0.1 0.1 0.1 1"/>
<geom type="cylinder" size="0.08 0.05" pos="-0.32  0.32 0" euler="1.5708 0 0" rgba="0.1 0.1 0.1 1"/>
</body>
</worldbody></mujoco>"""


def render_scene_png(out_path):
    """MuJoCo 离屏渲染单帧三车型场景静态图（网格地面 + 车体，斜俯视）。

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
    with open(csv_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(
            f, fieldnames=['grade_pct', 'speed_kmh', 'total_resistance_N',
                            'power_kW', 'energy_kwh_per_100km'])
        writer.writeheader()
        writer.writerows(rows)

    plot_drag_components(os.path.join(RESULTS_DIR, 'cruise_drag_components.png'))
    plot_power_by_model(os.path.join(RESULTS_DIR, 'cruise_power_model.png'))
    plot_energy_by_grade(os.path.join(RESULTS_DIR, 'cruise_energy_grade.png'))

    speeds = np.linspace(10.0, 120.0, 18)
    frames = [render_demo_frame(float(v)) for v in speeds]
    if frames:
        duration = max(80, int(1000 / len(frames) * 10))
        frames[0].save(os.path.join(RESULTS_DIR, 'cruise_energy_demo.gif'),
                       save_all=True, append_images=frames[1:],
                       duration=duration, loop=0, optimize=True)

    scene_png = render_scene_png(os.path.join(RESULTS_DIR, 'cruise_scene.png'))
    if scene_png:
        print(f"仿真场景静态图已写入 cruise_scene.png（{scene_png // 1024} KB）")

    for r in rows:
        print(f"grade={r['grade_pct']:+.0f}% v={r['speed_kmh']:.0f}km/h "
              f"F={r['total_resistance_N']}N P={r['power_kW']}kW "
              f"E={r['energy_kwh_per_100km']}kWh/100km")
    print(f'实验完成，结果保存在 {RESULTS_DIR} 目录')


if __name__ == '__main__':
    main()
