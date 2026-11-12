"""
无信号路口冲突时序蒙特卡洛仿真（纯 Python 数值仿真）

依赖: numpy, matplotlib, Pillow
使用: python main.py
说明: 模拟无信号十字交叉口两个方向的直行车，各自以带扰动的匀速驶向路口
      冲突点，按到达时间差判断是否落入安全时距阈值内构成冲突；通过蒙特卡洛
      仿真统计不同限速、不同让行延迟下的冲突概率，并绘制到达时间差直方图、
      冲突概率-限速曲线与位置-时间轨迹图。所有结果写入脚本同级 results/ 目录。
"""
import os
import csv

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image

RESULTS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'results')

# ---------- 仿真参数 ----------
DISTANCE = 40.0          # 两车到冲突点的距离 (m)
SPEED_SIGMA = 1.8        # 车速扰动标准差 (m/s)
V_MIN = 2.0              # 车速截断下限 (m/s)
T_SAFE = 2.0             # 安全时距阈值 (s)，|到达时差| 小于该值判为冲突
N_MC = 2000              # 蒙特卡洛样本数
SEED = 20261111
KMH = 1000.0 / 3600.0

# 固定对比情景：名称 -> (限速 km/h, B 车让行延迟 s)
SCENARIOS = {
    'speed_30':  (30.0, 0.0),
    'speed_50':  (50.0, 0.0),
    'yield_2s':  (50.0, 2.0),
    'yield_4s':  (50.0, 4.0),
}

SCEN_LABEL = {
    'speed_30': '30 km/h, no yield',
    'speed_50': '50 km/h, no yield',
    'yield_2s': '50 km/h, yield 2s',
    'yield_4s': '50 km/h, yield 4s',
}


def sample_arrival(rng, mean_kmh, delay_b, n):
    """采样两车到达冲突点时间，返回时间差 Δt = tA − tB (s)。"""
    mu = mean_kmh * KMH
    va = np.maximum(V_MIN, rng.normal(mu, SPEED_SIGMA, n))
    vb = np.maximum(V_MIN, rng.normal(mu, SPEED_SIGMA, n))
    ta = DISTANCE / va
    tb = DISTANCE / vb + delay_b
    return ta - tb


def conflict_probability(delta_t, t_safe=T_SAFE):
    """落入安全时距阈值内的样本比例。"""
    return float(np.mean(np.abs(delta_t) < t_safe))


def summarize(rng):
    """各固定情景的冲突指标。"""
    rows, deltas = [], {}
    for name, (mean_kmh, delay_b) in SCENARIOS.items():
        dt = sample_arrival(rng, mean_kmh, delay_b, N_MC)
        deltas[name] = dt
        rows.append({
            'scenario': name,
            'speed_kmh': mean_kmh,
            'yield_delay_s': delay_b,
            'conflict_prob': round(conflict_probability(dt), 4),
            'delta_mean_s': round(float(np.mean(dt)), 3),
            'delta_std_s': round(float(np.std(dt)), 3),
            'delta_p95_abs_s': round(float(np.percentile(np.abs(dt), 95)), 3),
        })
    return rows, deltas


# ---------- 绘图 ----------
def plot_histogram(deltas, out_path):
    """两种车速情景到达时间差直方图，标注安全时距带。"""
    fig, ax = plt.subplots(figsize=(9, 4.8))
    bins = np.linspace(-8, 8, 40)
    ax.hist(deltas['speed_30'], bins=bins, alpha=0.55, color='tab:blue',
            label=SCEN_LABEL['speed_30'], density=True)
    ax.hist(deltas['speed_50'], bins=bins, alpha=0.55, color='tab:red',
            label=SCEN_LABEL['speed_50'], density=True)
    ax.axvspan(-T_SAFE, T_SAFE, color='tab:red', alpha=0.12, label='safe-gap band')
    ax.axvline(-T_SAFE, color='tab:red', ls='--', lw=1)
    ax.axvline(T_SAFE, color='tab:red', ls='--', lw=1)
    ax.set_xlabel('arrival time difference Δt (s)')
    ax.set_ylabel('density')
    ax.set_title('Arrival time difference at conflict point')
    ax.legend(fontsize=8); ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)


def plot_probability_curve(rng, out_path):
    """冲突概率随限速变化：无让行 / 让行 2s / 让行 4s。"""
    speeds = np.arange(20.0, 61.0, 5.0)
    curves = {'no_yield': 0.0, 'yield_2s': 2.0, 'yield_4s': 4.0}
    colors = {'no_yield': 'tab:red', 'yield_2s': 'tab:green', 'yield_4s': 'tab:blue'}
    fig, ax = plt.subplots(figsize=(9, 4.8))
    for key, delay in curves.items():
        probs = []
        for sp in speeds:
            dt = sample_arrival(rng, sp, delay, N_MC)
            probs.append(conflict_probability(dt))
        ax.plot(speeds, probs, marker='o', color=colors[key], lw=1.6,
                label=f'yield {delay:.0f}s' if delay > 0 else 'no yield')
    ax.set_xlabel('speed limit (km/h)'); ax.set_ylabel('conflict probability')
    ax.set_title('Conflict probability vs speed limit')
    ax.set_ylim(0, 1.0); ax.legend(fontsize=8); ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)


def plot_spacetime(deltas_sample, out_path):
    """位置-时间轨迹图：base 情景若干样本，两车驶向冲突点。"""
    rng = np.random.default_rng(SEED + 7)
    mu = 50.0 * KMH
    n_show = 12
    va = np.maximum(V_MIN, rng.normal(mu, SPEED_SIGMA, n_show))
    vb = np.maximum(V_MIN, rng.normal(mu, SPEED_SIGMA, n_show))
    fig, ax = plt.subplots(figsize=(9, 5))
    t_line = np.linspace(0, DISTANCE / V_MIN, 60)
    for i in range(n_show):
        # A 车：位置从 DISTANCE 降到 0；B 车画在下方车道做区分
        ta = DISTANCE / va[i]
        pos_a = DISTANCE - va[i] * t_line[t_line <= ta]
        tt_a = t_line[t_line <= ta]
        ax.plot(tt_a, pos_a, color='tab:blue', lw=1.0, alpha=0.7)
        tb = DISTANCE / vb[i]
        pos_b = -(DISTANCE - vb[i] * t_line[t_line <= tb])
        tt_b = t_line[t_line <= tb]
        ax.plot(tt_b, pos_b, color='tab:orange', lw=1.0, alpha=0.7)
    ax.axhline(0, color='black', lw=1.4, label='conflict point')
    ax.set_xlabel('time (s)'); ax.set_ylabel('position (m)')
    ax.set_title('Space-time trajectories at unsignalized intersection')
    ax.legend(fontsize=8, loc='upper right'); ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)


def render_demo_frame(rng, v_cur, out_size=(800, 420)):
    """渲染一帧：冲突概率-限速曲线 + 当前限速标记。"""
    speeds = np.arange(20.0, 61.0, 5.0)
    fig, ax = plt.subplots(figsize=(10, 4.6))
    for key, delay in {'no_yield': 0.0, 'yield_2s': 2.0}.items():
        probs = [conflict_probability(sample_arrival(rng, sp, delay, N_MC))
                 for sp in speeds]
        ax.plot(speeds, probs, marker='o',
                color='tab:red' if delay == 0 else 'tab:green',
                lw=1.4, label='no yield' if delay == 0 else 'yield 2s')
    ax.axvline(v_cur, color='black', ls=':', lw=1.2)
    ax.set_xlim(20, 60); ax.set_ylim(0, 1.0)
    ax.set_xlabel('speed limit (km/h)'); ax.set_ylabel('conflict probability')
    ax.set_title(f'Conflict probability @ {v_cur:.0f} km/h')
    ax.legend(fontsize=8); ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.canvas.draw()
    buf = np.frombuffer(fig.canvas.buffer_rgba(), dtype=np.uint8)
    img = Image.fromarray(buf.reshape(fig.canvas.get_width_height()[::-1] + (4,)))
    plt.close(fig)
    return img.convert('RGB').resize(out_size)


# ---------- MuJoCo 路口双车场景静态帧（可选依赖） ----------
# 十字交叉口两个方向的直行车：A 车（蓝）纵向驶来，B 车（橙）横向驶来，
# 中心圆盘为冲突点；车体朝向与配色和时空轨迹图一致，定量结论以概率曲线与指标表为准。
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
<geom name="conflict" type="cylinder" size="0.4 0.005" pos="0 0 0.003" rgba="0.90 0.85 0.50 1"/>
<camera name="cam" mode="fixed" pos="0 -7 5.2" xyaxes="1 0 0 0 0.74 0.67" fovy="45"/>
<body name="car_a" pos="0 -3.0 0.16" euler="0 0 1.5708"><joint type="free"/>
<geom type="box" size="0.5 0.26 0.14" rgba="0.12 0.47 0.71 1"/>
<geom type="cylinder" size="0.08 0.05" pos="0.32 -0.28 0" euler="1.5708 0 0" rgba="0.1 0.1 0.1 1"/>
<geom type="cylinder" size="0.08 0.05" pos="0.32  0.28 0" euler="1.5708 0 0" rgba="0.1 0.1 0.1 1"/>
<geom type="cylinder" size="0.08 0.05" pos="-0.32 -0.28 0" euler="1.5708 0 0" rgba="0.1 0.1 0.1 1"/>
<geom type="cylinder" size="0.08 0.05" pos="-0.32  0.28 0" euler="1.5708 0 0" rgba="0.1 0.1 0.1 1"/>
</body>
<body name="car_b" pos="3.0 0 0.16" euler="0 0 0"><joint type="free"/>
<geom type="box" size="0.5 0.26 0.14" rgba="0.95 0.50 0.08 1"/>
<geom type="cylinder" size="0.08 0.05" pos="0.32 -0.28 0" euler="1.5708 0 0" rgba="0.1 0.1 0.1 1"/>
<geom type="cylinder" size="0.08 0.05" pos="0.32  0.28 0" euler="1.5708 0 0" rgba="0.1 0.1 0.1 1"/>
<geom type="cylinder" size="0.08 0.05" pos="-0.32 -0.28 0" euler="1.5708 0 0" rgba="0.1 0.1 0.1 1"/>
<geom type="cylinder" size="0.08 0.05" pos="-0.32  0.28 0" euler="1.5708 0 0" rgba="0.1 0.1 0.1 1"/>
</body>
</worldbody></mujoco>"""


def render_scene_png(out_path):
    """MuJoCo 离屏渲染单帧路口双车场景静态图（网格地面 + 冲突点，斜俯视）。

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
    rng = np.random.default_rng(SEED)
    rows, deltas = summarize(rng)

    csv_path = os.path.join(RESULTS_DIR, 'metrics.csv')
    with open(csv_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(
            f, fieldnames=['scenario', 'speed_kmh', 'yield_delay_s',
                           'conflict_prob', 'delta_mean_s', 'delta_std_s',
                           'delta_p95_abs_s'])
        writer.writeheader()
        writer.writerows(rows)

    plot_histogram(deltas, os.path.join(RESULTS_DIR, 'conflict_histogram.png'))
    plot_probability_curve(np.random.default_rng(SEED + 2),
                           os.path.join(RESULTS_DIR, 'conflict_probability.png'))
    plot_spacetime(deltas['speed_50'],
                   os.path.join(RESULTS_DIR, 'conflict_spacetime.png'))

    speeds = np.linspace(20.0, 60.0, 18)
    frames = [render_demo_frame(np.random.default_rng(SEED + 3 + i), float(v))
              for i, v in enumerate(speeds)]
    if frames:
        duration = max(80, int(1000 / len(frames) * 10))
        frames[0].save(os.path.join(RESULTS_DIR, 'conflict_demo.gif'),
                       save_all=True, append_images=frames[1:],
                       duration=duration, loop=0, optimize=True)

    scene_png = render_scene_png(os.path.join(RESULTS_DIR, 'conflict_scene.png'))
    if scene_png:
        print(f"仿真场景静态图已写入 conflict_scene.png（{scene_png // 1024} KB）")

    for r in rows:
        print(f"{r['scenario']:10s} P(conflict)={r['conflict_prob']:.3f} "
              f"Δt mean={r['delta_mean_s']:.2f} std={r['delta_std_s']:.2f}")
    print(f'实验完成，结果保存在 {RESULTS_DIR} 目录')


if __name__ == '__main__':
    main()
