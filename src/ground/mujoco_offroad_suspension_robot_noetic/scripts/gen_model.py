#!/usr/bin/env python3
"""生成 models/suspension_robot.xml + 程序贴图 soil.png / rock.png。

用法：
    python3 scripts/gen_model.py
改地形后需重跑本脚本：XML 中石头与目标旗标的高度按新地形重新计算。
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from terrain import elevation, slope  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "models", "suspension_robot.xml")

# 高度场：半边长 60 m × 40 m，480 格 → 单格 0.25 m（远小于 0.44 m 轴距）
HF_SIZE_X, HF_SIZE_Y, HF_SIZE_Z = 60.0, 40.0, 0.60
HF_NROW, HF_NCOL = 480, 480

# 石头：(x, y, 半径, 是否参与碰撞)
ROCKS = [
    (-7.5, 5.6, 0.55, True),
    (-3.0, 7.2, 0.40, True),
    (5.5, 6.8, 0.62, True),
    (11.0, 8.3, 0.45, True),
    (17.5, 5.5, 0.58, True),
    (9.0, -6.0, 0.50, True),
    (15.0, -6.8, 0.42, True),
    (2.0, -7.0, 0.60, True),
    (-6.0, -5.0, 0.38, True),
    (21.0, 4.5, 0.35, False),
    (24.0, -4.0, 0.45, False),
    (-11.0, 9.0, 0.70, False),
    (-14.0, -8.0, 0.50, False),
    (4.0, 12.0, 0.65, False),
    (14.0, 11.0, 0.55, False),
]

GOAL = (18.0, 2.0)


# ---------------------------------------------------------------- 贴图生成
def _tileable_blur_noise(N, rng, sigma):
    """白噪声 + FFT 高斯模糊：频域即周期边界，天然无缝可平铺。"""
    white = rng.random((N, N)).astype(np.float32)
    ky = np.fft.fftfreq(N)[:, None]
    kx = np.fft.fftfreq(N)[None, :]
    H = np.exp(-2.0 * (np.pi * sigma) ** 2 * (kx * kx + ky * ky))
    out = np.real(np.fft.ifft2(np.fft.fft2(white) * H)).astype(np.float32)
    out -= out.min()
    return out / (out.max() + 1e-9)


def _tileable_noise(N, rng, sigma_ratio=0.06, octaves=3):
    out = np.zeros((N, N), dtype=np.float32)
    for k in range(octaves):
        sigma = N * sigma_ratio * (2.4 ** k)
        out += _tileable_blur_noise(N, rng, sigma) * (0.75 ** k)
    out -= out.min()
    return out / (out.max() + 1e-9)


def write_textures(out_dir=None):
    """生成 soil.png / rock.png。out_dir 默认 <pkg>/models，装到 share 后可显式传目录。"""
    from PIL import Image
    rng = np.random.default_rng(20261109)
    N = 512
    out_dir = out_dir or os.path.join(ROOT, "models")
    os.makedirs(out_dir, exist_ok=True)

    # --- 土壤 ---
    blotch = _tileable_noise(N, rng, sigma_ratio=0.020, octaves=3)      # 大块色差
    grit = _tileable_blur_noise(N, rng, 0.8)                            # 砂砾
    grit2 = _tileable_blur_noise(N, rng, 2.2)                           # 小碎石簇
    f = 0.42 + 0.26 * (blotch - 0.5) + 0.34 * (grit - 0.5) + 0.16 * (grit2 - 0.5)
    f = np.clip(f, 0, 1)
    dark = np.array([0.300, 0.262, 0.205], dtype=np.float32)
    light = np.array([0.545, 0.470, 0.352], dtype=np.float32)
    soil = dark * (1 - f[..., None]) + light * f[..., None]
    for _ in range(420):
        cx, cy = rng.integers(0, N, 2)
        r = int(rng.integers(1, 3))
        shade = float(rng.uniform(0.32, 0.66))
        col = np.array([shade * 0.99, shade * 0.93, shade * 0.83], dtype=np.float32)
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy <= r * r:
                    soil[(cy + dy) % N, (cx + dx) % N] = col
    Image.fromarray((np.clip(soil, 0, 1) * 255).astype(np.uint8)).save(
        os.path.join(out_dir, "soil.png"))

    # --- 岩石 ---
    rf = np.clip(_tileable_noise(N, rng, 0.02, 4) + rng.normal(0, 0.035, (N, N)), 0, 1)
    rock = np.repeat((0.20 + 0.30 * rf)[..., None], 3, axis=2).astype(np.float32)
    rock[..., 0] *= 1.03
    rock[..., 2] *= 0.95
    for _ in range(520):
        cx, cy = rng.integers(0, N, 2)
        r = int(rng.integers(1, 3))
        shade = float(rng.uniform(0.40, 0.72))
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy <= r * r:
                    rock[(cy + dy) % N, (cx + dx) % N] = shade
    Image.fromarray((np.clip(rock, 0, 1) * 255).astype(np.uint8)).save(
        os.path.join(out_dir, "rock.png"))


# ---------------------------------------------------------------- XML 片段
def rock_xml():
    items = []
    for i, (x, y, r, collide) in enumerate(ROCKS):
        z = float(elevation(x, y)) + 0.52 * r
        _, dy = slope(x, y)
        tilt = max(-0.22, min(0.22, float(dy) * 0.5))
        mask = "" if collide else ' contype="0" conaffinity="0"'
        items.append(
            f'    <geom name="rock_{i}" type="ellipsoid" pos="{x:.2f} {y:.2f} {z:.3f}" '
            f'size="{r:.2f} {0.82 * r:.2f} {0.68 * r:.2f}" '
            f'euler="0 {tilt:.3f} {i * 0.7:.3f}" material="rock_mat" '
            f'friction="1.5 0.02 0.1" mass="0"{mask}/>'
        )
    return "\n".join(items)


def goal_xml():
    x, y = GOAL
    gz = float(elevation(x, y))
    # 直接放在 worldbody 里（和石头一样），不要套 <body>：
    # MuJoCo 3.1.1 对"全 mass=0 的 body"会报 "body mass is too small"，
    # 而 worldbody 下的纯视觉 geom 不受此限。
    return f'''    <!-- 目标点：纯视觉标记，碰撞关闭，避免车被旗杆挡住 -->
    <geom name="goal_pad" type="cylinder" size="0.42 0.012" pos="{x:.2f} {y:.2f} {gz + 0.012:.3f}"
          rgba="0.95 0.72 0.15 0.55" contype="0" conaffinity="0" mass="0"/>
    <geom name="goal_pole" type="cylinder" size="0.020 0.55" pos="{x:.2f} {y:.2f} {gz + 0.57:.3f}"
          rgba="0.86 0.86 0.88 1" contype="0" conaffinity="0" mass="0"/>
    <geom name="goal_flag" type="box" size="0.19 0.004 0.11" pos="{x + 0.20:.2f} {y:.2f} {gz + 1.01:.3f}"
          rgba="0.90 0.18 0.15 1" contype="0" conaffinity="0" mass="0"/>'''


def wheel_lugs(side):
    """胎纹：沿轮缘的 10 个切向胎块。"""
    out = []
    for k in range(10):
        th = 2 * np.pi * k / 10
        r = 0.134
        out.append(
            f'          <geom name="lug_{side}_{k}" type="box" '
            f'size="0.010 0.060 0.013" pos="{r * np.cos(th):.4f} 0 {r * np.sin(th):.4f}" '
            f'euler="0 {-th:.3f} 0" material="tire_mat" mass="0.010"/>'
        )
    return "\n".join(out)


def suspension(side, sx, sy):
    """单侧悬架：上摇臂 + 减振器外筒 + 轮胎总成。

    side: fl/fr/rl/rr；sx: 前后位置；sy: 左右符号(+1 左 / -1 右)
    """
    y_arm = -0.098 * sy
    return f'''      <body name="sus_{side}" pos="{sx} {sy * 0.270} 0.02">
        <joint name="sus_j_{side}" type="slide" axis="0 0 -1" range="-0.11 0.09"
               stiffness="380" springref="0" damping="30" armature="0.02"/>
        <geom name="arm_{side}" type="capsule" fromto="0 {y_arm:.3f} 0.055 0 0 0"
              size="0.020" material="arm_mat" mass="0.05"/>
        <geom name="shock_{side}" type="cylinder"
              fromto="0 {y_arm:.3f} 0.135 0 {y_arm:.3f} 0.055"
              size="0.020" material="shock_mat" mass="0.03"/>
        <body name="wheel_{side}" pos="0 0 0">
          <joint name="drive_{side}" type="hinge" axis="0 1 0" damping="0.30" armature="0.01"/>
          <geom name="tire_{side}" type="cylinder" size="0.14 0.062" euler="1.5708 0 0"
                material="tire_mat" mass="0.34" friction="1.45 0.02 0.18"/>
{wheel_lugs(side)}
          <geom name="hub_{side}" type="cylinder" size="0.050 0.066" euler="1.5708 0 0"
                material="hub_mat" mass="0.05"/>
        </body>
      </body>'''


XML = f'''<mujoco model="offroad_suspension_robot">
  <!-- 四轮独立悬架越野小车：滑移转向 + 高度场起伏地形 + IMU 姿态反馈 -->
  <compiler angle="radian" inertiafromgeom="true" autolimits="true"/>
  <option timestep="0.001" gravity="0 0 -9.81" integrator="implicitfast"
          cone="elliptic" impratio="8" density="1.0" viscosity="0.0"/>

  <visual>
    <!-- 离屏渲染尺寸上限由 offwidth/offheight 决定；skybox 在 asset 中定义后自动作为背景 -->
    <global offwidth="1280" offheight="720" fovy="46"/>
    <quality shadowsize="2048" offsamples="4"/>
    <headlight ambient="0.26 0.27 0.30" diffuse="0.52 0.50 0.47" specular="0.10 0.10 0.10"/>
    <map znear="0.001" zfar="200" haze="0.0"/>
  </visual>

  <asset>
    <texture name="soil" type="2d" file="soil.png"/>
    <material name="soil_mat" texture="soil" texrepeat="34 34" texuniform="true"
              reflectance="0.01" shininess="0.0"/>
    <texture name="rock" type="2d" file="rock.png"/>
    <material name="rock_mat" texture="rock" texrepeat="1 1" texuniform="true"
              reflectance="0.03" shininess="0.05"/>
    <texture name="sky" type="skybox" builtin="gradient" width="512" height="3072"
             rgb1="0.50 0.66 0.86" rgb2="0.88 0.91 0.94"/>
    <texture name="metal" type="2d" builtin="flat" width="64" height="64" rgb1="0.62 0.64 0.66"/>
    <material name="metal_mat" texture="metal" reflectance="0.28" shininess="0.4"/>
    <material name="shock_mat" rgba="0.85 0.55 0.10 1" reflectance="0.25" shininess="0.45"/>
    <material name="chassis_mat" rgba="0.10 0.48 0.40 1" reflectance="0.20" shininess="0.35"/>
    <material name="deck_mat" rgba="0.16 0.19 0.22 1" reflectance="0.12" shininess="0.25"/>
    <material name="tire_mat" rgba="0.075 0.075 0.085 1" reflectance="0.03" shininess="0.08"/>
    <material name="hub_mat" rgba="0.24 0.25 0.27 1" reflectance="0.18" shininess="0.35"/>
    <material name="arm_mat" rgba="0.52 0.54 0.57 1" reflectance="0.25" shininess="0.45"/>
    <material name="lamp_mat" rgba="1.0 0.92 0.55 1" reflectance="0.55" shininess="0.7"/>
  </asset>

  <worldbody>
    <light name="sun" pos="12 -16 22" dir="-0.42 0.55 -1" directional="true"
           diffuse="0.80 0.76 0.68" specular="0.28 0.28 0.26" castshadow="true"/>
    <light name="fill" pos="-14 12 14" dir="0.5 -0.4 -1" directional="false"
           diffuse="0.24 0.26 0.30" specular="0.06 0.06 0.06"/>

    <!-- 起伏地形：480×480 高度场，单格 0.25 m，米制起伏 ±0.25 m -->
    <geom name="terrain" type="hfield" hfield="offroad"
          pos="0 0 0" rgba="0.88 0.88 0.88 1" material="soil_mat"
          friction="1.2 0.02 0.15"/>

{rock_xml()}

{goal_xml()}

    <camera name="overview" pos="-8 -16 9" xyaxes="0.86 0.51 0 -0.32 0.54 0.78"/>

    <body name="base_link" pos="0 0 0.42">
      <freejoint name="base_free"/>
      <!-- 窄车身 + 宽轮距，让悬架摇臂露出来 -->
      <geom name="chassis" type="box" size="0.30 0.118 0.070" pos="0 0 0.135"
            material="chassis_mat" mass="2.30"/>
      <geom name="deck" type="box" size="0.215 0.105 0.028" pos="0 0 0.226"
            material="deck_mat" mass="0.22"/>
      <geom name="bumper_f" type="box" size="0.028 0.116 0.024" pos="0.318 0 0.112"
            material="metal_mat" mass="0.05"/>
      <geom name="bumper_r" type="box" size="0.028 0.116 0.024" pos="-0.318 0 0.112"
            material="metal_mat" mass="0.05"/>
      <geom name="cage_bar" type="capsule" fromto="0.02 -0.092 0.286 0.02 0.092 0.286"
            size="0.011" material="metal_mat" mass="0.03"/>
      <geom name="cage_post_l" type="capsule" fromto="0.02 0.092 0.186 0.02 0.092 0.286"
            size="0.009" material="metal_mat" mass="0.02"/>
      <geom name="cage_post_r" type="capsule" fromto="0.02 -0.092 0.186 0.02 -0.092 0.286"
            size="0.009" material="metal_mat" mass="0.02"/>
      <geom name="sensor_pod" type="box" size="0.055 0.050 0.038" pos="-0.10 0 0.285"
            material="deck_mat" mass="0.12"/>
      <geom name="antenna" type="capsule" fromto="-0.12 -0.092 0.32 -0.12 0.092 0.32"
            size="0.007" material="metal_mat" mass="0.01"/>
      <geom name="lamp_l" type="cylinder" size="0.030 0.010" pos="0.305 0.075 0.175"
            euler="0 1.5708 0" material="lamp_mat" mass="0.008"/>
      <geom name="lamp_r" type="cylinder" size="0.030 0.010" pos="0.305 -0.075 0.175"
            euler="0 1.5708 0" material="lamp_mat" mass="0.008"/>
      <site name="imu_site" pos="0 0 0.14" size="0.02" rgba="1 0.2 0.2 0.25"/>

{suspension("fl", 0.225, 1)}
{suspension("fr", 0.225, -1)}
{suspension("rl", -0.225, 1)}
{suspension("rr", -0.225, -1)}

    </body>
  </worldbody>

  <!-- 轮速伺服：ctrl 即目标轮角速度(rad/s) -->
  <actuator>
    <velocity name="wheel_fl_vel" joint="drive_fl" kv="7" ctrlrange="-9 9"/>
    <velocity name="wheel_fr_vel" joint="drive_fr" kv="7" ctrlrange="-9 9"/>
    <velocity name="wheel_rl_vel" joint="drive_rl" kv="7" ctrlrange="-9 9"/>
    <velocity name="wheel_rr_vel" joint="drive_rr" kv="7" ctrlrange="-9 9"/>
  </actuator>

  <sensor>
    <gyro name="gyro" site="imu_site"/>
    <accelerometer name="accel" site="imu_site"/>
    <framepos name="frame_pos" objtype="site" objname="imu_site"/>
    <framequat name="frame_quat" objtype="site" objname="imu_site"/>
    <jointpos name="sus_fl_pos" joint="sus_j_fl"/>
    <jointpos name="sus_fr_pos" joint="sus_j_fr"/>
    <jointpos name="sus_rl_pos" joint="sus_j_rl"/>
    <jointpos name="sus_rr_pos" joint="sus_j_rr"/>
  </sensor>
</mujoco>
'''

XML = XML.replace('  </asset>', f'''    <hfield name="offroad" nrow="{HF_NROW}" ncol="{HF_NCOL}"
           size="{HF_SIZE_X} {HF_SIZE_Y} {HF_SIZE_Z} 0.02"/>
  </asset>''')

if __name__ == "__main__":
    write_textures()
    with open(OUT, "w", encoding="utf-8") as f:
        f.write(XML)
    print(f"written: {OUT}")
    print(f"hfield: {HF_NROW}x{HF_NCOL}, size={HF_SIZE_X}x{HF_SIZE_Y}x{HF_SIZE_Z}, "
          f"cell={2 * HF_SIZE_X / HF_NCOL:.3f} m")
