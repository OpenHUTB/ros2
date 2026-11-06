# -*- coding: utf-8 -*-
"""MuJoCo 世界：差速小车 + 圆形赛道 + 限速牌 + 车载/俯视相机。

模型在运行时按场景生成 MJCF 字符串并编译：地面与牌面纹理由 PIL 绘制后
以内存资产注入，不依赖外部素材文件。控制上 /cmd_vel 的 (v, w) 按差速
关系换算为左右驱动轮的目标轮速，由速度 actuator 跟踪；物理推进调用
mj_step（每控制周期 10 个 2 ms 刚体动力学子步）。
"""
import io
import math
import os

os.environ.setdefault("MUJOCO_GL", "osmesa")  # 无头环境软件渲染，可被外部 MUJOCO_GL 覆盖

import cv2
from PIL import Image, ImageDraw

from . import scene as S
from .sign_render import draw_limit_sign

# ---- 几何常量（与 MJCF 模板保持一致） ----
GROUND_HALF_M = 5.0        # 地面平面半宽（m）
TOP_RES = 480              # 俯视相机渲染分辨率（方形）
WHEEL_R = 0.035            # 驱动轮半径（m）
HALF_TRACK = 0.095         # 轮距之半（m）
WHEEL_V_MAX = 60.0         # 轮速上限（rad/s）
CAM_FOVY_DEG = 73.74       # 车载相机垂直视场角（4:3 分辨率下水平恰为 90°）
N_SUBSTEPS = 10            # 每控制周期物理子步数（0.002 s x 10 = 20 ms）
# 世界坐标与 scene.py 对齐：赛道圆心 (CX, CY)，地面平面以圆心为中心
CX = S.TRACK_CENTER[0] / S.PX_PER_M
CY = S.TRACK_CENTER[1] / S.PX_PER_M


def _quat_mul(a, b):
    """四元数乘法 (w, x, y, z)，结果表示先施加 b 再施加 a。"""
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return (w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2)


def _mat2quat(r):
    """旋转矩阵（行主序 3x3，列 = 局部轴的世界方向）-> 四元数 (w, x, y, z)。"""
    tr = r[0][0] + r[1][1] + r[2][2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        return (0.25 * s, (r[2][1] - r[1][2]) / s, (r[0][2] - r[2][0]) / s,
                (r[1][0] - r[0][1]) / s)
    if r[0][0] > r[1][1] and r[0][0] > r[2][2]:
        s = math.sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]) * 2.0
        return ((r[2][1] - r[1][2]) / s, 0.25 * s, (r[0][1] + r[1][0]) / s,
                (r[0][2] + r[2][0]) / s)
    if r[1][1] > r[2][2]:
        s = math.sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]) * 2.0
        return ((r[0][2] - r[2][0]) / s, (r[0][1] + r[1][0]) / s, 0.25 * s,
                (r[1][2] + r[2][1]) / s)
    s = math.sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]) * 2.0
    return ((r[1][0] - r[0][1]) / s, (r[0][2] + r[2][0]) / s,
            (r[1][2] + r[2][1]) / s, 0.25 * s)


def _face_quat(nx, ny, roll_deg):
    """牌面朝向四元数。

    牌面局部 +z（法线）指向赛道中心方向 (nx, ny, 0)，局部 +y 保持竖直，
    使牌面文字对观察者竖直可读；再绕法线施加 roll_deg 的侧倾退化。
    """
    # 列向量：局部 x = u×n（观察者右侧），局部 y = u（竖直），局部 z = n
    r = [[-ny, 0.0, nx],
         [nx, 0.0, ny],
         [0.0, 1.0, 0.0]]
    q_frame = _mat2quat(r)
    rho = math.radians(roll_deg) / 2.0
    q_roll = (math.cos(rho), nx * math.sin(rho), ny * math.sin(rho), 0.0)
    return _quat_mul(q_roll, q_frame)


def make_ground_texture(res=1000):
    """绘制俯视赛道纹理：草地底色 + 沥青环道 + 路缘线 + 中央虚线。"""
    ppm = res / (2.0 * GROUND_HALF_M)              # 像素/米
    img = Image.new("RGB", (res, res), (168, 192, 148))
    d = ImageDraw.Draw(img)
    c = res / 2.0
    r = S.TRACK_RADIUS / S.PX_PER_M * ppm          # 赛道中心半径
    half = 0.45 * ppm                              # 路面半宽
    box = lambda rr: [c - rr, c - rr, c + rr, c + rr]
    d.ellipse(box(r + half), fill=(70, 70, 74))    # 沥青环道
    d.ellipse(box(r - half), fill=(168, 192, 148))
    d.ellipse(box(r + half), outline=(110, 112, 118), width=2)
    d.ellipse(box(r - half), outline=(110, 112, 118), width=2)
    for a in range(0, 360, 18):                    # 中央虚线
        x = c + r * math.cos(math.radians(a))
        y = c + r * math.sin(math.radians(a))
        s2 = 0.03 * ppm
        d.rectangle([x - s2, y - s2, x + s2, y + s2], fill=(225, 225, 225))
    return img


def _png_bytes(img):
    buf = io.BytesIO()
    img.save(buf, "PNG")
    return buf.getvalue()


def build_model(signs, start_angle_deg):
    """按场景生成 (MJCF 文本, 纹理资产 dict)。"""
    assets = {"ground.png": _png_bytes(make_ground_texture())}

    # ---- 限速牌：牌面纹理 + 立杆 + 朝向赛道中心的牌面平面 ----
    sign_assets, sign_bodies = [], []
    for i, s in enumerate(signs):
        fname = f"sign_{i}_{s.value_kmh}.png"
        assets[fname] = _png_bytes(draw_limit_sign(s.value_kmh, 256).convert("RGB"))
        sx, sy = S.sign_xy(s.angle_deg)
        # 牌面朝向：面向沿赛道来车方向 2.0 m 处的轨道点，使检测中段（1~3.5 m）
        # 视角接近正面（旧 billboard 渲染的等效效果），斜视细笔画不致糊化
        r_track = S.TRACK_RADIUS / S.PX_PER_M
        aim = s.angle_deg - math.degrees(2.0 / r_track)
        ax = CX + r_track * math.cos(math.radians(aim))
        ay = CY + r_track * math.sin(math.radians(aim))
        n = math.hypot(ax - sx, ay - sy)
        nx, ny = (ax - sx) / n, (ay - sy) / n   # 牌面法线
        qw, qx, qy, qz = _face_quat(nx, ny,
                                    float((s.degrade or {}).get("rotate", 0.0)))
        sign_assets.append(
            f'<texture name="tex_sign{i}" type="2d" file="{fname}"/>'
            f'<material name="mat_sign{i}" texture="tex_sign{i}" texrepeat="1 1"/>')
        sign_bodies.append(
            f'<body name="sign{i}" pos="{sx:.4f} {sy:.4f} 0">'
            f'<geom name="pole{i}" type="cylinder" size="0.02 0.175" '
            f'pos="{-nx * 0.04:.4f} {-ny * 0.04:.4f} 0.175" rgba="0.45 0.47 0.5 1" '
            f'contype="0" conaffinity="0"/>'
            f'<body pos="0 0 {S.SIGN_CENTER_H_M:.3f}" '
            f'quat="{qw:.6f} {qx:.6f} {qy:.6f} {qz:.6f}">'
            f'<geom name="face{i}" type="box" '
            f'size="{S.SIGN_PHYS_M / 2:.3f} {S.SIGN_PHYS_M / 2:.3f} 0.005" '
            f'material="mat_sign{i}" contype="0" conaffinity="0"/>'
            f'</body></body>')

    # ---- 小车初始位姿：赛道上 start_angle_deg 处，沿角度增大方向的切向 ----
    a0 = math.radians(start_angle_deg)
    r0 = S.TRACK_RADIUS / S.PX_PER_M
    x0, y0 = CX + r0 * math.cos(a0), CY + r0 * math.sin(a0)
    th0 = a0 + math.pi / 2.0
    qw, qz = math.cos(th0 / 2.0), math.sin(th0 / 2.0)

    xml = f"""<mujoco model="traffic_sign_speed_control">
  <compiler angle="degree" autolimits="true"/>
  <option timestep="0.002" integrator="implicitfast"/>
  <default><geom friction="1.0 0.005 0.0001"/></default>
  <asset>
    <texture name="tex_sky" type="skybox" builtin="gradient"
             rgb1="0.55 0.72 0.92" rgb2="0.85 0.92 0.98" width="256" height="256"/>
    <texture name="tex_ground" type="2d" file="ground.png"/>
    <material name="mat_ground" texture="tex_ground" texrepeat="1 1"/>
    {''.join(sign_assets)}
  </asset>
  <worldbody>
    <light directional="true" pos="{CX} {CY} 6" dir="0 0 -1" diffuse="0.7 0.7 0.7"
           ambient="0.55 0.55 0.55" specular="0.1 0.1 0.1"/>
    <geom name="ground" type="plane" pos="{CX} {CY} 0"
          size="{GROUND_HALF_M} {GROUND_HALF_M} 0.05" material="mat_ground"/>
    <camera name="overview" pos="{CX} {CY} 8.5" fovy="61"/>
    {''.join(sign_bodies)}
    <body name="base" pos="{x0:.4f} {y0:.4f} 0.035" quat="{qw:.6f} 0 0 {qz:.6f}">
      <freejoint name="root"/>
      <camera name="onboard" pos="0.12 0 0.165" xyaxes="0 -1 0 0 0 1" fovy="{CAM_FOVY_DEG}"/>
      <geom name="chassis" type="box" size="0.15 0.09 0.025" pos="0 0 0.025"
            mass="2.0" rgba="0.80 0.16 0.16 1"/>
      <geom name="caster" type="sphere" size="0.02" pos="0.12 0 -0.015" mass="0.05"
            rgba="0.1 0.1 0.1 1" friction="0.02 0.005 0.0001"/>
      <body name="wheel_left" pos="-0.02 0.095 0">
        <joint name="j_left" type="hinge" axis="0 1 0" armature="0.01" damping="0.002"/>
        <geom name="g_wheel_left" type="cylinder" size="{WHEEL_R} 0.012"
              axisangle="1 0 0 90" mass="0.1" rgba="0.08 0.08 0.08 1"
              friction="1.5 0.008 0.0002"/>
      </body>
      <body name="wheel_right" pos="-0.02 -0.095 0">
        <joint name="j_right" type="hinge" axis="0 1 0" armature="0.01" damping="0.002"/>
        <geom name="g_wheel_right" type="cylinder" size="{WHEEL_R} 0.012"
              axisangle="1 0 0 90" mass="0.1" rgba="0.08 0.08 0.08 1"
              friction="1.5 0.008 0.0002"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <velocity name="a_left" joint="j_left" kv="1.0" ctrlrange="-{WHEEL_V_MAX} {WHEEL_V_MAX}"/>
    <velocity name="a_right" joint="j_right" kv="1.0" ctrlrange="-{WHEEL_V_MAX} {WHEEL_V_MAX}"/>
  </actuator>
</mujoco>
"""
    return xml, assets


class MjWorld:
    """MuJoCo 仿真封装：物理推进 + 车载/俯视相机渲染。"""

    def __init__(self, signs, start_angle_deg=180.0):
        import mujoco
        self._mj = mujoco
        xml, assets = build_model(signs, start_angle_deg)
        self.model = mujoco.MjModel.from_xml_string(xml, assets)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
        self.sim_dt = self.model.opt.timestep * N_SUBSTEPS   # 每控制周期仿真时长
        self._cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA,
                                         "onboard")
        self._top_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA,
                                         "overview")
        self._cam_renderer = None
        self._top_renderer = None

    def reset(self):
        """复位到 MJCF 中的初始位姿（速度清零）。"""
        self._mj.mj_resetData(self.model, self.data)
        self._mj.mj_forward(self.model, self.data)

    def pose(self):
        """返回 (x, y, theta)，theta 为偏航角（rad）。"""
        q = self.data.qpos
        qw, qx, qy, qz = q[3], q[4], q[5], q[6]
        theta = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        return float(q[0]), float(q[1]), theta

    def set_cmd(self, v, w):
        """差速运动学换算：将 (线速度, 角速度) 写入左右轮速度 actuator。"""
        wl = (v - w * HALF_TRACK) / WHEEL_R
        wr = (v + w * HALF_TRACK) / WHEEL_R
        self.data.ctrl[0] = max(-WHEEL_V_MAX, min(WHEEL_V_MAX, wl))
        self.data.ctrl[1] = max(-WHEEL_V_MAX, min(WHEEL_V_MAX, wr))

    def step(self):
        """推进一个控制周期（N_SUBSTEPS 个物理子步）。"""
        for _ in range(N_SUBSTEPS):
            self._mj.mj_step(self.model, self.data)

    def camera_frame(self):
        """渲染车载前视相机，返回 uint8 (CAM_H, CAM_W, 3) RGB。

        以 2 倍分辨率渲染后 INTER_AREA 降采样，避免远处牌面纹理
        缩采样时数字笔画糊化（字符切分依赖细笔画完整性）。
        """
        if self._cam_renderer is None:
            self._cam_renderer = self._mj.Renderer(
                self.model, 2 * S.CAM_H, 2 * S.CAM_W)
        self._cam_renderer.update_scene(self.data, camera=self._cam_id)
        big = self._cam_renderer.render()
        return cv2.resize(big, (S.CAM_W, S.CAM_H),
                          interpolation=cv2.INTER_AREA)

    def topdown_frame(self):
        """渲染顶部俯视相机，返回 uint8 (TOP_RES, TOP_RES, 3) RGB。"""
        if self._top_renderer is None:
            self._top_renderer = self._mj.Renderer(self.model, TOP_RES, TOP_RES)
        self._top_renderer.update_scene(self.data, camera=self._top_id)
        return self._top_renderer.render()

    def close(self):
        """释放渲染器（GL 上下文）。"""
        for attr in ("_cam_renderer", "_top_renderer"):
            r = getattr(self, attr)
            if r is not None:
                try:
                    r.close()
                except Exception:  # noqa: BLE001
                    pass
                setattr(self, attr, None)
