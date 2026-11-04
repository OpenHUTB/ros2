# -*- coding: utf-8 -*-
"""MuJoCo 三自由度机械臂空间螺旋线仿真（单文件，XML 内嵌）

功能：三自由度机械臂（肩方位 yaw + 肩俯仰 shoulder + 肘俯仰 elbow）的末端
在 3D 空间中沿圆柱螺旋线运动（水平画圆 + 垂直匀速上升），
用解析空间 IK 实时解算三个关节角，PD 位置控制驱动关节，
渲染每帧动画并合成 demo_arm3d.gif 效果图。

运行（虚拟机 / 无显卡环境）：
    pip install mujoco pillow numpy
    MUJOCO_GL=osmesa python3 mujoco_arm_3d_single.py

输出：同目录 demo_arm3d.gif
"""
import os, math

os.environ.setdefault("MUJOCO_GL", "glfw")
import numpy as np
import mujoco
from PIL import Image, ImageDraw

# ============ 模型（XML 内嵌） ============
MODEL_XML = r"""
<mujoco model="arm3d">
  <compiler angle="radian"/>
  <option timestep="0.002" gravity="0 0 -9.81"/>
  <visual>
    <headlight ambient="0.6 0.6 0.6" diffuse="0.4 0.4 0.4"/>
  </visual>
  <worldbody>
    <light name="l1" diffuse="0.75 0.75 0.75" pos="0.6 0.6 1.4" dir="-0.4 -0.4 -1" castshadow="false"/>
    <light name="l2" diffuse="0.35 0.35 0.4" pos="-0.8 -0.6 1.2" dir="0.6 0.4 -1" castshadow="false"/>
    <geom name="floor" type="plane" size="1.2 1.2 0.02" rgba="0.88 0.88 0.9 1"/>
    <!-- 参考圆：螺旋线在地面的投影（16 个绿点） -->
    <body name="ref" pos="0 0 0.001">
      <geom name="rc0"  type="sphere" size="0.012" pos="0.15 0.000 0" rgba="0.1 0.8 0.3 1"/>
      <geom name="rc1"  type="sphere" size="0.012" pos="0.138 0.057 0" rgba="0.1 0.8 0.3 1"/>
      <geom name="rc2"  type="sphere" size="0.012" pos="0.106 0.106 0" rgba="0.1 0.8 0.3 1"/>
      <geom name="rc3"  type="sphere" size="0.012" pos="0.057 0.138 0" rgba="0.1 0.8 0.3 1"/>
      <geom name="rc4"  type="sphere" size="0.012" pos="0.000 0.150 0" rgba="0.1 0.8 0.3 1"/>
      <geom name="rc5"  type="sphere" size="0.012" pos="-0.057 0.138 0" rgba="0.1 0.8 0.3 1"/>
      <geom name="rc6"  type="sphere" size="0.012" pos="-0.106 0.106 0" rgba="0.1 0.8 0.3 1"/>
      <geom name="rc7"  type="sphere" size="0.012" pos="-0.138 0.057 0" rgba="0.1 0.8 0.3 1"/>
      <geom name="rc8"  type="sphere" size="0.012" pos="-0.150 0.000 0" rgba="0.1 0.8 0.3 1"/>
      <geom name="rc9"  type="sphere" size="0.012" pos="-0.138 -0.057 0" rgba="0.1 0.8 0.3 1"/>
      <geom name="rc10" type="sphere" size="0.012" pos="-0.106 -0.106 0" rgba="0.1 0.8 0.3 1"/>
      <geom name="rc11" type="sphere" size="0.012" pos="-0.057 -0.138 0" rgba="0.1 0.8 0.3 1"/>
      <geom name="rc12" type="sphere" size="0.012" pos="0.000 -0.150 0" rgba="0.1 0.8 0.3 1"/>
      <geom name="rc13" type="sphere" size="0.012" pos="0.057 -0.138 0" rgba="0.1 0.8 0.3 1"/>
      <geom name="rc14" type="sphere" size="0.012" pos="0.106 -0.106 0" rgba="0.1 0.8 0.3 1"/>
      <geom name="rc15" type="sphere" size="0.012" pos="0.138 -0.057 0" rgba="0.1 0.8 0.3 1"/>
    </body>
    <!-- 底座 + 三自由度机械臂 -->
    <body name="base" pos="0 0 0">
      <geom name="basegeom" type="cylinder" size="0.07 0.10" pos="0 0 0.10" rgba="0.32 0.32 0.38 1"/>
      <geom name="pillar"  type="cylinder" size="0.035 0.10" pos="0 0 0.22" rgba="0.45 0.45 0.5 1"/>
      <joint name="yaw" type="hinge" axis="0 0 1" pos="0 0 0.24" range="-720 720"/>
      <body name="l1" pos="0 0 0.24">
        <geom name="link1" type="capsule" fromto="0 0 0 0.30 0 0" size="0.035" rgba="0.2 0.42 0.92 1"/>
        <joint name="shoulder" type="hinge" axis="0 -1 0" range="-180 180"/>
        <body name="l2" pos="0.30 0 0">
          <geom name="link2" type="capsule" fromto="0 0 0 0.30 0 0" size="0.028" rgba="0.95 0.55 0.12 1"/>
          <joint name="elbow" type="hinge" axis="0 -1 0" range="-180 180"/>
          <body name="tip" pos="0.30 0 0">
            <geom name="tipball" type="sphere" size="0.028" rgba="0.9 0.15 0.15 1"/>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="m_yaw" joint="yaw" ctrlrange="-30 30" ctrllimited="true"/>
    <motor name="m_shoulder" joint="shoulder" ctrlrange="-30 30" ctrllimited="true"/>
    <motor name="m_elbow" joint="elbow" ctrlrange="-30 30" ctrllimited="true"/>
  </actuator>
</mujoco>
"""

# ============ 参数 ============
L1 = 0.30                # 第一段杆长（m）
L2 = 0.30                # 第二段杆长（m）
BASE_Z = 0.24            # 肩俯仰铰点高度（m，与 XML 一致）

# 螺旋线轨迹（世界坐标）
RADIUS = 0.15            # 螺旋半径（m）
Z_LO = 0.20              # 螺旋起点高度（m）
Z_HI = 0.40              # 螺旋终点高度（m）
TURNS = 1.5              # 圈数
PERIOD = 6.0             # 每圈秒数
DURATION = 9.0           # 总时长（s）

# 渲染与控制
FPS = 24
KP = 40.0
KD = 12.0
WIDTH, HEIGHT = 640, 480


def ik3(x, y, z):
    """三自由度空间逆运动学（yaw + shoulder + elbow），返回 (q0, q1, q2) 或 None。

    几何：先由水平投影求方位角 q0；再在臂平面（r-z）内用 2 连杆解析解求 q1、q2。
    模型约定：shoulder/elbow 绕 -Y 轴，正角度使连杆向 +Z 方向（向上）弯折。
    """
    r = math.hypot(x, y)
    dz = z - BASE_Z
    d = math.hypot(r, dz)
    if d > L1 + L2 - 1e-6 or d < abs(L1 - L2) + 1e-6:
        return None
    cos_q2 = (d * d - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
    q2 = math.acos(max(-1.0, min(1.0, cos_q2)))          # 肘上姿势
    q1 = math.atan2(dz, r) - math.atan2(L2 * math.sin(q2),
                                        L1 + L2 * math.cos(q2))
    q0 = math.atan2(y, x)
    return q0, q1, q2


def fk3(q0, q1, q2):
    """三自由度正运动学：关节角 -> 末端世界坐标（与 ik3 互查用）。"""
    x_arm = L1 * math.cos(q1) + L2 * math.cos(q1 + q2)
    z_arm = L1 * math.sin(q1) + L2 * math.sin(q1 + q2)
    x = x_arm * math.cos(q0)
    y = x_arm * math.sin(q0)
    z = BASE_Z + z_arm
    return x, y, z


def main():
    model = mujoco.MjModel.from_xml_string(MODEL_XML)
    data = mujoco.MjData(model)

    # 关节索引（按名字查）
    def jq(name):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name.encode())
        if jid < 0:
            raise RuntimeError('joint %s not found' % name)
        return model.jnt_qposadr[jid], model.jnt_dofadr[jid]

    qa_y, va_y = jq('yaw')
    qa_s, va_s = jq('shoulder')
    qa_e, va_e = jq('elbow')
    tip_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, b'tip')

    # 渲染器 + 斜视角自由相机
    renderer = mujoco.Renderer(model, HEIGHT, WIDTH)
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat = np.array([0.0, 0.0, 0.30])
    cam.distance = 1.30
    cam.azimuth = 45.0
    cam.elevation = 28.0

    frames = []
    traj = []                      # 末端轨迹点（像素）
    q_prev = None                  # 上一帧目标关节角（用于 yaw 解缠）
    total_frames = int(DURATION * FPS)

    print("三自由度机械臂空间螺旋线：半径 %.2f m，高度 %.2f→%.2f m，%g 圈，共 %d 帧" %
          (RADIUS, Z_LO, Z_HI, TURNS, total_frames))
    import time
    t0 = time.time()

    for i in range(total_frames):
        t = i / FPS
        ang = 2.0 * math.pi * (t % PERIOD) / PERIOD
        # 螺旋：水平圆 + 垂直匀速上升（并伴随半径轻微变化，轨迹更立体）
        frac = t / DURATION
        tx = RADIUS * math.cos(ang)
        ty = RADIUS * math.sin(ang)
        tz = Z_LO + (Z_HI - Z_LO) * frac

        q = ik3(tx, ty, tz)
        if q is None:
            print("  [warn] 目标点在工作空间外：x=%.2f y=%.2f z=%.2f" % (tx, ty, tz))
            continue
        # yaw 解缠：螺旋 1.5 圈时 atan2 会在 ±pi 处跳变，需保持关节角连续
        q0 = q[0]
        if q_prev is not None:
            while q0 - q_prev[0] > math.pi:
                q0 -= 2.0 * math.pi
            while q0 - q_prev[0] < -math.pi:
                q0 += 2.0 * math.pi
        q = (q0, q[1], q[2])
        q_prev = q

        # PD 位置控制（三关节）
        data.ctrl[0] = KP * (q[0] - data.qpos[qa_y]) - KD * data.qvel[va_y]
        data.ctrl[1] = KP * (q[1] - data.qpos[qa_s]) - KD * data.qvel[va_s]
        data.ctrl[2] = KP * (q[2] - data.qpos[qa_e]) - KD * data.qvel[va_e]
        for _ in range(3):
            mujoco.mj_step(model, data)

        # 渲染当前帧
        renderer.update_scene(data, camera=cam)
        rgb = renderer.render().copy()

        # 末端世界坐标 -> 像素（按 MjvCamera 参数手算 lookAt + 透视投影）
        az = math.radians(cam.azimuth)
        el = math.radians(cam.elevation)
        eye = cam.lookat + cam.distance * np.array([
            math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])
        f = cam.lookat - eye
        f = f / np.linalg.norm(f)
        up = np.array([0.0, 0.0, 1.0])
        s = np.cross(f, up)
        s = s / np.linalg.norm(s)
        u = np.cross(s, f)
        view = np.eye(4)
        view[0, :3] = s
        view[1, :3] = u
        view[2, :3] = -f
        view[0, 3] = -np.dot(s, eye)
        view[1, 3] = -np.dot(u, eye)
        view[2, 3] = np.dot(f, eye)
        near, far = 0.01, 10.0
        half_h = math.tan(math.radians(45.0) / 2.0) * near   # MuJoCo 默认 fovy=45
        half_w = half_h * (WIDTH / HEIGHT)
        proj = np.zeros((4, 4))
        proj[0, 0] = near / half_w
        proj[1, 1] = near / half_h
        proj[2, 2] = -(far + near) / (far - near)
        proj[2, 3] = -2.0 * far * near / (far - near)
        proj[3, 2] = -1.0
        p4 = np.array([tx, ty, tz, 1.0])
        clip = proj @ view @ p4
        ndc = clip[:3] / clip[3]
        px = int((ndc[0] * 0.5 + 0.5) * WIDTH)
        py = int((1.0 - (ndc[1] * 0.5 + 0.5)) * HEIGHT)
        traj.append((px, py))

        # 叠加黄色轨迹线
        im = Image.fromarray(rgb)
        draw = ImageDraw.Draw(im)
        if len(traj) >= 2:
            for k in range(1, len(traj)):
                draw.line([traj[k - 1], traj[k]], fill=(255, 200, 0), width=4)
        frames.append(im)

        if (i + 1) % (total_frames // 4) == 0:
            print("  已渲染 %d/%d 帧（%.0f%%）" % (i + 1, total_frames,
                                                  (i + 1) * 100.0 / total_frames))

    print("渲染完成，耗时 %.1f 秒，合成 GIF..." % (time.time() - t0))
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "demo_arm3d.gif")
    if frames:
        frames[0].save(out, save_all=True, append_images=frames[1:],
                       duration=int(1000 / FPS), loop=0)
        print("已生成效果图：%s（%d 帧，%.1f 秒）" % (out, len(frames),
                                                  len(frames) / FPS))
    else:
        print("未生成任何帧！请检查模型与渲染后端。")
        import sys
        sys.exit(1)


if __name__ == "__main__":
    main()
