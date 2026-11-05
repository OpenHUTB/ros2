# -*- coding: utf-8 -*-
"""MuJoCo 三自由度机械臂拧螺丝仿真（单文件，XML 内嵌）

机械臂末端夹持竖直螺丝刀，对准桌面上的螺丝，连续旋转 2.5 圈（900°）
把螺丝拧入桌面螺孔；螺丝随转角按螺距匀速下沉，螺丝刀同步下压跟随。
渲染叠加螺丝刀刀头的螺旋轨迹（青色），输出 demo_armscrew.gif 效果图。

运行（虚拟机 / 无显卡环境）：
    sudo apt-get update && sudo apt-get install -y libosmesa6
    pip install mujoco pillow numpy
    MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa python3 mujoco_arm_screw_single.py

输出：同目录 demo_armscrew.gif
"""
import os, math

os.environ.setdefault("MUJOCO_GL", "glfw")
import numpy as np
import mujoco
from PIL import Image, ImageDraw

# ============ 模型（XML 内嵌） ============
MODEL_XML = r"""
<mujoco model="armscrew">
  <compiler angle="radian"/>
  <option timestep="0.002" gravity="0 0 -9.81"/>
  <visual>
    <headlight ambient="0.62 0.62 0.62" diffuse="0.4 0.4 0.4"/>
  </visual>
  <worldbody>
    <light name="l1" diffuse="0.78 0.78 0.78" pos="0.7 0.7 1.0" dir="-0.45 -0.45 -1" castshadow="false"/>
    <light name="l2" diffuse="0.35 0.35 0.42" pos="-0.7 -0.6 0.9" dir="0.55 0.4 -1" castshadow="false"/>
    <geom name="floor" type="plane" size="1.0 1.0 0.02" rgba="0.9 0.9 0.92 1"/>

    <!-- 桌面木板 -->
    <body name="table" pos="0.18 0.05 0.0">
      <geom name="board" type="box" size="0.13 0.11 0.01" pos="0 0 0.01" rgba="0.72 0.55 0.38 1"/>
      <!-- 螺孔（深色圆盘标记） -->
      <geom name="hole" type="cylinder" size="0.021 0.002" pos="0 0 0.0205" rgba="0.10 0.10 0.12 1"/>
    </body>

    <!-- 三自由度空间机械臂（肩方位 + 肩俯仰 + 肘俯仰） -->
    <body name="base" pos="0 0 0">
      <geom name="basegeom" type="cylinder" size="0.052 0.016" pos="0 0 0.016" rgba="0.3 0.32 0.38 1"/>
      <joint name="yaw" type="hinge" axis="0 0 1" pos="0 0 0.06" range="-180 180"/>
      <body name="shoulder" pos="0 0 0.06">
        <geom name="shoulderjoint" type="sphere" size="0.022" pos="0 0 0" rgba="0.45 0.45 0.5 1"/>
        <joint name="pitch1" type="hinge" axis="0 -1 0" pos="0 0 0" range="-120 120"/>
        <body name="l1" pos="0 0 0">
          <geom name="link1" type="capsule" fromto="0 0 0 0.30 0 0" size="0.034" rgba="0.2 0.42 0.92 1"/>
          <joint name="pitch2" type="hinge" axis="0 -1 0" pos="0.30 0 0" range="-150 150"/>
          <body name="l2" pos="0.30 0 0">
            <geom name="link2" type="capsule" fromto="0 0 0 0.30 0 0" size="0.027" rgba="0.95 0.55 0.12 1"/>
            <body name="tip" pos="0.30 0 0">
              <geom name="tipball" type="sphere" size="0.024" rgba="0.9 0.15 0.15 1"/>
              <!-- 夹持爪（末端小夹头，视觉） -->
              <geom name="grip" type="cylinder" size="0.014 0.012" pos="0.01 0 0.0"
                    euler="0 1.5707963 0" rgba="0.3 0.3 0.34 1"/>
            </body>
          </body>
        </body>
      </body>
    </body>

    <!-- 螺丝刀（竖直悬挂于末端下方，位置由脚本控制） -->
    <body name="driver" pos="0.18 0.05 0.1">
      <freejoint name="driverfree"/>
      <!-- 刀杆（竖直） -->
      <geom name="shaft" type="cylinder" size="0.009 0.05" pos="0 0 0.05" rgba="0.7 0.72 0.78 1"/>
      <!-- 刀头横杆 -->
      <geom name="bladeA" type="box" size="0.016 0.003 0.003" pos="0 0 -0.003" rgba="0.88 0.9 0.95 1"/>
      <geom name="bladeB" type="box" size="0.003 0.016 0.003" pos="0 0 -0.003" rgba="0.88 0.9 0.95 1"/>
    </body>

    <!-- 螺丝（自由体，位置/转角/下沉由脚本控制） -->
    <body name="screw" pos="0.18 0.05 0.02">
      <freejoint name="screwfree"/>
      <geom name="screwbody" type="cylinder" size="0.019 0.023" pos="0 0 0" rgba="0.78 0.78 0.84 1"/>
      <geom name="screwtipg" type="cylinder" size="0.012 0.007" pos="0 0 -0.027" rgba="0.72 0.72 0.78 1"/>
      <!-- 螺丝顶十字槽（随旋转可见） -->
      <geom name="slotA" type="box" size="0.015 0.0026 0.0022" pos="0 0 0.0235" rgba="0.16 0.16 0.18 1"/>
      <geom name="slotB" type="box" size="0.0026 0.015 0.0022" pos="0 0 0.0235" rgba="0.16 0.16 0.18 1"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="m_yaw"    joint="yaw"    ctrlrange="-30 30" ctrllimited="true"/>
    <motor name="m_pitch1" joint="pitch1" ctrlrange="-30 30" ctrllimited="true"/>
    <motor name="m_pitch2" joint="pitch2" ctrlrange="-30 30" ctrllimited="true"/>
  </actuator>
</mujoco>
"""

# ============ 参数 ============
L1 = 0.30
L2 = 0.30
BASE_Z = 0.06            # 肩俯仰中心高度

SCREW_XY = (0.18, 0.05)  # 螺丝/螺孔平面位置
SCREW_TOP_Z = 0.02        # 螺丝中心初始高度
SCREW_HALF = 0.023        # 螺丝圆柱半高（顶面 = 中心 + 0.023）

SHAFT_LEN = 0.10         # 螺丝刀刀头到末端 tip 的竖直距离
DRIVER_OFF = 0.11        # 刀头在 tip 下方 0.11 m

START_Z = -0.040         # 拧入量（螺丝中心下沉 40 mm，顶面拧到与板面齐平）
TURNS = 2.5              # 旋转圈数
THETA_MAX = 2 * math.pi * TURNS

HOME = (0.14, 0.0, 0.20)           # 待机点

# 时间线（帧，每帧 4 ms，450 帧 / 15 s）
APPROACH_END = 70     # 移向螺丝上方
PRESS_END = 110       # 下压对位
SCREW_END = 350       # 旋转拧入完成（240 帧 = 0.96 s，900°）
LIFT_END = 420        # 抬起
TOTAL = 450

FPS = 30
DT_SIM = 0.004
QSTEP = 0.07          # 运动学插值每帧最大关节角步长（rad）
WIDTH, HEIGHT = 640, 480
FOVY = 45.0
F = (HEIGHT / 2.0) / math.tan(math.radians(FOVY) / 2.0)

CAM_LOOKAT = (0.11, 0.04, 0.14)
CAM_DIST = 0.98
CAM_AZ = 60.0
CAM_EL = -30.0


def ik3(x, y, z, yaw_prev=None):
    """三自由度空间 IK：肩方位 yaw + 臂平面内二连杆。

    目标靠近桌面时取负解（q2 < 0）：肘抬到肩上方，L2 从上往下伸到目标，
    形成"臂从上方伸下来拧螺丝"的构型（正解会让肘沉到地面以下）。
    """
    r = math.hypot(x, y)
    z = z - BASE_Z
    d = math.hypot(r, z)
    if d > L1 + L2 - 1e-6 or d < abs(L1 - L2) + 1e-6 or r < 1e-9:
        return None
    yaw = math.atan2(y, x)
    if yaw_prev is not None:
        while yaw - yaw_prev > math.pi:
            yaw -= 2 * math.pi
        while yaw - yaw_prev < -math.pi:
            yaw += 2 * math.pi
    cos_q2 = (d * d - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
    q2 = -math.acos(max(-1.0, min(1.0, cos_q2)))           # 肘上（L2 向下伸）
    q1 = math.atan2(z, r) - math.atan2(L2 * math.sin(q2),
                                       L1 + L2 * math.cos(q2))
    return yaw, q1, q2


def screw_state(frame):
    """返回 (臂末端目标 tip, 螺丝旋转角 theta, 螺丝中心 z)。"""
    hi_z = 0.22          # 螺丝上方待机高度
    down_z = SCREW_TOP_Z + SCREW_HALF + DRIVER_OFF   # 刀头贴螺丝顶时的 tip 高度
    if frame < APPROACH_END:
        t = frame / APPROACH_END
        tz = HOME[2] + (hi_z - HOME[2]) * t
        return (SCREW_XY[0], SCREW_XY[1], tz), 0.0, SCREW_TOP_Z
    if frame < PRESS_END:
        t = (frame - APPROACH_END) / (PRESS_END - APPROACH_END)
        tz = hi_z + (down_z - hi_z) * t
        return (SCREW_XY[0], SCREW_XY[1], tz), 0.0, SCREW_TOP_Z
    if frame < SCREW_END:
        t = (frame - PRESS_END) / (SCREW_END - PRESS_END)
        theta = THETA_MAX * t
        # 螺距：按转角匀速下沉
        sz = SCREW_TOP_Z - abs(START_Z) * t
        # tip 跟随螺丝顶（刀头始终贴住）
        return (SCREW_XY[0], SCREW_XY[1], sz + SCREW_HALF + DRIVER_OFF), theta, sz
    if frame < LIFT_END:
        sz = SCREW_TOP_Z + START_Z
        tt = (frame - SCREW_END) / (LIFT_END - SCREW_END)
        base = sz + SCREW_HALF + DRIVER_OFF
        tz = base + (hi_z - base) * tt
        return (SCREW_XY[0], SCREW_XY[1], tz), THETA_MAX, sz
    return (SCREW_XY[0], SCREW_XY[1], hi_z), THETA_MAX, SCREW_TOP_Z + START_Z


def quat_z(theta):
    """绕 z 轴旋转 theta 的四元数 (w, x, y, z)。"""
    return (math.cos(theta / 2.0), 0.0, 0.0, math.sin(theta / 2.0))


def project(x, y, z):
    """手算 lookAt 透视投影（与渲染相机一致）。"""
    az = math.radians(CAM_AZ)
    el = math.radians(CAM_EL)
    cx, cy, cz = CAM_LOOKAT
    ex = cx + CAM_DIST * math.cos(el) * math.cos(az)
    ey = cy + CAM_DIST * math.cos(el) * math.sin(az)
    ez = cz + CAM_DIST * math.sin(el)

    fwd = np.array([cx - ex, cy - ey, cz - ez])
    fwd = fwd / np.linalg.norm(fwd)
    up = np.array([0.0, 0.0, 1.0])
    right = np.cross(fwd, up)
    right = right / np.linalg.norm(right)
    upv = np.cross(right, fwd)

    rel = np.array([x - ex, y - ey, z - ez])
    pz = float(np.dot(rel, fwd))
    if pz <= 1e-4:
        return None
    px = float(np.dot(rel, right))
    py = float(np.dot(rel, upv))
    return (WIDTH / 2 + px / pz * F, HEIGHT / 2 - py / pz * F)


def main():
    model = mujoco.MjModel.from_xml_string(MODEL_XML)
    data = mujoco.MjData(model)

    def jq(name):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name.encode())
        if jid < 0:
            raise RuntimeError('joint %s not found' % name)
        return model.jnt_qposadr[jid], model.jnt_dofadr[jid]

    (qa_y, va_y), (qa_p1, va_p1), (qa_p2, va_p2) = jq('yaw'), jq('pitch1'), jq('pitch2')
    jf_d = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, b'driverfree')
    d_qadr = model.jnt_qposadr[jf_d]
    jf_s = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, b'screwfree')
    s_qadr = model.jnt_qposadr[jf_s]

    # 渲染器 + 相机
    renderer = mujoco.Renderer(model, HEIGHT, WIDTH)
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat = np.array(CAM_LOOKAT)
    cam.distance = CAM_DIST
    cam.azimuth = CAM_AZ
    cam.elevation = CAM_EL

    tip = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, b'tip')

    # 初始姿态：HOME
    q0 = ik3(*HOME, None)
    if q0 is not None:
        data.qpos[qa_y], data.qpos[qa_p1], data.qpos[qa_p2] = q0
    yaw_prev = q0[0] if q0 is not None else None
    mujoco.mj_forward(model, data)

    print("拧螺丝：螺丝 %s，旋转 %.1f 圈（%.0f°），拧入 %.0f mm"
          % (SCREW_XY, TURNS, math.degrees(THETA_MAX), abs(START_Z) * 1000))

    import time
    try:
        from PIL import ImageFont
        font = ImageFont.load_default()
        for fp in (r"C:\Windows\Fonts\arial.ttf",
                   "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"):
            try:
                font = ImageFont.truetype(fp, 15)
                break
            except Exception:
                pass
    except Exception:
        font = None

    t0 = time.time()
    frames = []
    tip_traj = []
    blade_traj = []
    min_gap = 1e9
    min_gap_frame = -1
    theta_final = 0.0

    for i in range(TOTAL):
        tgt, theta, sz = screw_state(i)

        # ---- 臂：运动学插值 + mj_forward ----
        q = ik3(tgt[0], tgt[1], tgt[2], yaw_prev)
        if q is not None:
            yaw_prev = q[0]
            for k, qa in enumerate((qa_y, qa_p1, qa_p2)):
                dq = q[k] - data.qpos[qa]
                dq = max(-QSTEP, min(QSTEP, dq))
                data.qpos[qa] += dq
        mujoco.mj_forward(model, data)

        # ---- 螺丝刀：竖直悬挂于 tip 下方（刀头位于 tip 下 DRIVER_OFF） ----
        ptip = data.xpos[tip]
        driver_pos = np.array([ptip[0], ptip[1], ptip[2] - DRIVER_OFF])
        data.qpos[d_qadr:d_qadr+3] = driver_pos
        data.qpos[d_qadr+3:d_qadr+7] = quat_z(theta)

        # ---- 螺丝：旋转 + 下沉 ----
        data.qpos[s_qadr:s_qadr+3] = (SCREW_XY[0], SCREW_XY[1], sz)
        data.qpos[s_qadr+3:s_qadr+7] = quat_z(theta)
        mujoco.mj_forward(model, data)
        theta_final = theta

        # 刀头与螺丝顶间隙（刀头即 driver body 原点）
        blade_z = driver_pos[2]
        screw_top = sz + SCREW_HALF
        gap = abs(blade_z - screw_top)
        if i >= PRESS_END:
            if gap < min_gap:
                min_gap, min_gap_frame = gap, i

        # ---- 渲染 ----
        renderer.update_scene(data, camera=cam)
        rgb = renderer.render().copy()

        pa = project(ptip[0], ptip[1], ptip[2])
        pb = project(driver_pos[0], driver_pos[1], driver_pos[2])

        im = Image.fromarray(rgb)
        draw = ImageDraw.Draw(im)
        if pa is not None:
            tip_traj.append(pa)
        if pb is not None and i >= PRESS_END and i < LIFT_END:
            blade_traj.append(pb)
        if len(tip_traj) >= 2:
            draw.line(tip_traj, fill=(255, 200, 0), width=2, joint='curve')
        if len(blade_traj) >= 2:
            draw.line(blade_traj, fill=(0, 220, 230), width=2, joint='curve')
        if pb is not None:
            draw.ellipse([pb[0]-3, pb[1]-3, pb[0]+3, pb[1]+3], fill=(0, 230, 240))

        # 顶部状态条（圈数 / 拧入深度）
        draw.rectangle([0, 0, WIDTH, 26], fill=(16, 16, 18))
        turns_now = theta / (2 * math.pi)
        depth_mm = (SCREW_TOP_Z - sz) * 1000
        draw.text((10, 6),
                  "SCREW DRIVING   turns %.2f / %.1f   depth %d / %d mm"
                  % (turns_now, TURNS, int(depth_mm), int(abs(START_Z) * 1000)),
                  fill=(235, 235, 240), font=font)
        frames.append(im.quantize(colors=96))   # 减色压缩 GIF

        if (i + 1) % (TOTAL // 4) == 0:
            print("  已渲染 %d/%d 帧（%.0f%%）" % (i + 1, TOTAL, (i + 1) * 100.0 / TOTAL))

    print("刀头-螺丝顶最小间隙：%.4f m（frame %d）" % (min_gap, min_gap_frame))
    print("最终转角：%.1f°（%.2f 圈）" %
          (math.degrees(theta_final), theta_final / (2 * math.pi)))
    print("螺丝下沉：%.1f mm" % (abs(START_Z) * 1000))
    print("渲染完成，耗时 %.1f 秒，合成 GIF..." % (time.time() - t0))

    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "demo_armscrew.gif")
    if frames:
        frames[0].save(out, save_all=True, append_images=frames[1:],
                       duration=int(1000 / FPS), loop=0)
        print("已生成效果图：%s（%d 帧，%.1f 秒）" % (out, len(frames), len(frames) / FPS))
    else:
        import sys
        print("未生成任何帧！请检查模型与渲染后端。")
        sys.exit(1)


if __name__ == "__main__":
    main()
