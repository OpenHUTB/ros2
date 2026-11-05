# -*- coding: utf-8 -*-
"""MuJoCo 三自由度机械臂投篮入筐仿真（单文件，XML 内嵌）

三自由度空间机械臂（肩方位 + 肩俯仰 + 肘俯仰）末端托球，每球先举到
预备点蓄力，释放前 12 帧快速上摆到出手点，把球按 65° 仰角向上抛出，
球沿抛物线飞向篮筐并落进筐中。
球用解析抛物线驱动（与渲染同一时间线，慢放 8 倍看清轨迹），
入筐判定按球心穿过筐平面时的水平偏差计算。

渲染叠加：橙色球抛物线、顶部状态条（投篮序号与命中数），
输出 demo_armball.gif 效果图。

运行（虚拟机 / 无显卡环境）：
    sudo apt-get update && sudo apt-get install -y libosmesa6
    pip install mujoco pillow numpy
    MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa python3 mujoco_arm_ball_single.py

运行（Windows 本机）：
    python mujoco_arm_ball_single.py

输出：同目录 demo_armball.gif
"""
import os, math

os.environ.setdefault("MUJOCO_GL", "glfw")
import numpy as np
import mujoco
from PIL import Image, ImageDraw

# ============ 参数 ============
L1 = 0.30                 # 大臂
L2 = 0.30                 # 小臂
BASE_Z = 0.06             # 肩俯仰中心高度

HOME = (0.15, 0.0, 0.20)  # 待机点（末端 tip 目标）

# 出手点（球心位置），两次投篮
SHOT_BALL = [(0.30, 0.0, 0.36), (0.29, 0.0, 0.34)]
# 预备点（球心位置）：举球在这里蓄力，释放前快速上摆到出手点
PRE_BALL = [(0.27, 0.0, 0.21), (0.26, 0.0, 0.19)]
GRIP_OFF = 0.065          # 球心在末端 tip 上方 0.065 m（手托球）

HOOP = (0.60, 0.0, 0.42)  # 筐心（位于臂前方 0.3 m，抛物线更长更好看）
R_HOOP = 0.11             # 筐半径
R_BALL = 0.05             # 球半径
G = 9.81
ALPHA_DEG = 65.0          # 出手仰角
ALPHA = math.radians(ALPHA_DEG)

# 时间线（帧，30 fps，共 450 帧 / 15 s）
SHOT_REL = [100, 340]     # 释放帧
TOTAL = 450
FPS = 30
DT_SIM = 0.004            # 每帧推进的物理时间（慢放 8 倍）
QSTEP = 0.09              # 每帧关节角最大步长（rad）

WIDTH, HEIGHT = 640, 480
FOVY = 45.0
F = (HEIGHT / 2.0) / math.tan(math.radians(FOVY) / 2.0)

CAM_LOOKAT = (0.32, 0.03, 0.25)
CAM_DIST = 1.44
CAM_AZ = 52.0
CAM_EL = -14.0


# ============ 模型 ============
def hoop_geoms(R=0.11, segs=12, net=8):
    """生成筐环（胶囊拼圆）+ 网线（细胶囊）。"""
    parts = []
    for i in range(segs):
        a1 = 2 * math.pi * i / segs
        a2 = 2 * math.pi * (i + 1) / segs
        parts.append(
            '<geom type="capsule" fromto="%.4f %.4f 0 %.4f %.4f 0" size="0.008" '
            'rgba="0.88 0.30 0.12 1"/>'
            % (R * math.cos(a1), R * math.sin(a1), R * math.cos(a2), R * math.sin(a2)))
    for i in range(net):
        a = 2 * math.pi * i / net
        parts.append(
            '<geom type="capsule" fromto="%.4f %.4f 0 %.4f %.4f -0.13" size="0.0025" '
            'rgba="0.95 0.95 0.96 1" contype="0" conaffinity="0"/>'
            % (R * math.cos(a), R * math.sin(a), 0.58 * R * math.cos(a), 0.58 * R * math.sin(a)))
    return "\n      ".join(parts)


MODEL_XML = r"""
<mujoco model="armball">
  <compiler angle="radian"/>
  <option timestep="0.002" gravity="0 0 -9.81"/>
  <visual>
    <headlight ambient="0.60 0.60 0.60" diffuse="0.42 0.42 0.42"/>
  </visual>
  <worldbody>
    <light name="l1" directional="true" diffuse="0.62 0.62 0.66" specular="0.10 0.10 0.10" dir="-0.45 -0.35 -1"/>
    <light name="l2" diffuse="0.34 0.34 0.42" pos="-0.6 -0.5 0.9" dir="0.5 0.35 -1" castshadow="false"/>
    <geom name="floor" type="plane" size="1.8 1.8 0.02" rgba="0.90 0.90 0.92 1"/>

    <!-- 三自由度空间机械臂 -->
    <body name="base" pos="0 0 0">
      <geom name="basegeom" type="cylinder" size="0.058 0.030" pos="0 0 0.030" rgba="0.30 0.32 0.38 1"/>
      <joint name="yaw" type="hinge" axis="0 0 1" pos="0 0 0.06" range="-180 180"/>
      <body name="shoulder" pos="0 0 0.06">
        <geom name="shoulderjoint" type="sphere" size="0.023" rgba="0.45 0.45 0.50 1"/>
        <joint name="pitch1" type="hinge" axis="0 -1 0" range="-120 120"/>
        <body name="l1" pos="0 0 0">
          <geom name="link1" type="capsule" fromto="0 0 0 0.30 0 0" size="0.033" rgba="0.20 0.42 0.92 1"/>
          <body name="l2" pos="0.30 0 0">
            <joint name="pitch2" type="hinge" axis="0 -1 0" range="-150 150"/>
            <geom name="link2" type="capsule" fromto="0 0 0 0.30 0 0" size="0.026" rgba="0.95 0.55 0.12 1"/>
            <body name="tip" pos="0.30 0 0">
              <geom name="tipball" type="sphere" size="0.023" rgba="0.90 0.15 0.15 1"/>
              <!-- 托球的手掌（开口向上的小圆盘） -->
              <geom name="palm" type="cylinder" size="0.045 0.010" pos="0.0 0 0.030" rgba="0.26 0.26 0.30 1"/>
            </body>
          </body>
        </body>
      </body>
    </body>

    <!-- 篮筐（筐环 + 网 + 立柱） -->
    <body name="hoop" pos="%.3f %.3f %.3f">
      """ % HOOP + hoop_geoms() + r"""
      <geom type="cylinder" size="0.016 0.21" pos="0 0.11 -0.21" rgba="0.55 0.57 0.62 1"/>
    </body>

    <!-- 两颗球（自由体，位置由脚本驱动） -->
    <body name="ball1" pos="0.30 0 0.36">
      <freejoint name="ball1free"/>
      <geom name="b1" type="sphere" size="0.05" rgba="0.98 0.52 0.10 1"/>
    </body>
    <body name="ball2" pos="0.29 0 0.34">
      <freejoint name="ball2free"/>
      <geom name="b2" type="sphere" size="0.05" rgba="0.98 0.52 0.10 1"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="m_yaw"    joint="yaw"    ctrlrange="-30 30" ctrllimited="true"/>
    <motor name="m_pitch1" joint="pitch1" ctrlrange="-30 30" ctrllimited="true"/>
    <motor name="m_pitch2" joint="pitch2" ctrlrange="-30 30" ctrllimited="true"/>
  </actuator>
</mujoco>
"""


# ============ 运动学 ============
def ik3(x, y, z, yaw_prev=None):
    """三自由度空间 IK：肩方位 + 臂平面内二连杆（取肘上负解）。"""
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
    q2 = -math.acos(max(-1.0, min(1.0, cos_q2)))
    q1 = math.atan2(z, r) - math.atan2(L2 * math.sin(q2), L1 + L2 * math.cos(q2))
    return yaw, q1, q2


def quat_q(theta):
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
    right = np.cross(up, fwd)          # 屏幕右方向（与世界 +x 同向时为正）
    right = right / np.linalg.norm(right)
    upv = np.cross(fwd, right)         # 屏幕上方向

    rel = np.array([x - ex, y - ey, z - ez])
    pz = float(np.dot(rel, fwd))
    if pz <= 1e-4:
        return None
    px = float(np.dot(rel, right))
    py = float(np.dot(rel, upv))
    return (WIDTH / 2 + px / pz * F, HEIGHT / 2 - py / pz * F)


# ============ 抛射参数 ============
def solve_v(p0, hoop, alpha):
    """给定出手点、筐心与仰角，解出所需出手速度。"""
    d = math.hypot(hoop[0] - p0[0], hoop[1] - p0[1])
    h = hoop[2] - p0[2]
    denom = 2.0 * math.cos(alpha) ** 2 * (d * math.tan(alpha) - h)
    if denom <= 1e-6:
        return None
    return math.sqrt(G * d * d / denom)


def ball_pos(p0, v, alpha, t):
    vx = v * math.cos(alpha)
    vz = v * math.sin(alpha)
    dx = HOOP[0] - p0[0]
    dy = HOOP[1] - p0[1]
    horiz = math.hypot(dx, dy)
    ux, uy = (dx / horiz, dy / horiz) if horiz > 1e-9 else (1.0, 0.0)
    return (p0[0] + ux * vx * t,
            p0[1] + uy * vx * t,
            p0[2] + vz * t - 0.5 * G * t * t)


def arm_target(frame):
    """分段的末端 tip 目标（球心位置减去 GRIP_OFF）。

    每球都先举到"预备点"蓄力，再在释放前 12 帧快速上摆到出手点，
    让手把球向上"抛"出去，而不是静止松手。
    """
    p1 = (SHOT_BALL[0][0], SHOT_BALL[0][1], SHOT_BALL[0][2] - GRIP_OFF)
    p2 = (SHOT_BALL[1][0], SHOT_BALL[1][1], SHOT_BALL[1][2] - GRIP_OFF)
    c1 = (PRE_BALL[0][0], PRE_BALL[0][1], PRE_BALL[0][2] - GRIP_OFF)
    c2 = (PRE_BALL[1][0], PRE_BALL[1][1], PRE_BALL[1][2] - GRIP_OFF)

    def lerp(a, b, t):
        t = max(0.0, min(1.0, t))
        return (a[0] + (b[0] - a[0]) * t,
                a[1] + (b[1] - a[1]) * t,
                a[2] + (b[2] - a[2]) * t)

    # 第 1 球：待机 -> 预备点 -> 上摆出手 -> 停住 -> 回位
    if frame < 45:
        return lerp(HOME, c1, frame / 45.0)
    if frame < 88:
        return c1                      # 蓄力：托球停在预备点
    if frame < 100:
        return lerp(c1, p1, (frame - 88) / 12.0)   # 快速上摆（12 帧）
    if frame < 170:
        return p1
    if frame < 240:
        return lerp(p1, HOME, (frame - 170) / 70.0)
    # 第 2 球：待机 -> 预备点 -> 上摆出手 -> 停住 -> 回位
    if frame < 288:
        return lerp(HOME, c2, (frame - 240) / 48.0)
    if frame < 328:
        return c2
    if frame < 340:
        return lerp(c2, p2, (frame - 328) / 12.0)
    if frame < 410:
        return p2
    return lerp(p2, HOME, (frame - 410) / 40.0)


def main():
    model = mujoco.MjModel.from_xml_string(MODEL_XML)
    data = mujoco.MjData(model)

    def qadr(name):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name.encode())
        if jid < 0:
            raise RuntimeError('joint not found: %s' % name)
        return model.jnt_qposadr[jid]

    qa = (qadr('yaw'), qadr('pitch1'), qadr('pitch2'))
    b_adr = [qadr('ball1free'), qadr('ball2free')]
    tip = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, b'tip')

    # 出手速度
    vs = []
    for i, p0 in enumerate(SHOT_BALL):
        v = solve_v(p0, HOOP, ALPHA)
        if v is None:
            raise RuntimeError('抛射参数无解')
        vs.append(v)
        print("第 %d 球：出手点 %s，仰角 %.0f°，初速 %.2f m/s" % (i + 1, p0, ALPHA_DEG, v))

    renderer = mujoco.Renderer(model, HEIGHT, WIDTH)
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat = np.array(CAM_LOOKAT)
    cam.distance = CAM_DIST
    cam.azimuth = CAM_AZ
    cam.elevation = CAM_EL

    q0 = ik3(*HOME, None)
    if q0 is not None:
        data.qpos[qa[0]], data.qpos[qa[1]], data.qpos[qa[2]] = q0
    yaw_prev = q0[0] if q0 is not None else None
    mujoco.mj_forward(model, data)

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

    import time
    t0 = time.time()

    balls = [{'rel': SHOT_REL[0], 'p0': SHOT_BALL[0], 'v': vs[0], 'landed': False,
              'pos': np.array(SHOT_BALL[0], dtype=float), 'prev_z': None,
              'scored': False, 'traj': [], 'rest': None, 'show_from': 0},
             {'rel': SHOT_REL[1], 'p0': SHOT_BALL[1], 'v': vs[1], 'landed': False,
              'pos': np.array(SHOT_BALL[1], dtype=float), 'prev_z': None,
              'scored': False, 'traj': [], 'rest': None, 'show_from': 300}]

    frames = []
    score = 0

    for i in range(TOTAL):
        # ---- 臂：运动学插值 ----
        tgt = arm_target(i)
        q = ik3(tgt[0], tgt[1], tgt[2], yaw_prev)
        if q is not None:
            yaw_prev = q[0]
            for k in range(3):
                dq = q[k] - data.qpos[qa[k]]
                dq = max(-QSTEP, min(QSTEP, dq))
                data.qpos[qa[k]] += dq
        mujoco.mj_forward(model, data)
        ptip = data.xpos[tip].copy()

        # ---- 球：释放前跟随手掌，释放后解析抛物线 ----
        for j, b in enumerate(balls):
            if i < b['show_from']:
                pos = np.array([HOOP[0], 0.0, -2.0])        # 未上场：藏到地面以下
            elif i < b['rel']:
                pos = np.array([ptip[0], ptip[1], ptip[2] + GRIP_OFF])
            elif b['landed'] and b['rest'] is not None:
                pos = b['rest'].copy()
            else:
                t = (i - b['rel']) * DT_SIM
                pos = np.array(ball_pos(b['p0'], b['v'], ALPHA, t))
                # 入筐判定：球心穿过筐平面时检查水平偏差
                if (not b['scored']) and b['prev_z'] is not None and b['prev_z'] >= HOOP[2] > pos[2]:
                    r = math.hypot(pos[0] - HOOP[0], pos[1] - HOOP[1])
                    if r < R_HOOP - R_BALL + 0.04:
                        b['scored'] = True
                        score += 1
                b['prev_z'] = pos[2]
                # 落地后停在真实落点（不做位置跳变）
                if pos[2] < R_BALL:
                    pos[2] = R_BALL
                    b['landed'] = True
                    b['rest'] = pos.copy()
            b['pos'] = pos
            data.qpos[b_adr[j]:b_adr[j]+3] = pos
            data.qpos[b_adr[j]+3:b_adr[j]+7] = quat_q(0.0)

        mujoco.mj_forward(model, data)

        # ---- 渲染 ----
        renderer.update_scene(data, camera=cam)
        rgb = renderer.render().copy()

        im = Image.fromarray(rgb)
        draw = ImageDraw.Draw(im)

        for b in balls:
            p = b['pos']
            pr = project(p[0], p[1], p[2])
            if pr is not None and i >= b['rel'] and not b['landed']:
                b['traj'].append(pr)
            if len(b['traj']) >= 2:
                draw.line(b['traj'], fill=(255, 130, 30), width=2, joint='curve')

        # 顶部状态条
        shot_now = 1 if i < SHOT_REL[1] else 2
        draw.rectangle([0, 0, WIDTH, 26], fill=(16, 16, 18))
        draw.text((10, 6),
                  "BASKETBALL   shot %d / 2    score %d / 2    arc %.0f deg"
                  % (shot_now, score, ALPHA_DEG),
                  fill=(235, 235, 240), font=font)
        frames.append(im.quantize(colors=96))

        if (i + 1) % (TOTAL // 5) == 0:
            print("  已渲染 %d/%d 帧（%.0f%%）" % (i + 1, TOTAL, (i + 1) * 100.0 / TOTAL))

    print("命中数：%d / 2" % score)
    print("渲染完成，耗时 %.1f 秒，合成 GIF..." % (time.time() - t0))

    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "demo_armball.gif")
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
