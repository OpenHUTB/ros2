#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MuJoCo 二连杆机械臂逆运动学（IK）画圆轨迹

功能：机械臂末端沿圆形轨迹运动，用解析 2 连杆 IK 实时解算关节角，
关节 PD 位置控制驱动，逐帧离屏渲染并合成 GIF 效果图。

关键设计：
  1) 黄色轨迹线画的是 data.site_xpos 读到的**末端实际位置**，不是把目标点投影上去充数；
  2) 运行时打印**仿真侧跟踪误差**（圆上 8 个等分点的末端实际位置 vs 目标），
     与 IK→FK 回代误差（解算精度，见 verify_ik.py ik）分开表述；
  3) 两个 hinge 关节在 arm_2link.xml 里补了 range（避免多圈累积后关节越界自穿），
     IK 侧对 q1 做 ±π 归一化；
  4) 像素投影用画面里 16 个绿色参考点自动标定（最小二乘），并打印重投影残差，
     不再依赖手写相机公式。

运行：
    python3 main.py                       # 自动选渲染后端
    MUJOCO_GL=osmesa python3 main.py      # 无显示器虚拟机（软渲染）
    MUJOCO_GL=glfw   python3 main.py      # 有桌面 / 有显卡
    python3 main.py --out ./demo.gif      # 指定输出

渲染后端默认值：
    Windows 或存在 DISPLAY   -> glfw
    无 DISPLAY（无头 Linux） -> osmesa
显式设置 MUJOCO_GL 时以环境变量为准。
"""
import argparse
import math
import os
import sys
import time

# 渲染后端：未显式指定时按平台/显示环境自动选择，避免无显示机器直接崩
if "MUJOCO_GL" not in os.environ:
    if sys.platform.startswith("win") or os.environ.get("DISPLAY"):
        os.environ["MUJOCO_GL"] = "glfw"
    else:
        os.environ["MUJOCO_GL"] = "osmesa"

import numpy as np
import mujoco
from PIL import Image, ImageDraw, ImageFont

# HUD 用默认位图字体（跨平台、无需额外字体文件）；Pillow>=10.1 支持指定字号
try:
    HUD_FONT = ImageFont.load_default(size=18)
except TypeError:                                   # 老版本 Pillow
    HUD_FONT = ImageFont.load_default()

_HERE = os.path.dirname(os.path.abspath(__file__))
# 模型默认取脚本同目录；被 launch 从 devel/lib/ 启动时用环境变量传入 share/ 下的模型路径
MODEL = os.environ.get("MUJOCO_ARM_IK_MODEL") or os.path.join(_HERE, "arm_2link.xml")

L1 = 0.30                       # 连杆长度（与模型一致）
L2 = 0.30

CX, CY = 0.30, 0.0              # 圆心（世界坐标，x 朝前、y 朝左）
RADIUS = 0.15                   # 轨迹半径
PERIOD = 6.0                    # 走完一圈的秒数
DURATION = 8.0                  # 总时长（秒）
FPS = 24
GIF_WARN_MB = 5.0               # GIF 体积上限（MB），落盘后超限就打印警告

KP = 600.0                      # 肩关节 PD 比例增益
KD = 15.0                       # 肩关节 PD 微分增益
KP2 = 1400.0                    # 肘关节比例增益（惯量约为肩的一半，需更高）
KD2 = 18.0                      # 肘关节微分增益
PHYS_DT = 0.002                 # 模型 timestep，与 arm_2link.xml 的 option 一致
# 每帧物理子步：1/24 s = 41.7 ms，按 2 ms 步长需要 21 步，物理时间才与轨迹推进对齐
SUBSTEPS = int(round((1.0 / FPS) / PHYS_DT))

REF_NAMES = ["rc%d" % i for i in range(16)]     # 16 个绿色参考点


def ik2(x, y):
    """二连杆平面机械臂解析逆运动学，返回 (q1, q2)；超出工作空间返回 None。

    q1 归一化到 (-π, π]，配合模型 hinge 的 range 避免多圈累积后超出限位。
    """
    d = math.hypot(x, y)
    if d > L1 + L2 - 1e-6 or d < abs(L1 - L2) + 1e-6:
        return None
    cos_q2 = (x * x + y * y - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
    q2 = math.acos(max(-1.0, min(1.0, cos_q2)))      # 肘下姿势（q2 > 0）
    q1 = math.atan2(y, x) - math.atan2(L2 * math.sin(q2),
                                       L1 + L2 * math.cos(q2))
    q1 = (q1 + math.pi) % (2.0 * math.pi) - math.pi
    return q1, q2


def _label_green(rgb):
    """分离画面里的绿色参考点，返回每个点的像素质心（u, v）。"""
    isg = ((rgb[:, :, 1] > 105) & (rgb[:, :, 0] < 115) & (rgb[:, :, 2] < 115))
    ys, xs = np.nonzero(isg)
    if len(xs) == 0:
        return []
    unvisited = set(zip(xs.tolist(), ys.tolist()))
    blobs = []
    while unvisited:
        seed = next(iter(unvisited))
        stack = [seed]
        unvisited.discard(seed)
        comp = []
        while stack:
            cu, cv = stack.pop()
            comp.append((cu, cv))
            for du in (-1, 0, 1):
                for dv in (-1, 0, 1):
                    nb = (cu + du, cv + dv)
                    if nb in unvisited:
                        unvisited.discard(nb)
                        stack.append(nb)
        if 18 <= len(comp) <= 95:            # 参考点约 43~50 px；滤掉碎片与大块背景
            arr = np.array(comp, dtype=float)
            blobs.append((arr[:, 0].mean(), arr[:, 1].mean()))
    return blobs


def calibrate_projection(model, data, renderer, cam):
    """渲染一帧，用 16 个绿色参考点拟合 世界(x,y) -> 像素(u,v) 的仿射矩阵。

    返回 (M, 平均残差 px, 参与点数)；M 为 2x3：
        u = M00*x + M01*y + M02
        v = M10*x + M11*y + M12
    """
    saved = data.qpos.copy()
    data.qpos[0], data.qpos[1] = math.pi, 0.0      # 机械臂转到 -x，避免遮挡地面参考点
    mujoco.mj_forward(model, data)
    renderer.update_scene(data, camera=cam)
    rgb = renderer.render()
    data.qpos[:] = saved
    mujoco.mj_forward(model, data)

    blobs = _label_green(rgb)
    # 参考点都在圆上、半径约 72 px，离画面中心超出 110 px 的必为背景/杂物
    blobs = [b for b in blobs if math.hypot(b[0] - 320.0, b[1] - 240.0) < 110.0]
    world = []
    for nm in REF_NAMES:
        gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, nm)
        if gid >= 0:
            world.append((float(model.geom_pos[gid][0]), float(model.geom_pos[gid][1])))
    if len(blobs) < 8 or len(world) < 8:
        return None, -1.0, 0

    # 以质心为极点按角度排序，与圆上参考点的角度顺序一一配对。
    # 像素 v 轴朝下、世界 y 轴朝上，所以两种翻转都试一遍，取重投影残差小的那组。
    cu = float(np.mean([b[0] for b in blobs]))
    cv = float(np.mean([b[1] for b in blobs]))
    world.sort(key=lambda w: math.atan2(w[1] - CY, w[0] - CX))
    best = None
    for flip in (1.0, -1.0):
        bl_sorted = sorted(blobs, key=lambda b: math.atan2(flip * (b[1] - cv), b[0] - cu))
        n = min(len(bl_sorted), len(world))
        A = np.array([[w[0], w[1], 1.0] for w in world[:n]])
        Bu = np.array([b[0] for b in bl_sorted[:n]])
        Bv = np.array([b[1] for b in bl_sorted[:n]])
        Mu, *_ = np.linalg.lstsq(A, Bu, rcond=None)
        Mv, *_ = np.linalg.lstsq(A, Bv, rcond=None)
        res = np.hypot(A.dot(Mu) - Bu, A.dot(Mv) - Bv)
        score = float(res.mean())
        if best is None or score < best[0]:
            best = (score, np.vstack([Mu, Mv]), n)
    return best[1], best[0], best[2]


def make_projector(M):
    """把仿射矩阵包装成 project(wx, wy) -> (px, py)。"""
    def project(wx, wy):
        u = M[0, 0] * wx + M[0, 1] * wy + M[0, 2]
        v = M[1, 0] * wx + M[1, 1] * wy + M[1, 2]
        return int(round(u)), int(round(v))
    return project


def _prepare_out_path(path):
    """检查并准备输出路径：父目录缺失就创建，不可写或指向目录则抛 OSError。

    渲染前就做掉这件事，避免跑完 8 秒才崩在 save()。
    """
    p = os.path.abspath(path)
    if os.path.isdir(p):
        raise OSError("输出路径是一个已存在的目录，请给出 .gif 文件名：%s" % p)
    d = os.path.dirname(p) or "."
    if not os.path.isdir(d):
        os.makedirs(d)
    if not os.access(d, os.W_OK):
        raise OSError("输出目录不可写：%s" % d)
    return p


def resolve_out(cli_out):
    """GIF 输出路径：--out > 环境变量 MUJOCO_ARM_IK_OUT > 仓库内 docs/img > 当前目录。

    显式指定的路径先做可写检查；默认路径本身就带三级回退。
    """
    explicit = cli_out or os.environ.get("MUJOCO_ARM_IK_OUT")
    if explicit:
        return _prepare_out_path(explicit)
    root = os.path.abspath(os.path.join(_HERE, "..", "..", ".."))
    img_dir = os.path.join(root, "docs", "img")
    if (os.path.isfile(os.path.join(root, "mkdocs.yml")) and os.path.isdir(img_dir)
            and os.access(img_dir, os.W_OK)):
        return os.path.join(img_dir, "mujoco_arm_ik_demo.gif")
    return os.path.join(_HERE, "demo.gif")


def main():
    ap = argparse.ArgumentParser(description="MuJoCo 二连杆机械臂 IK 画圆仿真")
    ap.add_argument("--out", default=None,
                    help="GIF 输出路径（默认写仓库内 docs/img，仓库外退当前目录）")
    args = ap.parse_args()

    try:
        out_gif = resolve_out(args.out)
    except OSError as e:
        print("[error] 输出路径不可用：%s" % e)
        print("        请用 --out 指定一个可写路径，例如 --out ./demo.gif")
        sys.exit(1)
    model = mujoco.MjModel.from_xml_path(MODEL)
    data = mujoco.MjData(model)

    tip = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "tip_site")
    if tip < 0:
        print("[error] 模型里找不到 site 'tip_site'，无法读取末端实际位置。")
        sys.exit(1)

    renderer = mujoco.Renderer(model, 480, 640)
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat = np.array([0.3, 0.0, 0.05])
    cam.distance = 1.15
    cam.azimuth = 0
    cam.elevation = -90

    M, res_px, n_pts = calibrate_projection(model, data, renderer, cam)
    if M is None:
        print("[warn] 可用参考点不足，退回默认俯视投影（像素可能与画面有偏差）")
        M = np.array([[482.8, 0.0, -144.8], [0.0, -482.8, 144.8]])
    else:
        print("投影标定：%d/%d 个参考点有效，平均重投影误差 %.2f px" %
              (n_pts, len(REF_NAMES), res_px))
    project = make_projector(M)

    frames = []
    traj = []
    max_traj = int(PERIOD * FPS)
    total_frames = int(DURATION * FPS)
    err_all = []
    err_at = {}

    # 预热：沿轨迹先空跑一圈让 PD 进入稳态（不计帧、不留轨迹、不统计误差）
    q0 = ik2(CX + RADIUS, CY)
    if q0 is not None:
        data.qpos[0], data.qpos[1] = q0
        data.qvel[:] = 0.0
        mujoco.mj_forward(model, data)
    for k in range(int(PERIOD * FPS)):
        angk = 2.0 * math.pi * ((k / FPS) % PERIOD) / PERIOD
        qk = ik2(CX + RADIUS * math.cos(angk), CY + RADIUS * math.sin(angk))
        if qk is None:
            continue
        for _ in range(SUBSTEPS):
            data.ctrl[0] = KP * (qk[0] - data.qpos[0]) - KD * data.qvel[0]
            data.ctrl[1] = KP2 * (qk[1] - data.qpos[1]) - KD2 * data.qvel[1]
            mujoco.mj_step(model, data)

    print("开始 IK 轨迹跟踪：圆心 (%.2f, %.2f) 半径 %.2f，共 %d 帧，后端 %s" %
          (CX, CY, RADIUS, total_frames, os.environ.get("MUJOCO_GL")))
    t0 = time.time()

    for i in range(total_frames):
        t = i / FPS
        ang = 2.0 * math.pi * (t % PERIOD) / PERIOD
        tx = CX + RADIUS * math.cos(ang)
        ty = CY + RADIUS * math.sin(ang)

        q = ik2(tx, ty)
        if q is None:
            print("  [warn] 目标点在机械臂工作空间外：x=%.2f y=%.2f" % (tx, ty))
            continue

        # PD 控制：每个物理子步都按当前关节状态重算力矩（2 ms 周期闭环）。
        # 若每帧只算一次并保持 21 步，等效控制周期变成 42 ms，末端会大幅滞后并震荡。
        for _ in range(SUBSTEPS):
            data.ctrl[0] = KP * (q[0] - data.qpos[0]) - KD * data.qvel[0]
            data.ctrl[1] = KP2 * (q[1] - data.qpos[1]) - KD2 * data.qvel[1]
            mujoco.mj_step(model, data)

        # 末端实际位置（引擎读数，不是目标点）
        px_act, py_act = data.site_xpos[tip][0], data.site_xpos[tip][1]
        err = math.hypot(px_act - tx, py_act - ty)
        err_all.append(err)
        k8 = int(round(ang / (2.0 * math.pi) * 8)) % 8
        if k8 not in err_at:
            err_at[k8] = err

        renderer.update_scene(data, camera=cam)
        rgb = renderer.render().copy()

        # 轨迹线画的是末端实际位置
        traj.append(project(px_act, py_act))
        if len(traj) > max_traj:
            traj = traj[-max_traj:]

        im = Image.fromarray(rgb)
        draw = ImageDraw.Draw(im)

        # 帧号 + 两关节角实时读数：静态截图也能看出这是运动过程中的某一帧，
        # 每帧数值都不同，避免被误读成「机械臂不动、只有轨迹在动」
        draw.text((10, 8),
                  "frame %03d/%d    q1 %+6.1f    q2 %+6.1f   deg"
                  % (i + 1, total_frames,
                     math.degrees(data.qpos[0]), math.degrees(data.qpos[1])),
                  fill=(255, 255, 255), font=HUD_FONT,
                  stroke_width=2, stroke_fill=(0, 0, 0))

        if len(traj) >= 2:
            for k in range(1, len(traj)):
                draw.line([traj[k - 1], traj[k]], fill=(255, 255, 0), width=5)
        # 压到 128 色：全彩 192 帧 640×480 会到 8 MB，超出 GIF < 5 MB 的仓库约定；
        # 再低（96 色）会把地面绿点压成灰、黄轨迹压成红，所以 128 是下限
        frames.append(im.quantize(colors=128, method=Image.MEDIANCUT))

        if (i + 1) % (total_frames // 4) == 0:
            print("  已渲染 %d/%d 帧（%.0f%%）" % (i + 1, total_frames,
                                                  (i + 1) * 100.0 / total_frames))

    print("渲染完成，耗时 %.1f 秒，合成 GIF..." % (time.time() - t0))
    if not frames:
        print("未生成任何帧！请检查模型文件与渲染后端（可设 MUJOCO_GL=osmesa 或 egl）。")
        sys.exit(1)

    frames[0].save(out_gif, save_all=True, append_images=frames[1:],
                   duration=int(1000 / FPS), loop=0, optimize=True)

    # GIF 体积对 Pillow 版本很敏感：同一套帧，Pillow 7 编出来 33 MB、Pillow 13 只有 4.6 MB。
    # 落盘后立刻量一次，超过上限直接提示，避免超大文件被误提交进仓库。
    gif_mb = os.path.getsize(out_gif) / 1024.0 / 1024.0
    if gif_mb > GIF_WARN_MB:
        print("[warn] GIF 体积 %.2f MB 已超过 %.1f MB 上限，通常是 Pillow 版本偏低导致编码效率差；"
              "建议 pip3 install --user -U 'pillow>=9.0' 后重跑。" % (gif_mb, GIF_WARN_MB))

    # 帧数与时长必须从落盘文件读回：GIF 的帧延迟以 1/100 s 为单位，
    # 这里写的 41 ms 会被落成 40 ms；再按「帧数 × 41」算出来的时长是错的。
    n_file, total_ms = len(frames), len(frames) * int(1000 / FPS)
    try:
        gif = Image.open(out_gif)
        n_file = getattr(gif, "n_frames", 1)
        total_ms = 0
        for k in range(n_file):
            gif.seek(k)
            total_ms += gif.info.get("duration", int(1000 / FPS))
    except Exception as e:
        print("[warn] 读回 GIF 元信息失败，退回按内存帧数估算：%s" % e)
        n_file, total_ms = len(frames), len(frames) * int(1000 / FPS)

    avg_ms = (total_ms / float(n_file)) if n_file else 0.0
    print("已生成效果图：%s（文件内 %d 帧，每帧约 %d ms，合计 %.2f 秒，%.2f MB）" %
          (out_gif, n_file, int(round(avg_ms)), total_ms / 1000.0,
           os.path.getsize(out_gif) / 1024.0 / 1024.0))
    print("  内存帧数 %d；与文件帧数%s（相同帧被 GIF 合并时会变少）" %
          (len(frames), "一致" if n_file == len(frames) else "不同"))
    print("输出位置：%s" % out_gif)

    # ---- 仿真侧跟踪误差（末端实际位置 vs 目标），与 IK 解算误差分开报告 ----
    if err_all:
        avg_mm = sum(err_all) / len(err_all) * 1000.0
        max_mm = max(err_all) * 1000.0
        print("")
        print("跟踪误差（仿真侧：末端实际位置 vs 目标点，含 PD 控制与物理步进）")
        for k in range(8):
            if k in err_at:
                print("  %3d°: %.3f mm" % (k * 45, err_at[k] * 1000.0))
        print("  全帧平均 %.3f mm / 最大 %.3f mm（%d 帧）" % (avg_mm, max_mm, len(err_all)))
        print("  注：IK→FK 回代误差（纯解算精度，不含控制）见 verify_ik.py ik，为 0.0000 mm；")
        print("      两者口径不同，不能互相替代。")


if __name__ == "__main__":
    main()
