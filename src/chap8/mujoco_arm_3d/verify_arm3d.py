# -*- coding: utf-8 -*-
"""三自由度机械臂验证脚本（对应审阅关注点：IK 数值精度 + 实时渲染）

用法：
    python3 verify_arm3d.py ik     # IK 精度：螺旋上 8 个空间点 逆解->正解互查
    python3 verify_arm3d.py gui    # glfw 实时渲染：按 24 fps 播放 216 帧（约 9 秒），按 Enter 关闭
"""
import os, math, sys, time
os.environ.setdefault("MUJOCO_GL", "glfw")
import numpy as np
import mujoco

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import main as m


def _joint_adr(model):
    """按关节名查表得到 qpos / dof 地址，避免硬编码下标。"""
    adr = {}
    for name in ('yaw', 'shoulder', 'elbow'):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name.encode())
        if jid < 0:
            raise RuntimeError("joint %s not found" % name)
        adr[name] = (model.jnt_qposadr[jid], model.jnt_dofadr[jid])
    return adr


def run_ik():
    """螺旋线上 8 个空间点：ik3 逆解 -> 写入 qpos -> 仿真器正解 -> 对比目标。"""
    model = mujoco.MjModel.from_xml_string(m.MODEL_XML)
    data = mujoco.MjData(model)
    adr = _joint_adr(model)
    tip = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, b'tip')

    print("== IK 数值精度验证（螺旋线 8 个空间点） ==")
    max_err = 0.0
    for k in range(8):
        ang = 2.0 * math.pi * k / 8.0
        frac = k / 7.0
        tx = m.RADIUS * math.cos(ang)
        ty = m.RADIUS * math.sin(ang)
        tz = m.Z_LO + (m.Z_HI - m.Z_LO) * frac

        q = m.ik3(tx, ty, tz)
        if q is None:
            print("  [FAIL] 点 %d 超出工作空间" % k)
            sys.exit(1)
        data.qpos[adr['yaw'][0]] = q[0]
        data.qpos[adr['shoulder'][0]] = q[1]
        data.qpos[adr['elbow'][0]] = q[2]
        mujoco.mj_forward(model, data)
        p = data.xpos[tip]
        err = math.hypot(p[0] - tx, p[1] - ty) * 1000.0
        errz = abs(p[2] - tz) * 1000.0
        e = math.sqrt(err * err + errz * errz)
        max_err = max(max_err, e)
        print("  点%d 目标(%.3f, %.3f, %.3f) 逆解(%.3f, %.3f, %.3f) -> 正解(%.3f, %.3f, %.3f) 误差 %.4f mm"
              % (k, tx, ty, tz, q[0], q[1], q[2], p[0], p[1], p[2], e))
    print("最大误差：%.4f mm" % max_err)
    print("结果：" + ("PASS（<0.01 mm）" if max_err < 0.01 else "FAIL"))
    return 0 if max_err < 0.01 else 1


def _traj_points(total):
    """按 24 fps 采样螺旋线上的 216 个 3D 末端点（与 main.py 的轨迹公式一致）。"""
    pts = []
    for i in range(total):
        t = i / m.FPS
        ang = 2.0 * math.pi * (t % m.PERIOD) / m.PERIOD
        frac = t / m.DURATION
        pts.append(np.array([m.RADIUS * math.cos(ang),
                             m.RADIUS * math.sin(ang),
                             m.Z_LO + (m.Z_HI - m.Z_LO) * frac]))
    return pts


def _draw_traj(v, pts, width=0.006, rgba=(1.0, 0.82, 0.05, 1.0)):
    """把整条螺旋线画进 viewer 的 user_scn（黄色胶囊段）。

    main.py 出 GIF 时那条黄线是 PIL 在 2D 图像上叠画的像素折线，viewer 里没有；
    这里用 3D 几何体补上，看到的才是末端真正走过的那条空间曲线。
    """
    scn = v.user_scn
    scn.ngeom = 0
    for k in range(1, len(pts)):
        if scn.ngeom >= scn.maxgeom:
            print("轨迹段数超过 maxgeom，只画了前 %d 段" % scn.ngeom)
            break
        g = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE,
                            np.zeros(3), np.zeros(3), np.eye(3).flatten(),
                            np.array(rgba, dtype=np.float32))
        mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, width, pts[k - 1], pts[k])
        scn.ngeom += 1
    return scn.ngeom


def run_gui():
    """glfw 实时渲染：查看器按 24 fps 播放螺旋线画线流程（216 帧 / 9 秒）。

    驱动方式与主程序一致：IK 解直接写入 qpos 再 mj_forward（运动学驱动），
    末端严格落在螺旋线上；不写死 dof 下标，全部按关节名查表。
    窗口里额外画出整条 3D 轨迹（黄色胶囊段），播放按帧率实时推进，
    播完窗口保持打开，按 Enter 才退出。
    """
    import mujoco.viewer

    model = mujoco.MjModel.from_xml_string(m.MODEL_XML)
    data = mujoco.MjData(model)
    adr = _joint_adr(model)
    total = int(m.DURATION * m.FPS)
    pts = _traj_points(total)

    print("== glfw 实时渲染验证（%d 帧 / %.1f 秒） ==" % (total, m.DURATION))
    print("弹出 MuJoCo 查看器窗口：黄色胶囊段是 3D 螺旋线轨迹，末端品红球沿线运动。")
    print("按 %.1f 秒实时播放；播放结束后窗口会保持，按 Enter 关闭。" % m.DURATION)

    dt = 1.0 / m.FPS
    t_start = time.time()
    q_prev = None
    n = 0
    with mujoco.viewer.launch_passive(model, data) as v:
        seg = _draw_traj(v, pts)
        print("已在 viewer 里叠加 %d 段 3D 轨迹几何体（maxgeom=%d）" % (seg, v.user_scn.maxgeom))
        v.sync()
        for i in range(total):
            if not v.is_running():
                print("窗口已关闭，提前结束。")
                break
            t = i / m.FPS
            ang = 2.0 * math.pi * (t % m.PERIOD) / m.PERIOD
            frac = t / m.DURATION
            tx = m.RADIUS * math.cos(ang)
            ty = m.RADIUS * math.sin(ang)
            tz = m.Z_LO + (m.Z_HI - m.Z_LO) * frac

            q = m.ik3(tx, ty, tz)
            if q is None:
                continue
            q0 = q[0]
            if q_prev is not None:
                while q0 - q_prev[0] > math.pi:
                    q0 -= 2.0 * math.pi
                while q0 - q_prev[0] < -math.pi:
                    q0 += 2.0 * math.pi
            q = (q0, q[1], q[2])
            q_prev = q

            data.qpos[adr['yaw'][0]] = q[0]
            data.qpos[adr['shoulder'][0]] = q[1]
            data.qpos[adr['elbow'][0]] = q[2]
            data.qvel[adr['yaw'][1]] = 0.0
            data.qvel[adr['shoulder'][1]] = 0.0
            data.qvel[adr['elbow'][1]] = 0.0
            mujoco.mj_forward(model, data)
            # 每帧重建轨迹几何体：viewer 重建场景时 user_scn 里的叠加几何体会被丢掉，
            # 只在 with 开头画一次的话，后续帧能不能留下取决于 viewer 内部时序，逐帧重画最稳。
            _draw_traj(v, pts)
            v.sync()
            n += 1
            # 补足到下一帧时刻：不加这个 sleep，216 帧会在 0.2 秒内跑完，窗口一闪而过
            wait = t_start + n * dt - time.time()
            if wait > 0:
                time.sleep(wait)

        print("播放完成，窗口保持打开；按 Enter 关闭窗口...")
        try:
            input()
        except (EOFError, KeyboardInterrupt):
            pass

    print("实时渲染播放完成：%d 帧 / %.1f 秒" % (n, n / m.FPS))
    return 0


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "ik"
    if mode == "ik":
        sys.exit(run_ik())
    elif mode == "gui":
        sys.exit(run_gui())
    else:
        print("用法：python3 verify_arm3d.py [ik|gui]")
        sys.exit(1)
