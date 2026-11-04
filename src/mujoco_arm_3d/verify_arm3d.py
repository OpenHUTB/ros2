# -*- coding: utf-8 -*-
"""三自由度机械臂验证脚本（对应审阅关注点：IK 数值精度 + 实时渲染）

用法：
    python3 verify_arm3d.py ik     # IK 精度：螺旋上 8 个空间点 逆解->正解互查
    python3 verify_arm3d.py gui    # glfw 实时渲染：完整播放 9 秒 / 216 帧画螺旋线
"""
import os, math, sys
os.environ.setdefault("MUJOCO_GL", "glfw")
import numpy as np
import mujoco

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mujoco_arm_3d_single as m


def _joint_qadr(model):
    qa = {}
    for name in ('yaw', 'shoulder', 'elbow'):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name.encode())
        qa[name] = model.jnt_qposadr[jid]
    return qa


def run_ik():
    """螺旋线上 8 个空间点：ik3 逆解 -> 写入 qpos -> 仿真器正解 -> 对比目标。"""
    model = mujoco.MjModel.from_xml_string(m.MODEL_XML)
    data = mujoco.MjData(model)
    qa = _joint_qadr(model)
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
        data.qpos[qa['yaw']] = q[0]
        data.qpos[qa['shoulder']] = q[1]
        data.qpos[qa['elbow']] = q[2]
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


def run_gui():
    """glfw 实时渲染：查看器完整播放螺旋线画线流程（216 帧 / 9 秒）。"""
    import mujoco.viewer

    model = mujoco.MjModel.from_xml_string(m.MODEL_XML)
    data = mujoco.MjData(model)
    qa = _joint_qadr(model)
    total = int(m.DURATION * m.FPS)

    print("== glfw 实时渲染验证（%d 帧 / %.1f 秒） ==" % (total, m.DURATION))
    print("弹出 MuJoCo 查看器窗口，播放结束后自动关闭。")

    q_prev = None
    n = 0
    with mujoco.viewer.launch_passive(model, data) as v:
        for i in range(total):
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

            data.ctrl[0] = m.KP * (q[0] - data.qpos[qa['yaw']]) - m.KD * data.qvel[0]
            data.ctrl[1] = m.KP * (q[1] - data.qpos[qa['shoulder']]) - m.KD * data.qvel[1]
            data.ctrl[2] = m.KP * (q[2] - data.qpos[qa['elbow']]) - m.KD * data.qvel[2]
            for _ in range(3):
                mujoco.mj_step(model, data)
            v.sync()
            n += 1

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
