# -*- coding: utf-8 -*-
"""拧螺丝项目验证脚本（三种模式）

用法：
    python verify_armscrew.py ik      # IK 精度：目标点逆解 -> 正解互查
    python verify_armscrew.py clear   # 无渲染全流程：刀头贴合 / 转角 / 下沉量
    python verify_armscrew.py gui     # glfw 实时渲染：完整播放拧螺丝
"""
import os, sys, math

os.environ.setdefault("MUJOCO_GL", "glfw")
import numpy as np
import mujoco

import mujoco_arm_screw_single as m


def fk3(yaw, q1, q2):
    """三自由度正解：关节角 -> 末端 tip 位置。"""
    r = m.L1 * math.cos(q1) + m.L2 * math.cos(q1 + q2)
    z = m.L1 * math.sin(q1) + m.L2 * math.sin(q1 + q2) + m.BASE_Z
    return (r * math.cos(yaw), r * math.sin(yaw), z)


def run_ik():
    print("== 拧螺丝 IK 精度验证（目标点逆解 -> 正解互查） ==")
    down_z = m.SCREW_TOP_Z + m.SCREW_HALF + m.DRIVER_OFF
    targets = [
        (m.SCREW_XY[0], m.SCREW_XY[1], down_z),      # 刀头贴螺丝顶
        (m.SCREW_XY[0], m.SCREW_XY[1], 0.22),        # 螺丝上方待机
        m.HOME,                                       # 待机点
    ]
    worst = 0.0
    for i, t in enumerate(targets):
        q = m.ik3(*t, None)
        if q is None:
            print("  目标 %d %s：不可达 FAIL" % (i, t))
            worst = 1e9
            continue
        p = fk3(*q)
        err = math.dist(p, t) * 1000.0
        worst = max(worst, err)
        print("  目标 %d %s -> q=(yaw %.3f, q1 %.3f, q2 %.3f) 正解误差 %.2f mm"
              % (i, t, q[0], q[1], q[2], err))
    ok = worst < 0.5
    print("逆向/正向最大误差：%.2f mm -> %s" % (worst, "PASS" if ok else "FAIL"))
    return ok


def run_clear():
    print("== 拧螺丝全流程验证（无渲染，%d 帧） ==" % m.TOTAL)
    model = mujoco.MjModel.from_xml_string(m.MODEL_XML)
    data = mujoco.MjData(model)

    def jq(name):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name.encode())
        return model.jnt_qposadr[jid]

    qa = (jq('yaw'), jq('pitch1'), jq('pitch2'))
    s_adr = jq('screwfree')

    q0 = m.ik3(*m.HOME, None)
    if q0 is not None:
        data.qpos[qa[0]], data.qpos[qa[1]], data.qpos[qa[2]] = q0
    yaw_prev = q0[0] if q0 is not None else None
    mujoco.mj_forward(model, data)

    tip_b = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, b'tip')
    max_gap = 0.0
    max_gap_frame = -1
    theta_max_seen = 0.0
    z_min = 1e9
    for i in range(m.TOTAL):
        tgt, theta, sz = m.screw_state(i)
        q = m.ik3(tgt[0], tgt[1], tgt[2], yaw_prev)
        if q is not None:
            yaw_prev = q[0]
            for k in range(3):
                dq = q[k] - data.qpos[qa[k]]
                dq = max(-m.QSTEP, min(m.QSTEP, dq))
                data.qpos[qa[k]] += dq
        mujoco.mj_forward(model, data)

        # 只在「对位 + 拧入」阶段检查刀头与螺丝顶贴合（抬起段刀头离开是正常的）
        if m.PRESS_END <= i < m.SCREW_END:
            # 刀头位置 = tip - DRIVER_OFF（竖直向下），与螺丝顶面比较
            blade_z = data.xpos[tip_b][2] - m.DRIVER_OFF
            screw_top = sz + m.SCREW_HALF
            gap = abs(blade_z - screw_top)
            if gap > max_gap:
                max_gap, max_gap_frame = gap, i
        theta_max_seen = theta
        z_min = min(z_min, sz)

    print("刀头-螺丝顶最大间隙：%.4f m（frame %d）" % (max_gap, max_gap_frame))
    print("最终转角：%.1f°（%.2f 圈，要求 %.2f 圈）"
          % (math.degrees(theta_max_seen), theta_max_seen / (2 * math.pi), m.TURNS))
    print("螺丝中心终了高度：%.4f m（初始 %.4f，下沉 %.1f mm）"
          % (z_min, m.SCREW_TOP_Z, (m.SCREW_TOP_Z - z_min) * 1000))
    print("螺丝终了顶面高度：%.4f m（板面 0.020 m）" % (z_min + m.SCREW_HALF))

    ok_turn = abs(theta_max_seen - m.THETA_MAX) < 1e-6
    ok_gap = max_gap < 0.005
    ok_depth = abs((m.SCREW_TOP_Z - z_min) - abs(m.START_Z)) < 1e-6
    print("转角 PASS=%s | 贴合 PASS=%s | 下沉量 PASS=%s"
          % (ok_turn, ok_gap, ok_depth))
    ok = ok_turn and ok_gap and ok_depth
    print("结果：%s" % ("PASS" if ok else "FAIL"))
    return ok


def run_gui():
    print("== glfw 实时渲染：完整播放拧螺丝 ==")
    model = mujoco.MjModel.from_xml_string(m.MODEL_XML)
    data = mujoco.MjData(model)

    def jq(name):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name.encode())
        return model.jnt_qposadr[jid]

    qa = (jq('yaw'), jq('pitch1'), jq('pitch2'))
    d_adr = jq('driverfree')
    s_adr = jq('screwfree')
    tip_b = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, b'tip')

    q0 = m.ik3(*m.HOME, None)
    if q0 is not None:
        data.qpos[qa[0]], data.qpos[qa[1]], data.qpos[qa[2]] = q0
    yaw_prev = q0[0] if q0 is not None else None
    mujoco.mj_forward(model, data)

    import time
    with mujoco.viewer.launch_passive(model, data) as v:
        for i in range(m.TOTAL):
            tgt, theta, sz = m.screw_state(i)
            q = m.ik3(tgt[0], tgt[1], tgt[2], yaw_prev)
            if q is not None:
                yaw_prev = q[0]
                for k in range(3):
                    dq = q[k] - data.qpos[qa[k]]
                    dq = max(-m.QSTEP, min(m.QSTEP, dq))
                    data.qpos[qa[k]] += dq
            mujoco.mj_forward(model, data)

            ptip = data.xpos[tip_b]
            data.qpos[d_adr:d_adr+3] = (ptip[0], ptip[1], ptip[2] - m.DRIVER_OFF)
            data.qpos[d_adr+3:d_adr+7] = m.quat_z(theta)
            data.qpos[s_adr:s_adr+3] = (m.SCREW_XY[0], m.SCREW_XY[1], sz)
            data.qpos[s_adr+3:s_adr+7] = m.quat_z(theta)
            mujoco.mj_forward(model, data)
            v.sync()
            time.sleep(1.0 / m.FPS)
    print("播放完成（%d 帧）" % m.TOTAL)


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "clear"
    if mode == "ik":
        sys.exit(0 if run_ik() else 1)
    elif mode == "gui":
        run_gui()
    else:
        sys.exit(0 if run_clear() else 1)
