# -*- coding: utf-8 -*-
"""投篮项目验证脚本（三种模式）

用法：
    python verify_armball.py ik      # IK 精度：目标点逆解 -> 正解互查
    python verify_armball.py swing   # 上摆动作：释放前手部抬升量 + 释放连续性
    python verify_armball.py clear   # 无渲染全流程：出手速度 / 入筐偏差 / 命中数
"""
import os, sys, math

os.environ.setdefault("MUJOCO_GL", "glfw")
import numpy as np
import mujoco

import mujoco_arm_ball_single as m


def fk3(yaw, q1, q2):
    """三自由度正解：关节角 -> 末端 tip 位置。"""
    r = m.L1 * math.cos(q1) + m.L2 * math.cos(q1 + q2)
    z = m.L1 * math.sin(q1) + m.L2 * math.sin(q1 + q2) + m.BASE_Z
    return (r * math.cos(yaw), r * math.sin(yaw), z)


def _tip_targets():
    """需要精确到达的末端 tip 目标点：待机 / 蓄力 / 出手。"""
    out = [("待机 HOME", m.HOME)]
    for i in (0, 1):
        out.append(("第%d球蓄力点" % (i + 1),
                    (m.PRE_BALL[i][0], m.PRE_BALL[i][1], m.PRE_BALL[i][2] - m.GRIP_OFF)))
        out.append(("第%d球出手点" % (i + 1),
                    (m.SHOT_BALL[i][0], m.SHOT_BALL[i][1], m.SHOT_BALL[i][2] - m.GRIP_OFF)))
    return out


def _make_env():
    """建模型、初始化臂姿态，返回常用句柄。"""
    model = mujoco.MjModel.from_xml_string(m.MODEL_XML)
    data = mujoco.MjData(model)

    def jq(name):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name.encode())
        return model.jnt_qposadr[jid]

    qa = (jq('yaw'), jq('pitch1'), jq('pitch2'))
    b_adr = [jq('ball1free'), jq('ball2free')]
    tip_b = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, b'tip')

    q0 = m.ik3(*m.HOME, None)
    if q0 is not None:
        data.qpos[qa[0]], data.qpos[qa[1]], data.qpos[qa[2]] = q0
    yaw_prev = [q0[0] if q0 is not None else None]
    mujoco.mj_forward(model, data)
    return model, data, qa, b_adr, tip_b, yaw_prev


def _step_arm(model, data, qa, tip_b, yaw_prev, frame):
    """推进一帧：末端目标 -> IK -> 限步长逼近 -> mj_forward，返回 tip 世界坐标。"""
    tgt = m.arm_target(frame)
    q = m.ik3(tgt[0], tgt[1], tgt[2], yaw_prev[0])
    if q is not None:
        yaw_prev[0] = q[0]
        for k in range(3):
            dq = q[k] - data.qpos[qa[k]]
            dq = max(-m.QSTEP, min(m.QSTEP, dq))
            data.qpos[qa[k]] += dq
    mujoco.mj_forward(model, data)
    return data.xpos[tip_b]


# ---------------------------------------------------------------- IK 精度
def run_ik():
    print("== 投篮 IK 精度验证（目标点逆解 -> 正解互查） ==")
    worst = 0.0
    for name, t in _tip_targets():
        q = m.ik3(*t, None)
        if q is None:
            print("  %s %s：不可达 FAIL" % (name, t))
            worst = 1e9
            continue
        p = fk3(*q)
        err = math.dist(p, t) * 1000.0
        worst = max(worst, err)
        print("  %s %s -> q=(yaw %.3f, q1 %.3f, q2 %.3f) 正解误差 %.2f mm"
              % (name, t, q[0], q[1], q[2], err))
    ok = worst < 0.5
    print("逆向/正向最大误差：%.2f mm -> %s" % (worst, "PASS" if ok else "FAIL"))
    return ok


# ---------------------------------------------------------------- 上摆动作
def run_swing():
    print("== 上摆动作验证（无渲染，%d 帧） ==" % m.TOTAL)
    model, data, qa, b_adr, tip_b, yaw_prev = _make_env()

    tip_z = {}
    for i in range(m.TOTAL):
        tip_z[i] = float(_step_arm(model, data, qa, tip_b, yaw_prev, i)[2])

    ok_all = True
    for j, rel in enumerate(m.SHOT_REL):
        z_low = tip_z[rel - 12]          # 上摆起点（蓄力位）
        z_high = tip_z[rel]              # 上摆终点（出手位）
        rise = z_high - z_low
        # 单调性：上摆段应逐帧上升（允许 1 mm 抖动）
        mono = all(tip_z[i + 1] >= tip_z[i] - 0.001 for i in range(rel - 12, rel))
        # 逐帧平均抬升速度
        speed = rise / 12.0

        # 释放连续性：释放帧手已摆到出手点，球从出手点起飞
        hand_ball = z_high + m.GRIP_OFF                      # 释放瞬间球心应在手托位置
        ball_start = m.SHOT_BALL[j][2]
        gap = abs(hand_ball - ball_start) * 1000.0
        # 释放前一帧球已随手上抬到 z[rel-1]+GRIP_OFF，与起飞点只差一帧球速
        prev_hand = tip_z[rel - 1] + m.GRIP_OFF
        step_gap = (ball_start - prev_hand) * 1000.0
        v = m.solve_v(m.SHOT_BALL[j], m.HOOP, m.ALPHA)
        v_z = v * math.sin(m.ALPHA)
        one_frame = v_z * m.DT_SIM * 1000.0                   # 球一帧上升量（mm）

        print("  第%d球（释放帧 %d）：手 tip z %.3f -> %.3f，抬升 %.1f mm / 12 帧"
              % (j + 1, rel, z_low, z_high, rise * 1000))
        print("        逐帧单调上升：%s；平均抬升 %.1f mm/帧（球 %.1f mm/帧）"
              % ("是" if mono else "否", speed * 1000, one_frame))
        print("        释放瞬间手托球心 %.3f，起飞点 %s，差 %.1f mm"
              % (hand_ball, ball_start, gap))
        print("        释放前一帧球心 %.3f，与起飞点差 %.1f mm（球一帧 %.1f mm）"
              % (prev_hand, step_gap, one_frame))
        ok = (rise > 0.10 and mono and gap < 5.0
              and -1.0 < step_gap < one_frame * 2.0)
        ok_all = ok_all and ok
        print("        判据（抬升>100 mm 且单调 且释放衔接<5 mm 且末帧步长合理）：%s"
              % ("PASS" if ok else "FAIL"))
    print("结果：%s" % ("PASS" if ok_all else "FAIL"))
    return ok_all


# ---------------------------------------------------------------- 全流程
def run_clear():
    print("== 投篮全流程验证（无渲染，%d 帧） ==" % m.TOTAL)
    model, data, qa, b_adr, tip_b, yaw_prev = _make_env()

    vs = [m.solve_v(p, m.HOOP, m.ALPHA) for p in m.SHOT_BALL]
    for i, v in enumerate(vs):
        print("  第%d球出手速度 %.2f m/s（出手点 %s，仰角 %.0f°）"
              % (i + 1, v, m.SHOT_BALL[i], m.ALPHA_DEG))

    balls = [{'rel': m.SHOT_REL[0], 'p0': m.SHOT_BALL[0], 'v': vs[0],
              'prev_z': None, 'scored': False, 'miss_r': None, 'landed': False,
              'rest': None, 'show_from': 0},
             {'rel': m.SHOT_REL[1], 'p0': m.SHOT_BALL[1], 'v': vs[1],
              'prev_z': None, 'scored': False, 'miss_r': None, 'landed': False,
              'rest': None, 'show_from': 300}]

    score = 0
    for i in range(m.TOTAL):
        ptip = _step_arm(model, data, qa, tip_b, yaw_prev, i)
        for j, b in enumerate(balls):
            if i < b['show_from']:
                pos = np.array([m.HOOP[0], 0.0, -2.0])
            elif i < b['rel']:
                pos = np.array([ptip[0], ptip[1], ptip[2] + m.GRIP_OFF])
            elif b['landed'] and b['rest'] is not None:
                pos = b['rest'].copy()
            else:
                t = (i - b['rel']) * m.DT_SIM
                pos = np.array(m.ball_pos(b['p0'], b['v'], m.ALPHA, t))
                if (not b['scored']) and b['prev_z'] is not None \
                        and b['prev_z'] >= m.HOOP[2] > pos[2]:
                    r = math.hypot(pos[0] - m.HOOP[0], pos[1] - m.HOOP[1])
                    b['miss_r'] = r
                    if r < m.R_HOOP - m.R_BALL + 0.04:
                        b['scored'] = True
                        score += 1
                b['prev_z'] = pos[2]
                if pos[2] < m.R_BALL:
                    pos[2] = m.R_BALL
                    b['landed'] = True
                    b['rest'] = pos.copy()
            data.qpos[b_adr[j]:b_adr[j]+3] = pos
            data.qpos[b_adr[j]+3:b_adr[j]+7] = m.quat_q(0.0)
        mujoco.mj_forward(model, data)

    for j, b in enumerate(balls):
        r = b['miss_r']
        tol = m.R_HOOP - m.R_BALL + 0.04
        print("  第%d球：穿过筐平面时水平偏差 %.3f m（容差 %.3f m）-> %s"
              % (j + 1, r if r is not None else float('nan'), tol,
                 "命中" if b['scored'] else "未中"))

    print("命中数：%d / 2" % score)
    print("篮筐内半径 %.3f m，球半径 %.3f m" % (m.R_HOOP, m.R_BALL))
    ok = score == 2
    print("结果：%s" % ("PASS" if ok else "FAIL"))
    return ok


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "clear"
    if mode == "ik":
        sys.exit(0 if run_ik() else 1)
    elif mode == "swing":
        sys.exit(0 if run_swing() else 1)
    else:
        sys.exit(0 if run_clear() else 1)
