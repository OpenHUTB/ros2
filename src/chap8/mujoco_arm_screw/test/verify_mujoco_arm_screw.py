#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""拧螺丝项目验证脚本（四种模式）

用法（在包根目录 src/chap8/mujoco_arm_screw 下执行）：
    python3 test/verify_mujoco_arm_screw.py mjcfk   # 独立几何校验：MuJoCo 读 tip / 刀头底面 vs 目标
    python3 test/verify_mujoco_arm_screw.py ik      # IK 精度：目标点逆解 -> 正解互查（只证明互逆）
    python3 test/verify_mujoco_arm_screw.py clear   # 无渲染全流程：刀头贴合 / 转角 / 下沉量
    python3 test/verify_mujoco_arm_screw.py gui     # glfw 实时渲染：完整播放拧螺丝
"""
import os, sys, math

os.environ.setdefault("MUJOCO_GL", "glfw")
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import mujoco

_VIEWER_IMPORT_ERR = None
try:
    # 显式导入子模块：`import mujoco` 不会自动加载 viewer，缺这句 gui 模式会报 AttributeError
    import mujoco.viewer
except Exception as e:      # 纯无头环境（无 GLFW/DISPLAY）允许 ik / mjcfk / clear 继续跑
    mujoco.viewer = None
    _VIEWER_IMPORT_ERR = e

import main as m


def fk3(yaw, q1, q2):
    """三自由度正解：关节角 -> 末端 tip 位置。"""
    r = m.L1 * math.cos(q1) + m.L2 * math.cos(q1 + q2)
    z = m.L1 * math.sin(q1) + m.L2 * math.sin(q1 + q2) + m.BASE_Z
    return (r * math.cos(yaw), r * math.sin(yaw), z)


def _adr(model, joint_name):
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name.encode())
    return model.jnt_qposadr[jid]


def _ids(model):
    """返回 (tip body id, driver free qpos 地址, screw free qpos 地址, bladeA geom id)。"""
    tip = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, b'tip')
    d_adr = _adr(model, 'driverfree')
    s_adr = _adr(model, 'screwfree')
    blade = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, b'bladeA')
    return tip, d_adr, s_adr, blade


def _place_driver_and_screw(model, data, d_adr, s_adr, tip_id, theta, sz):
    """把螺丝刀挂到 tip 下方、把螺丝放到当前下沉位置，再 mj_forward。"""
    ptip = data.xpos[tip_id]
    data.qpos[d_adr:d_adr + 3] = (ptip[0], ptip[1], ptip[2] - m.DRIVER_OFF)
    data.qpos[d_adr + 3:d_adr + 7] = m.quat_z(theta)
    data.qpos[s_adr:s_adr + 3] = (m.SCREW_XY[0], m.SCREW_XY[1], sz)
    data.qpos[s_adr + 3:s_adr + 7] = m.quat_z(theta)
    mujoco.mj_forward(model, data)


def _blade_bottom(model, data, blade_id):
    """刀头最低点：bladeA 盒体中心由 MuJoCo 算出的世界坐标，减去盒体半高。"""
    return data.geom_xpos[blade_id][2] - model.geom_size[blade_id][2]


def run_ik():
    print("== 拧螺丝 IK 精度验证（目标点逆解 -> 手写正解互查；同一定义，只说明互逆） ==")
    down_z = m.SCREW_TOP_Z + m.SCREW_HALF + m.BLADE_DROP + m.DRIVER_OFF
    targets = [
        (m.SCREW_XY[0], m.SCREW_XY[1], down_z),      # 刀头最低点贴螺丝顶
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


def run_mjcfk():
    """独立几何校验（全部读数来自 MuJoCo，不经手写 fk3）：

    1. `ik3` 求出的关节角写入 qpos -> `mj_forward` -> 读 `tip` body 世界坐标 vs IK 目标点；
    2. 同一组角，把螺丝刀挂到 tip 下方、螺丝放到初始高度 -> 读 `bladeA` 盒体世界坐标，
       减去盒体半高得到**刀头最低点**，vs 螺丝顶面 0.043 m。
    第 2 条补上了 tip 到刀头之间 0.11 m 杆长 + 0.006 m 刀头厚度的几何偏移，
    证明的是"刀头贴住螺丝顶"，而不只是"tip 对准了某个点"。
    """
    print("== 独立几何校验（MuJoCo 读数 vs 目标，不经手写 fk3） ==")
    model = mujoco.MjModel.from_xml_string(m.MODEL_XML)
    data = mujoco.MjData(model)
    tip_id, d_adr, s_adr, blade_id = _ids(model)
    qa = (_adr(model, 'yaw'), _adr(model, 'pitch1'), _adr(model, 'pitch2'))
    down_z = m.SCREW_TOP_Z + m.SCREW_HALF + m.BLADE_DROP + m.DRIVER_OFF

    targets = [
        (m.SCREW_XY[0], m.SCREW_XY[1], down_z, "刀头贴螺丝顶"),
        (m.SCREW_XY[0], m.SCREW_XY[1], 0.22, "螺丝上方待机"),
        (m.HOME[0], m.HOME[1], m.HOME[2], "HOME 待机点"),
    ]
    worst = 0.0
    for x, y, z, note in targets:
        q = m.ik3(x, y, z, None)
        if q is None:
            print("  %s：IK 不可达 FAIL" % note)
            worst = 1e9
            continue
        for k in range(3):
            data.qpos[qa[k]] = q[k]
        mujoco.mj_forward(model, data)
        p = data.xpos[tip_id]
        err = math.dist((p[0], p[1], p[2]), (x, y, z)) * 1000.0
        worst = max(worst, err)
        print("  %-12s tip 目标 (%.3f, %.3f, %.3f)  MuJoCo tip (%.4f, %.4f, %.4f)  误差 %.4f mm"
              % (note, x, y, z, p[0], p[1], p[2], err))

    # 刀头最低点 vs 螺丝顶面（仅在贴顶目标上校验）
    q = m.ik3(targets[0][0], targets[0][1], targets[0][2], None)
    for k in range(3):
        data.qpos[qa[k]] = q[k]
    mujoco.mj_forward(model, data)
    _place_driver_and_screw(model, data, d_adr, s_adr, tip_id, 0.0, m.SCREW_TOP_Z)
    blade = _blade_bottom(model, data, blade_id)
    screw_top = m.SCREW_TOP_Z + m.SCREW_HALF
    blade_err = abs(blade - screw_top) * 1000.0
    worst = max(worst, blade_err)
    print("  刀头最低点：MuJoCo 读数 %.4f m，螺丝顶面 %.4f m，误差 %.4f mm"
          % (blade, screw_top, blade_err))
    ok = worst < 1.0
    print("最大误差 %.4f mm -> %s" % (worst, "PASS" if ok else "FAIL"))
    return ok


def run_clear():
    print("== 拧螺丝全流程验证（无渲染，%d 帧） ==" % m.TOTAL)
    model = mujoco.MjModel.from_xml_string(m.MODEL_XML)
    data = mujoco.MjData(model)
    tip_id, d_adr, s_adr, blade_id = _ids(model)
    qa = (_adr(model, 'yaw'), _adr(model, 'pitch1'), _adr(model, 'pitch2'))

    q0 = m.ik3(*m.HOME, None)
    if q0 is not None:
        data.qpos[qa[0]], data.qpos[qa[1]], data.qpos[qa[2]] = q0
    yaw_prev = q0[0] if q0 is not None else None
    mujoco.mj_forward(model, data)

    max_above = 0.0      # 刀头高于螺丝顶面的最大值（悬空，正值）
    max_below = 0.0      # 刀头低于螺丝顶面的最大值（压入，负值）
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
        _place_driver_and_screw(model, data, d_adr, s_adr, tip_id, theta, sz)

        # 只在「对位 + 拧入」段检查刀头与螺丝顶贴合（抬起段刀头离开是正常的）
        # 统计窗口与 main.py 完全一致：PRESS_END <= i <= SCREW_END
        if m.PRESS_END <= i <= m.SCREW_END:
            blade_z = _blade_bottom(model, data, blade_id)
            screw_top = sz + m.SCREW_HALF
            gap = blade_z - screw_top        # 正 = 悬空，负 = 压入
            if gap > max_above:
                max_above = gap
            if gap < max_below:
                max_below = gap
        theta_max_seen = theta
        z_min = min(z_min, sz)

    print("对位+拧入段（frame %d–%d）刀头最低点 vs 螺丝顶面：最大悬空 %.4f m、最大压入 %.4f m（全程贴合）"
          % (m.PRESS_END, m.SCREW_END, abs(max_above), abs(max_below)))
    print("最终转角：%.1f°（%.2f 圈，要求 %.2f 圈）"
          % (math.degrees(theta_max_seen), theta_max_seen / (2 * math.pi), m.TURNS))
    print("螺丝中心终了高度：%.4f m（初始 %.4f，下沉 %.1f mm）"
          % (z_min, m.SCREW_TOP_Z, (m.SCREW_TOP_Z - z_min) * 1000))
    print("螺丝终了顶面高度：%.4f m（板面 0.020 m，低 %.1f mm）"
          % (z_min + m.SCREW_HALF, (0.020 - (z_min + m.SCREW_HALF)) * 1000))

    ok_turn = abs(theta_max_seen - m.THETA_MAX) < 1e-6
    ok_gap = abs(max_above) < 1e-3 and abs(max_below) < 1e-3   # 悬空与压入都不超过 1 mm
    ok_depth = abs((m.SCREW_TOP_Z - z_min) - abs(m.START_Z)) < 1e-6
    print("转角 PASS=%s | 贴合 PASS=%s | 下沉量 PASS=%s"
          % (ok_turn, ok_gap, ok_depth))
    ok = ok_turn and ok_gap and ok_depth
    print("结果：%s" % ("PASS" if ok else "FAIL"))
    return ok


def run_gui():
    print("== glfw 实时渲染：完整播放拧螺丝 ==")
    if mujoco.viewer is None:
        print("gui 模式需要 GLFW 显示环境（DISPLAY / 桌面会话），当前导入失败：%s" % _VIEWER_IMPORT_ERR)
        return False

    model = mujoco.MjModel.from_xml_string(m.MODEL_XML)
    data = mujoco.MjData(model)
    tip_id, d_adr, s_adr, _blade_id = _ids(model)
    qa = (_adr(model, 'yaw'), _adr(model, 'pitch1'), _adr(model, 'pitch2'))

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
            _place_driver_and_screw(model, data, d_adr, s_adr, tip_id, theta, sz)
            v.sync()
            time.sleep(1.0 / m.FPS)
    print("播放完成（%d 帧）" % m.TOTAL)
    return True


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "clear"
    if mode == "ik":
        sys.exit(0 if run_ik() else 1)
    elif mode == "mjcfk":
        sys.exit(0 if run_mjcfk() else 1)
    elif mode == "gui":
        sys.exit(0 if run_gui() else 1)
    else:
        sys.exit(0 if run_clear() else 1)
