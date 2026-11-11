#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""2R 机械臂画正方形验证脚本（三种模式）。

用法（在包根目录 src/chap8/two_dof_arm_square/ 下执行）：
    python3 test/verify_two_dof_arm_square.py ik      # IK 互查：四个顶点逆解->正解误差
    python3 test/verify_two_dof_arm_square.py clear   # 全流程无头：跑一圈并打印各顶点偏差
    python3 test/verify_two_dof_arm_square.py gui     # glfw 实时渲染完整播放
"""
import os
import sys
import math

os.environ.setdefault("MUJOCO_GL", "glfw")
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import mujoco

_VIEWER_IMPORT_ERR = None
try:
    import mujoco.viewer
except Exception as e:
    mujoco.viewer = None
    _VIEWER_IMPORT_ERR = e

import main as m


def run_ik():
    print("== IK 互查：四个顶点解析逆解 -> MuJoCo 正解 ==")
    model = m.load_model()
    data = mujoco.MjData(model)
    end_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "end_effector")
    worst = 0.0
    for (tx, ty) in m.TARGET_CORNERS:
        q1, q2 = m.ik(tx, ty)
        data.qpos[0], data.qpos[1] = q1, q2
        mujoco.mj_forward(model, data)
        ex, ey = data.xpos[end_id][0], data.xpos[end_id][1]
        err = math.hypot(ex - tx, ey - ty) * 1000.0
        worst = max(worst, err)
        print(f"  目标 ({tx:+.3f},{ty:+.3f}) m -> q1={math.degrees(q1):+7.2f}° q2={math.degrees(q2):+7.2f}° "
              f"| 实测 ({ex:+.4f},{ey:+.4f}) m | 偏差 {err:.3f} mm")
    ok = worst < 0.5
    print(f"最大误差 {worst:.3f} mm -> {'PASS' if ok else 'FAIL'}")
    return ok


def run_clear():
    print("== 全流程无头验证：跑一圈正方形轨迹 ==")
    model = m.load_model()
    data = mujoco.MjData(model)
    end_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "end_effector")
    max_err, corner_errs = m.run_lap(model, data, end_id, viewer=None, step_sleep=0.0)
    for (tx, ty), ex, ey, e in corner_errs:
        print(f"  顶点 ({tx:+.3f},{ty:+.3f}) | 实测 ({ex:+.4f},{ey:+.4f}) | 偏差 {e*1000:.3f} mm")
    print(f"一圈最大位置误差: {max_err*1000:.3f} mm")
    ok = max_err < 2.0  # mm
    print(f"结果: {'PASS' if ok else 'FAIL'}（阈值 2 mm）")
    return ok


def run_gui():
    print("== glfw 实时渲染：循环播放画正方形 ==")
    if mujoco.viewer is None:
        print(f"gui 模式需要 GLFW 显示环境，当前导入失败：{_VIEWER_IMPORT_ERR}")
        return False
    model = m.load_model()
    data = mujoco.MjData(model)
    end_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "end_effector")
    with mujoco.viewer.launch_passive(model, data) as v:
        print("viewer 已打开，关闭窗口退出。")
        while v.is_running():
            m.run_lap(model, data, end_id, viewer=v)
    return True


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "clear"
    if mode == "ik":
        sys.exit(0 if run_ik() else 1)
    elif mode == "clear":
        sys.exit(0 if run_clear() else 1)
    elif mode == "gui":
        sys.exit(0 if run_gui() else 1)
    else:
        print(f"未知模式: {mode}（可选 ik / clear / gui）")
        sys.exit(2)
