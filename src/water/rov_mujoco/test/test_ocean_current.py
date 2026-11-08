#!/usr/bin/env python3
"""
洋流拖曳方向回归测试
====================
修复前：F = -drag_coeff * V_current，+X 洋流会把 ROV 推向 -X。
修复后：F = -drag_coeff * (V_body - V_current)，+X 洋流把静止 ROV 推向 +X。

运行（不依赖 ROS）：
    python3 -I test/test_ocean_current.py
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import mujoco  # noqa: E402
from rov_mujoco.ocean_current import apply_current_drag  # noqa: E402

MODEL_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "models", "underwater_rov_with_arm.xml",
)
DRAG_COEFF = 10.0


def _run(current, seconds=3.0):
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "rov")
    jnt_id = int(model.body_jntadr[body_id])
    qadr = int(model.jnt_qposadr[jnt_id])
    total_mass = float(np.sum(model.body_mass))

    for _ in range(int(seconds / model.opt.timestep)):
        data.xfrc_applied[body_id, :] = 0.0
        data.xfrc_applied[body_id, 2] = total_mass * 9.81  # 中性浮力
        apply_current_drag(data, body_id, np.array(current, dtype=float), DRAG_COEFF)
        mujoco.mj_step(model, data)

    return data, body_id, qadr


def run_test():
    print("=" * 60)
    print("  洋流拖曳方向回归测试")
    print("=" * 60)

    # 1. 力的符号：静止机器人 + +X 洋流 -> +X 力
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "rov")
    mujoco.mj_forward(model, data)
    data.qvel[:] = 0.0
    apply_current_drag(data, body_id, np.array([0.5, 0.0, 0.0]), DRAG_COEFF)
    fx = float(data.xfrc_applied[body_id, 0])
    print(f"[1] 静止 + +X 洋流: fx = {fx:+.3f} N")
    assert fx > 0.0, "洋流应把静止机器人推向洋流方向 (+X)！"

    # 2. 机器人自身运动 -> 阻力与运动方向相反
    data.qvel[:] = 0.0
    jnt_id = int(model.body_jntadr[body_id])
    dof_adr = int(model.jnt_dofadr[jnt_id])
    data.qvel[dof_adr] = 1.0  # +X 1 m/s
    data.xfrc_applied[body_id, :] = 0.0
    apply_current_drag(data, body_id, np.array([0.0, 0.0, 0.0]), DRAG_COEFF)
    fx = float(data.xfrc_applied[body_id, 0])
    print(f"[2] 机器人 +X 1 m/s、无洋流: fx = {fx:+.3f} N")
    assert fx < 0.0, "机器人运动时拖曳力必须与运动方向相反！"

    # 3. 闭环漂移方向
    for current, label, positive in [
        ((0.0, 0.0, 0.0), "无洋流", None),
        ((0.5, 0.0, 0.0), "+X 洋流 0.5 m/s", True),
        ((-0.5, 0.0, 0.0), "-X 洋流 0.5 m/s", False),
    ]:
        data, bid, qadr = _run(current)
        x = float(data.qpos[qadr])
        print(f"[3] {label:<20s} 3 s 后 x = {x:+.5f} m")
        if positive is True:
            assert x > 0.01, "ROV 应顺 +X 洋流漂移！"
        elif positive is False:
            assert x < -0.01, "ROV 应顺 -X 洋流漂移！"
        else:
            assert abs(x) < 1e-3, "无洋流时不应产生横向漂移！"

    print("\n✅ 洋流拖曳方向回归测试全部通过！")


if __name__ == "__main__":
    run_test()
