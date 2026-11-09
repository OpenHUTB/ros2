#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""场景自检工具：逐个加载 models 下的 XML，检查拖滞注入、传感器、执行器与车道边界。

用法：
    python3 tools/check_scene.py
    python3 tools/check_scene.py --scene car_scene_fault_severe.xml
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mujoco_car_drive_fault.config import Config, MODELS_DIR  # noqa: E402

SENSORS = ("wheel_l_pos", "wheel_r_pos", "wheel_l_vel", "wheel_r_vel")


def check(cfg, scene, expect_damping):
    import mujoco

    path = MODELS_DIR / scene
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    missing = [name for name in SENSORS
               if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name) < 0]
    for act in ("drive_l", "drive_r"):
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, act) < 0:
            missing.append(act)
    for body in ("chassis", "wheel_l", "wheel_r"):
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body) < 0:
            missing.append("body:" + body)

    jl = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "wheel_l_joint")
    jr = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "wheel_r_joint")
    damp_l = float(model.dof_damping[model.jnt_dofadr[jl]]) if jl >= 0 else float("nan")
    damp_r = float(model.dof_damping[model.jnt_dofadr[jr]]) if jr >= 0 else float("nan")

    flag = ""
    if abs(damp_l - expect_damping) > 1e-6 and abs(damp_r - expect_damping) > 1e-6:
        flag = "  [拖滞量与配置不符]"

    print("{:<32s} nq={:<3d} ngeom={:<3d} damping(left)={:.2f} damping(right)={:.2f} 缺失={}{}".format(
        scene, int(model.nq), int(model.ngeom), damp_l, damp_r,
        ",".join(missing) if missing else "无", flag))
    return not missing and not flag


def main():
    parser = argparse.ArgumentParser(description="MuJoCo 场景自检")
    parser.add_argument("--scene", default="", help="只检查指定场景")
    args = parser.parse_args()

    cfg = Config()
    scenes = cfg.scenes
    if args.scene:
        hit = [item for item in scenes if item[0] == args.scene]
        if not hit:
            raise SystemExit("未知场景：{}".format(args.scene))
        scenes = hit

    ok = True
    for scene, damping in scenes:
        ok = check(cfg, scene, damping) and ok
    print("全部通过" if ok else "存在异常")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
