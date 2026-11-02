#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""场景自检工具：逐个加载 models 下的 XML，检查传感器、执行器与关键几何。

用法：
    python3 tools/check_scene.py
    python3 tools/check_scene.py --scene car_scene_payload_high.xml
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mujoco_car_rollover_guard.config import Config, MODELS_DIR  # noqa: E402

SENSORS = ("imu_gyro", "imu_acc", "imu_linvel")


def check(cfg, scene):
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
    chassis = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
    payload = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "payload")
    if chassis < 0 or payload < 0:
        missing.append("body:chassis/payload")

    n_geom = int(model.ngeom)
    print("{:<32s} nq={:<3d} ngeom={:<3d} payload_z={:.2f} m 缺失={}".format(
        scene, int(model.nq), n_geom,
        float(model.body_pos[payload][2]) if payload >= 0 else -1.0,
        missing if missing else "无"))
    return not missing


def main():
    parser = argparse.ArgumentParser(description="检查三个载荷场景的 MuJoCo 结构")
    parser.add_argument("--scene", default=None, help="只检查指定场景")
    args = parser.parse_args()
    cfg = Config()
    scenes = [args.scene] if args.scene else [s[0] for s in cfg.scenes]
    ok = True
    for scene in scenes:
        ok = check(cfg, scene) and ok
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
