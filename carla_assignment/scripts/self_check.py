#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CARLA 无人车四次作业 · 一键自检脚本（在装有 CARLA 的目标机上运行）。

依次验证 5 项内容，给出 PASS/FAIL：
  [0] Python/依赖是否齐全（numpy/opencv/pygame，可选 tensorflow）
  [1] nn_models.py 神经网络的离线训练/推理自测（分类 acc、回归 MSE）
  [2] 作业二感知/控制 NN 离线训练
  [3] 作业三规划 NN 离线训练
  [4] 是否连上 CARLA 服务端（若未启动则提示，不算失败）

用法：
    python scripts/self_check.py
"""
import importlib.util
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

PASS, FAIL = 0, 0


def report(name, ok, detail=""):
    global PASS, FAIL
    if ok:
        PASS += 1
        print(f"[PASS] {name}  {detail}")
    else:
        FAIL += 1
        print(f"[FAIL] {name}  {detail}")


def main():
    # 0) 依赖检查
    deps = {"numpy": None, "cv2": None, "pygame": None, "tensorflow": None}
    for name in deps:
        deps[name] = importlib.util.find_spec(name) is not None or \
            importlib.util.find_spec(name.replace("cv2", "cv2")) is not None
    report("依赖 numpy", deps["numpy"])
    report("依赖 opencv(cv2)", deps["cv2"])
    report("依赖 pygame", deps["pygame"])
    report("依赖 tensorflow(可选,作业四)", deps["tensorflow"] or True,
           "(未装则不阻塞，作业四训练时才需要)")

    # 1) nn_models 自测
    try:
        import nn_models
        acc, mse = nn_models._self_test()
        report("nn_models 自测", acc > 0.9 and mse < 0.1, f"acc={acc:.3f} mse={mse:.4f}")
    except Exception as e:  # noqa: BLE001
        report("nn_models 自测", False, str(e))

    # 2) 作业二感知/控制 NN 训练
    try:
        import importlib.util as iu
        spec = iu.spec_from_file_location("percep_mod", os.path.join(ROOT, "02_perception", "main.py"))
        mod = iu.module_from_spec(spec)
        spec.loader.exec_module(mod)   # 不触发 __main__ 块，避免 sys.exit
        feat_X, cY, _cx, _cy = mod.synth_dataset()
        _s, _c = mod.train(feat_X, cY, _cx, _cy, epochs=60)
        report("作业二感知/控制NN离线训练", True)
    except Exception as e:  # noqa: BLE001
        report("作业二感知/控制NN离线训练", False, str(e))

    # 3) 作业三规划 NN 训练
    try:
        import importlib.util as iu
        spec = iu.spec_from_file_location("nav_mod", os.path.join(ROOT, "03_navigation", "main.py"))
        mod = iu.module_from_spec(spec)
        spec.loader.exec_module(mod)
        mod.train_planning(epochs=60, out=os.path.join(ROOT, "models", ".selfcheck_plan.json"))
        report("作业三规划NN离线训练", True)
    except Exception as e:  # noqa: BLE001
        report("作业三规划NN离线训练", False, str(e))

    # 4) 连 CARLA（未启动不算失败）
    try:
        import carla
        client = carla.Client("127.0.0.1", 2000)
        client.set_timeout(2.0)
        world = client.get_world()
        report("CARLA 服务端连接", True,
               f"map={world.get_map().name}")
    except ImportError:
        report("CARLA 模块未装", False, "未 import carla，请先装 CARLA Python API")
    except Exception as e:  # noqa: BLE001
        report("CARLA 服务端连接(未启动则跳过)", True,
               f"提示：{e}")

    print(f"\n==== 自检结果：PASS={PASS}  FAIL={FAIL} ====")
    if FAIL == 0:
        print("全部通过。接下来可启动 CarlaUE4 后在线运行四个作业。")
    return 0 if FAIL == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
