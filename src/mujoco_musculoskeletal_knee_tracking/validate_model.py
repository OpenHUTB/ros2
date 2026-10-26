# -*- coding: utf-8 -*-
"""
本地验证脚本 (local validation, 不依赖 ROS2)

用法:
    py -3.14 validate_model.py

验证项:
  1. 模型能否被 MuJoCo 加载 (结构/元素数量)
  2. 单独激活屈肌(flexor) -> 膝关节角增大 (屈曲)
  3. 单独激活伸肌(extensor) -> 膝关节角减小 (伸展)
  4. 无控制时关节角保持在 0 deg (竖直下垂平衡)
  5. 输出一张模型渲染图 model_render.png (便于查看几何)
"""
import math
import os

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL = os.path.join(HERE, "mujoco_musculoskeletal_knee_tracking",
                     "model", "muscle_knee.xml")


def load_model():
    # 用 Python 读取(utf-8), 避免 MuJoCo C 层读取中文/特殊路径失败
    with open(MODEL, "r", encoding="utf-8") as f:
        xml = f.read()
    return mujoco.MjModel.from_xml_string(xml)


def step_ctrl(model, data, ctrl, steps, qpos0=None):
    """给定激活向量 ctrl, 从 (可选) 初始 qpos 开始跑 steps 步, 返回最终关节角(deg)."""
    data.ctrl[:] = ctrl
    data.act[:] = 0.0          # 重置肌肉激活, 避免跨测试残留
    if qpos0 is not None:
        data.qpos[:] = qpos0
    mujoco.mj_forward(model, data)
    for _ in range(steps):
        mujoco.mj_step(model, data)
    return math.degrees(data.qpos[0])


def main():
    model = load_model()
    data = mujoco.MjData(model)
    n_act = model.nu
    print(f"[OK] 模型加载成功: {model.nq} 个广义坐标, {model.nu} 个执行器")
    print(f"     执行器: {[model.actuator(i).name for i in range(n_act)]}")
    print(f"     关节  : {[model.joint(i).name for i in range(model.njnt)]}")
    assert model.nq == 1, "期望单自由度模型"
    assert n_act == 2, "期望 2 个肌肉执行器"

    idx = {model.actuator(i).name: i for i in range(n_act)}
    i_flex, i_ext = idx["flexor"], idx["extensor"]

    # --- 1. 屈肌: 从伸展位(0deg)激活, 应屈曲 (角度增大) ---
    a_flex = step_ctrl(model, data, np.array([1.0, 0.0]) if i_flex == 0 else np.array([0.0, 1.0]),
                       400, qpos0=np.zeros(model.nq))
    print(f"[屈肌] 单独激活屈肌 2s -> 关节角 = {a_flex:+.2f} deg")
    assert a_flex > 1.0, "屈肌激活后关节角应增大(屈曲)"

    # --- 2. 伸肌: 从屈曲位(~30deg)激活, 应伸展 (角度减小) ---
    q0 = np.array([math.radians(30.0)])
    ctrl_ext = np.array([1.0, 0.0]) if i_ext == 0 else np.array([0.0, 1.0])
    a_ext = step_ctrl(model, data, ctrl_ext, 400, qpos0=q0)
    print(f"[伸肌] 从 30deg 激活伸肌 2s -> 关节角 = {a_ext:+.2f} deg")
    assert a_ext < 28.0, "伸肌激活后关节角应减小(伸展)"

    # --- 3. 无控制: 应停在 0deg 附近 (竖直下垂平衡) ---
    a_free = step_ctrl(model, data, np.zeros(n_act), 400, qpos0=np.zeros(model.nq))
    print(f"[自由] 无控制 2s -> 关节角 = {a_free:+.2f} deg (应≈0)")
    assert abs(a_free) < 3.0, "无控制时应停在伸展位附近"

    # --- 4. 渲染一张图, 便于查看几何 ---
    data.ctrl[:] = 0.0
    data.qpos[:] = math.radians(20.0)
    mujoco.mj_forward(model, data)
    renderer = mujoco.Renderer(model, 480, 640)  # (height, width)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.azimuth = 90.0
    cam.elevation = -10.0
    cam.distance = 1.0
    cam.lookat = np.array([0.0, 0.0, -0.30])
    renderer.update_scene(data, camera=cam)
    img = renderer.render()
    if img.max() < 10:
        print("[WARN] 渲染结果近乎全黑, 请检查相机视角/几何位置")
    out = os.path.join(HERE, "model_render.png")
    import PIL.Image
    PIL.Image.fromarray(img).save(out)
    print(f"[OK] 已输出模型渲染图: {out}")

    print("\n全部验证通过 ✓")


if __name__ == "__main__":
    main()
