# -*- coding: utf-8 -*-
"""本地验证脚本（不依赖 ROS 2）。

用法:
    python3 validate_model.py                # 含渲染检查
    python3 validate_model.py --no-render    # 无图形环境：跳过渲染

验证项:
  1. 模型能被 MuJoCo 加载（结构/元素数量）
  2. 单独激活屈肌 -> 膝关节角增大（屈曲）
  3. 单独激活伸肌 -> 膝关节角减小（伸展）
  4. 无控制时关节角保持在 0 deg 附近
  5. （可选）输出模型渲染图 model_render.png
"""
import argparse
import math
import os

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL = os.path.join(HERE, "mujoco_musculoskeletal_knee_tracking",
                     "model", "muscle_knee.xml")


def load_model():
    # 用 Python 读取 (utf-8)，避免 MuJoCo C 层处理非 ASCII 路径失败
    with open(MODEL, "r", encoding="utf-8") as f:
        xml = f.read()
    return mujoco.MjModel.from_xml_string(xml)


def step_ctrl(model, data, ctrl, steps, qpos0=None):
    data.ctrl[:] = ctrl
    data.act[:] = 0.0
    if qpos0 is not None:
        data.qpos[:] = qpos0
    mujoco.mj_forward(model, data)
    for _ in range(steps):
        mujoco.mj_step(model, data)
    return math.degrees(data.qpos[0])


def render_preview(model, data):
    """渲染一张图；无图形环境时降级为提示，不影响断言结果。"""
    try:
        renderer = mujoco.Renderer(model, 480, 640)
    except Exception as exc:
        print(f"[WARN] 渲染不可用（{exc}）")
        print("[WARN] 已跳过 model_render.png，其余验证结果有效。"
              "如需出图请在带图形界面的环境运行，或设置 MUJOCO_GL=egl。")
        return

    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.azimuth = 90.0
    cam.elevation = -10.0
    cam.distance = 1.0
    cam.lookat = np.array([0.0, 0.0, -0.30])

    data.ctrl[:] = 0.0
    data.qpos[:] = math.radians(20.0)
    mujoco.mj_forward(model, data)
    renderer.update_scene(data, camera=cam)
    img = renderer.render()
    if img.max() < 10:
        print("[WARN] 渲染结果近乎全黑，请检查相机视角 / 几何位置")

    import PIL.Image
    out = os.path.join(HERE, "model_render.png")
    PIL.Image.fromarray(img).save(out)
    print(f"[OK] 已输出模型渲染图: {out}")


def main():
    ap = argparse.ArgumentParser(description="膝关节模型本地验证")
    ap.add_argument("--no-render", action="store_true", help="跳过渲染（无图形环境）")
    args = ap.parse_args()

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

    ctrl_flex = np.zeros(n_act)
    ctrl_flex[i_flex] = 1.0
    a_flex = step_ctrl(model, data, ctrl_flex, 400, qpos0=np.zeros(model.nq))
    print(f"[屈肌] 单独激活屈肌 2s -> 关节角 = {a_flex:+.2f} deg")
    assert a_flex > 1.0, "屈肌激活后关节角应增大（屈曲）"

    ctrl_ext = np.zeros(n_act)
    ctrl_ext[i_ext] = 1.0
    q0 = np.array([math.radians(30.0)])
    a_ext = step_ctrl(model, data, ctrl_ext, 400, qpos0=q0)
    print(f"[伸肌] 从 30deg 激活伸肌 2s -> 关节角 = {a_ext:+.2f} deg")
    assert a_ext < 28.0, "伸肌激活后关节角应减小（伸展）"

    a_free = step_ctrl(model, data, np.zeros(n_act), 400, qpos0=np.zeros(model.nq))
    print(f"[自由] 无控制 2s -> 关节角 = {a_free:+.2f} deg (应≈0)")
    assert abs(a_free) < 3.0, "无控制时应停在伸展位附近"

    if not args.no_render:
        render_preview(model, data)
    else:
        print("[SKIP] 已按 --no-render 跳过渲染")

    print("\n全部验证通过")


if __name__ == "__main__":
    main()
