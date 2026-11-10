"""
generate_gif.py —— 端到端模块演示 gif（offscreen 渲染）
================================================================
用最终端到端网络驱动，先空站再施加推力，录制 overview 视角的 gif。
默认输出到本模块目录下的 demo_test.gif（与 README、PR 描述引用的文件名一致）。
"""
from __future__ import annotations
import os, sys, argparse
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import mujoco
from humanoid_e2e_recovery.sim import E2ESim, DEFAULT_STAND_TARGET
from humanoid_e2e_recovery.predictor import E2EPredictor
from humanoid_e2e_recovery.expert import expert_action

MODEL = os.path.join(HERE, "models", "humanoid_camera.xml")
WEIGHTS = os.path.join(HERE, "models_trained", "e2e_policy.npz")
DEFAULT_OUT = os.path.join(HERE, "demo_test.gif")


def generate(out_gif: str = DEFAULT_OUT, force: float = 50.0, direction_deg: float = 0.0,
             stand_seconds: float = 2.0, after_seconds: float = 8.0):
    sim = E2ESim(MODEL)
    predictor = E2EPredictor(WEIGHTS)

    renderer = mujoco.Renderer(sim.model, height=360, width=480)
    cam_id = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_CAMERA, "overview")

    frames = []

    def snap():
        renderer.update_scene(sim.data, camera=cam_id)
        frames.append(renderer.render().copy())

    # 1) 空站 2 s
    sim.reset()
    u = np.zeros(15)
    for i in range(int(stand_seconds / 0.002)):
        if i % 2 == 0:
            u = predictor.predict_from_image(sim.render_front(gray=True))
        sim.step(target=DEFAULT_STAND_TARGET, torque_extra=u)
        if i % 6 == 0:
            snap()

    # 2) 施加推力（推扰期间由专家维持，推扰结束后交给网络）
    rad = np.radians(direction_deg)
    sim.apply_push((force * np.cos(rad), force * np.sin(rad), 0), steps=40,
                   target=DEFAULT_STAND_TARGET, balance=expert_action)

    # 3) 网络接管，录制恢复过程
    fell = None
    for i in range(int(after_seconds / 0.002)):
        if i % 2 == 0:
            u = predictor.predict_from_image(sim.render_front(gray=True))
        sim.step(target=DEFAULT_STAND_TARGET, torque_extra=u)
        if i % 6 == 0:
            snap()
        if sim.fallen():
            fell = float(sim.data.time)
            for _ in range(10):
                snap()
            break

    import imageio.v2 as imageio
    os.makedirs(os.path.dirname(os.path.abspath(out_gif)) or ".", exist_ok=True)
    imageio.mimsave(out_gif, frames, fps=30)
    print(f"[gif] saved: {out_gif} ({len(frames)} frames)"
          f"{' | 跌倒于 %.2fs' % fell if fell else ' | 全程站稳'}")
    sim.close()
    return out_gif, len(frames), fell


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--out", default=DEFAULT_OUT,
                   help="输出路径，默认 demo_test.gif（与 README 引用一致）")
    p.add_argument("--force", type=float, default=50)
    p.add_argument("--direction", type=float, default=0)
    p.add_argument("--stand", type=float, default=2.0)
    p.add_argument("--after", type=float, default=8.0)
    a = p.parse_args()
    generate(a.out, a.force, a.direction, a.stand, a.after)
