"""
generate_gif.py —— 生成演示 gif（offscreen 渲染）
"""
from __future__ import annotations
import os, sys, argparse
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import mujoco
from humanoid_vision_fall_prediction.sim import VisionFallSim
from humanoid_vision_fall_prediction.predictor import FallPredictor
from humanoid_vision_fall_prediction.controller import BalanceController

MODEL = os.path.join(HERE, "models", "humanoid_camera.xml")
WEIGHTS = os.path.join(HERE, "models_trained", "fall_predictor.npz")


def generate(out_gif: str, force: float = 120.0, direction_deg: float = 0.0):
    sim = VisionFallSim(MODEL)
    predictor = FallPredictor(WEIGHTS)
    ctrl = BalanceController()

    renderer = mujoco.Renderer(sim.model, height=360, width=480)
    cam_id = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_CAMERA, "overview")

    sim.reset()
    for _ in range(200):
        sim.step()

    frames = []
    rad = np.radians(direction_deg)
    sim.apply_push((force*np.cos(rad), force*np.sin(rad), 0), duration_steps=40)

    for i in range(1200):
        gray = sim.render_front(gray=True)
        label, cls, conf = predictor.predict_from_image(gray)
        target = ctrl.target_for(cls, conf)
        sim.step(target=target)

        if i % 4 == 0:
            renderer.update_scene(sim.data, camera=cam_id)
            frames.append(renderer.render())

        if sim.fallen():
            for _ in range(10):
                renderer.update_scene(sim.data, camera=cam_id)
                frames.append(renderer.render())
            break

    import imageio.v2 as imageio
    os.makedirs(os.path.dirname(out_gif) or ".", exist_ok=True)
    imageio.mimsave(out_gif, frames, fps=30)
    print(f"[gif] saved: {out_gif} ({len(frames)} frames)")
    sim.close()


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--out", default=os.path.join(HERE, "demo.gif"))
    p.add_argument("--force", type=float, default=120)
    p.add_argument("--direction", type=float, default=0)
    a = p.parse_args()
    generate(a.out, a.force, a.direction)
