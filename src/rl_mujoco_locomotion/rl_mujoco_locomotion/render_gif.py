"""Render a trained policy to an animated GIF."""

import os
import numpy as np
import imageio.v2 as imageio
from PIL import Image

from .evaluate import evaluate

RESULTS_DIR = os.path.join(os.path.dirname(__file__), "..", "results")
OUT_WIDTH = 360


def _resize(frames, width=OUT_WIDTH):
    resized = []
    for f in frames:
        img = Image.fromarray(f)
        ratio = width / img.width
        img = img.resize((width, int(img.height * ratio)), Image.BILINEAR)
        resized.append(np.asarray(img))
    return resized


def render_gif(env_name, alg="ppo", episodes=2, seed=42, fps=25, out_path=None,
               frame_stride=5):
    result = evaluate(env_name, alg=alg, episodes=episodes, seed=seed,
                      render=True, frame_stride=frame_stride)
    frames = result["frames"]
    if not frames:
        print(f"[Render] No frames captured for {env_name}/{alg}")
        return None
    frames = _resize(frames)
    if out_path is None:
        out_path = os.path.join(RESULTS_DIR, f"{env_name}_{alg}.gif")
    imageio.mimsave(out_path, frames, fps=fps, loop=0)
    size_mb = os.path.getsize(out_path) / 1024 / 1024
    print(f"[Render] Saved GIF to {out_path} ({size_mb:.1f} MB, {len(frames)} frames)")
    return out_path
