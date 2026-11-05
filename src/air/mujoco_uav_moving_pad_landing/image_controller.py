"""End-to-end RGB image to velocity controller for the MuJoCo landing scene."""

from __future__ import annotations

import math
import os
from pathlib import Path

import joblib
import numpy as np
from PIL import Image
from sklearn.metrics import mean_squared_error
from sklearn.model_selection import GroupShuffleSplit
from sklearn.neural_network import MLPRegressor
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler

from .landing import DT, LandingSimulation, expert_command, vision_features


MODULE_DIR = Path(__file__).resolve().parent
IMAGE_MODEL_PATH = Path(
    os.environ.get("MUJOCO_LANDING_IMAGE_MODEL", MODULE_DIR / "output" / "image_controller.joblib")
).expanduser()
IMAGE_SIZE = 32


def image_input(image_rgb: np.ndarray, size: int = IMAGE_SIZE) -> np.ndarray:
    """Resize a rendered RGB frame and normalize pixels for direct MLP input."""
    if image_rgb.ndim != 3 or image_rgb.shape[2] != 3:
        raise ValueError("expected an RGB image with three channels")
    resized = Image.fromarray(image_rgb).resize((size, size), Image.Resampling.BILINEAR)
    return np.asarray(resized, dtype=np.float32).reshape(-1) / 255.0


def collect_rendered_samples(
    samples: int = 600,
    seed: int = 7,
    steps_per_episode: int = 20,
    image_size: int = IMAGE_SIZE,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Collect real camera frames and expert velocity labels by episode."""
    if samples < 200:
        raise ValueError("samples must be at least 200 to provide episode-level holdout")
    rng = np.random.default_rng(seed)
    episode_count = math.ceil(samples / steps_per_episode)
    frames: list[np.ndarray] = []
    commands: list[np.ndarray] = []
    groups: list[int] = []
    sim = LandingSimulation(width=160, height=160, static_pad=True)
    try:
        for episode in range(episode_count):
            initial_height = (
                float(rng.uniform(0.22, 0.36))
                if episode % 3 == 0
                else float(rng.uniform(0.4, 1.6))
            )
            sim.reset(
                x=float(rng.uniform(-0.5, 0.7)),
                y=float(rng.uniform(-0.5, 0.5)),
                z=initial_height,
            )
            episode_samples = min(steps_per_episode, samples - len(frames))
            phase_offset = float(rng.uniform(0.0, 4.0))
            for step in range(episode_samples):
                time_s = phase_offset + step * DT
                frame, _pad_velocity = sim.render(time_s)
                # The end-to-end model sees one image, so its label must not depend on hidden velocity.
                features = vision_features(frame, float(sim.position[2]), (0.0, 0.0))
                command = expert_command(features)
                command[:2] = np.clip(command[:2] * 3.0, -0.55, 0.55)
                command[2] = np.clip(command[2] * 5.0, -0.3, 0.0)
                frames.append(image_input(frame, image_size))
                commands.append(command)
                groups.append(episode)
                sim.step(command, time_s)
    finally:
        sim.close()

    return (
        np.stack(frames),
        np.stack(commands),
        np.asarray(groups, dtype=np.int32),
    )


def train_image_controller(
    path: Path = IMAGE_MODEL_PATH,
    samples: int = 600,
    seed: int = 7,
) -> dict[str, float | int]:
    """Train RGB-to-velocity MLP and evaluate on held-out complete episodes."""
    x, y, groups = collect_rendered_samples(samples=samples, seed=seed)
    splitter = GroupShuffleSplit(n_splits=1, test_size=0.2, random_state=seed)
    train_rows, test_rows = next(splitter.split(x, y, groups=groups))
    train_groups = np.unique(groups[train_rows])
    test_groups = np.unique(groups[test_rows])
    if np.intersect1d(train_groups, test_groups).size:
        raise RuntimeError("episode leakage detected between train and test sets")

    model = make_pipeline(
        StandardScaler(),
        MLPRegressor(
            hidden_layer_sizes=(96, 48),
            activation="tanh",
            solver="adam",
            alpha=0.001,
            learning_rate_init=0.002,
            max_iter=350,
            random_state=seed,
            early_stopping=True,
            n_iter_no_change=20,
        ),
    )
    model.fit(x[train_rows], y[train_rows])
    predicted = model.predict(x[test_rows])
    rmse = float(np.sqrt(mean_squared_error(y[test_rows], predicted)))
    path.parent.mkdir(parents=True, exist_ok=True)
    joblib.dump(model, path)
    return {
        "input": f"{IMAGE_SIZE}x{IMAGE_SIZE} RGB camera image",
        "train_samples": int(train_rows.size),
        "test_samples": int(test_rows.size),
        "train_episodes": int(train_groups.size),
        "test_episodes": int(test_groups.size),
        "test_rmse_m_s": rmse,
        "model_path": str(path),
    }


def load_image_controller(path: Path = IMAGE_MODEL_PATH):
    return joblib.load(path) if path.exists() else None


def predict_image_command(image_rgb: np.ndarray, model) -> np.ndarray:
    """Predict bounded body-frame velocity directly from an RGB camera frame."""
    if model is None:
        raise ValueError("an image controller model is required")
    pixels = image_input(image_rgb)
    command = np.asarray(model.predict(pixels.reshape(1, -1))[0], dtype=np.float32)
    command[0:2] = np.clip(command[0:2], -0.55, 0.55)
    command[2] = np.clip(command[2], -0.3, 0.0)
    return command
