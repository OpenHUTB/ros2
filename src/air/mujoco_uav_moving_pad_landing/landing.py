"""Simulation, image perception, and a compact supervised landing controller."""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
from typing import Iterable

import cv2
import mujoco
import numpy as np
from PIL import Image
from .dataset import generate_grouped_samples, split_by_episode
from sklearn.metrics import mean_squared_error
from sklearn.neural_network import MLPRegressor
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler
import joblib


MODULE_DIR = Path(__file__).resolve().parent
MODEL_PATH = Path(os.environ.get("MUJOCO_LANDING_MODEL", MODULE_DIR / "output" / "landing_controller.joblib")).expanduser()
SCENE_PATH = MODULE_DIR / "assets" / "moving_pad.xml"
DT = 0.02
Z_GRAVITY_COMPENSATION_M = 0.15
CONTACT_HEIGHT_M = 0.235


def detect_colored_center(image_rgb: np.ndarray, color: str) -> tuple[float, float] | None:
    """Return a normalized image centroid for the green pad or red UAV."""
    hsv = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2HSV)
    if color == "pad":
        mask = cv2.inRange(hsv, np.array([35, 70, 35]), np.array([90, 255, 255]))
    elif color == "uav":
        red_low = cv2.inRange(hsv, np.array([0, 75, 45]), np.array([15, 255, 255]))
        red_high = cv2.inRange(hsv, np.array([165, 75, 45]), np.array([179, 255, 255]))
        mask = cv2.bitwise_or(red_low, red_high)
    else:
        raise ValueError("color must be 'pad' or 'uav'")

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = [c for c in contours if cv2.contourArea(c) >= 8]
    if not candidates:
        return None
    moments = cv2.moments(max(candidates, key=cv2.contourArea))
    if moments["m00"] <= 0:
        return None
    height, width = image_rgb.shape[:2]
    return (
        float(moments["m10"] / moments["m00"] / width - 0.5),
        float(moments["m01"] / moments["m00"] / height - 0.5),
    )


def vision_features(image_rgb: np.ndarray, altitude: float, pad_velocity: Iterable[float]) -> np.ndarray:
    """Build normalized image, altitude, platform-velocity, and validity features."""
    pad = detect_colored_center(image_rgb, "pad")
    uav = detect_colored_center(image_rgb, "uav")
    velocity = np.asarray(tuple(pad_velocity), dtype=np.float32)
    if pad is None or uav is None:
        return np.array([0.0, 0.0, np.clip((altitude - 0.22) / 1.5, 0.0, 1.0), *velocity[:2], 0.0], dtype=np.float32)
    # The overhead camera's image y-axis is opposite to the world y-axis.
    error = np.array([pad[0] - uav[0], uav[1] - pad[1]], dtype=np.float32)
    return np.array([*error, np.clip((altitude - 0.22) / 1.5, 0.0, 1.0), *velocity[:2], 1.0], dtype=np.float32)


def expert_command(features: np.ndarray) -> np.ndarray:
    """Conservative reference policy used to label the small training set."""
    if features.shape[0] >= 6 and float(features[5]) < 0.5:
        return np.zeros(3, dtype=np.float32)
    ex, ey, height, pad_vx, pad_vy = map(float, features[:5])
    vx = np.clip(2.0 * ex + 0.45 * pad_vx, -0.55, 0.55)
    vy = np.clip(2.0 * ey + 0.45 * pad_vy, -0.55, 0.55)
    centered = math.hypot(ex, ey) < 0.055
    vz = -0.24 if centered else -0.035
    if height < 0.06:
        vz = 0.0
    return np.array([vx, vy, vz], dtype=np.float32)


def controller_command(features: np.ndarray, model=None) -> np.ndarray:
    """Return a zero command when visual detection is invalid."""
    if features.shape != (6,):
        raise ValueError("expected six visual and odometry features")
    if not np.isfinite(features).all():
        return np.zeros(3, dtype=np.float32)
    if float(features[5]) < 0.5:
        return np.zeros(3, dtype=np.float32)
    if model is None:
        return expert_command(features)
    return np.asarray(model.predict(features.reshape(1, -1))[0], dtype=np.float32)


def train_controller(path: Path = MODEL_PATH, samples: int = 5000, seed: int = 7) -> dict[str, float | int]:
    """Fit and save an MLP with episode-level train/test isolation."""
    if samples < 200:
        raise ValueError("samples must be at least 200 to provide ten evaluation episodes")
    x, y, groups = generate_grouped_samples(
        expert_command, steps_per_episode=20, seed=seed, samples=samples
    )
    x_train, y_train, x_test, y_test, train_episodes, test_episodes = split_by_episode(
        x, y, groups, seed=seed
    )
    model = make_pipeline(
        StandardScaler(),
        MLPRegressor(hidden_layer_sizes=(24, 16), activation="tanh", solver="adam", alpha=0.0005,
                     learning_rate_init=0.003, max_iter=650, random_state=seed, early_stopping=True,
                     n_iter_no_change=20),
    )
    model.fit(x_train, y_train)
    predicted = model.predict(x_test)
    rmse = float(np.sqrt(mean_squared_error(y_test, predicted)))
    path.parent.mkdir(parents=True, exist_ok=True)
    joblib.dump(model, path)
    return {
        "train_samples": int(x_train.shape[0]),
        "test_samples": int(x_test.shape[0]),
        "train_episodes": train_episodes,
        "test_episodes": test_episodes,
        "test_rmse_m_s": rmse,
    }


def load_controller(path: Path = MODEL_PATH):
    return joblib.load(path) if path.exists() else None


class LandingSimulation:
    """MuJoCo plant with a sinusoidally moving mocap landing platform."""

    def __init__(self, width: int = 320, height: int = 320, static_pad: bool = False):
        self.static_pad = static_pad
        self.model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
        self.data = mujoco.MjData(self.model)
        self.renderer = mujoco.Renderer(self.model, height=height, width=width)
        self.pad_id = self.model.body("landing_pad").mocapid[0]
        self.qpos_ids = [self.model.joint(name).qposadr[0] for name in ("uav_x", "uav_y", "uav_z")]
        self.ctrl_ids = [self.model.actuator(name).id for name in ("x_servo", "y_servo", "z_servo")]
        self.reset()

    def reset(self, x: float = 0.65, y: float = 0.5, z: float = 1.5) -> None:
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[self.qpos_ids] = [x, y, z]
        self._move_pad(0.0)
        self.data.ctrl[self.ctrl_ids] = [x, y, z]
        mujoco.mj_forward(self.model, self.data)

    def _move_pad(self, time_s: float) -> np.ndarray:
        if self.static_pad:
            self.data.mocap_pos[self.pad_id] = [0.0, 0.0, 0.09]
            return np.zeros(2, dtype=float)
        position = np.array([0.34 * math.sin(0.42 * time_s), 0.22 * math.sin(0.31 * time_s), 0.09])
        velocity = np.array([0.34 * 0.42 * math.cos(0.42 * time_s), 0.22 * 0.31 * math.cos(0.31 * time_s)])
        self.data.mocap_pos[self.pad_id] = position
        return velocity

    @property
    def position(self) -> np.ndarray:
        return self.data.qpos[self.qpos_ids].copy()

    def render(self, time_s: float) -> tuple[np.ndarray, np.ndarray]:
        velocity = self._move_pad(time_s)
        mujoco.mj_forward(self.model, self.data)
        self.renderer.update_scene(self.data, camera="overhead")
        return self.renderer.render(), velocity

    def step(self, command: np.ndarray, time_s: float, compensate_gravity: bool = False) -> None:
        self._move_pad(time_s)
        command = np.asarray(command, dtype=float)
        target = self.position + command * DT * 3.0
        target[0:2] = np.clip(target[0:2], -1.3, 1.3)
        if compensate_gravity:
            target[2] = self.position[2] + command[2] * DT * 10.0
            if command[2] >= -0.01 and target[2] <= CONTACT_HEIGHT_M + 0.06:
                target[2] = max(target[2], CONTACT_HEIGHT_M)
            target[2] = np.clip(target[2] + Z_GRAVITY_COMPENSATION_M, 0.22, 2.2)
        else:
            target[2] = np.clip(target[2], 0.22, 2.0)
        self.data.ctrl[self.ctrl_ids] = target
        mujoco.mj_step(self.model, self.data)

    def close(self) -> None:
        self.renderer.close()


def run_demo(
    output_dir: Path,
    steps: int = 180,
    controller_path: Path = MODEL_PATH,
    end_to_end: bool = False,
) -> dict[str, object]:
    output_dir.mkdir(parents=True, exist_ok=True)
    if end_to_end:
        from .image_controller import IMAGE_MODEL_PATH, load_image_controller, predict_image_command

        image_model = load_image_controller(IMAGE_MODEL_PATH)
        if image_model is None:
            raise FileNotFoundError(f"image controller model not found: {IMAGE_MODEL_PATH}")
        model = image_model
    else:
        predict_image_command = None
        model = load_controller(controller_path)
    sim = LandingSimulation(static_pad=end_to_end)
    frames: list[Image.Image] = []
    trace: list[dict[str, float]] = []
    try:
        for index in range(steps):
            time_s = index * DT
            image, pad_velocity = sim.render(time_s)
            features = vision_features(image, sim.position[2], pad_velocity)
            command = (
                predict_image_command(image, model)
                if end_to_end
                else controller_command(features, model)
            )
            sim.step(command, time_s, compensate_gravity=end_to_end)
            if index % 3 == 0:
                frames.append(Image.fromarray(image))
            trace.append({"time_s": time_s, "x_m": float(sim.position[0]), "y_m": float(sim.position[1]),
                          "z_m": float(sim.position[2]), "vx_cmd_m_s": float(command[0]),
                          "vy_cmd_m_s": float(command[1]), "vz_cmd_m_s": float(command[2])})
        final_image, _ = sim.render(steps * DT)
        cv2.imwrite(str(output_dir / "landing_scene.png"), cv2.cvtColor(final_image, cv2.COLOR_RGB2BGR))
        if frames:
            frames[::2][0].save(output_dir / "landing_run.gif", save_all=True, append_images=frames[::2][1:],
                                duration=60, loop=0, optimize=True)
        (output_dir / "trajectory.json").write_text(json.dumps(trace, ensure_ascii=False, indent=2), encoding="utf-8")
        pad_offset = sim.position[:2] - sim.data.mocap_pos[sim.pad_id][:2]
        approach_error = float(np.linalg.norm(pad_offset))
        fully_inside_pad = bool(np.all(np.abs(pad_offset) <= (0.38 - 0.13)))
        contact_height = 0.09 + 0.07 + 0.075
        landing_success = fully_inside_pad and abs(float(sim.position[2]) - contact_height) <= 0.03
        result = {"steps": steps, "duration_s": round(steps * DT, 3),
                  "landing_approach_error_m": approach_error, "landing_success": landing_success,
                  "final_altitude_m": float(sim.position[2]),
                  "controller": "image_mlp" if end_to_end else ("mlp" if model else "expert")}
        (output_dir / "metrics.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
        return result
    finally:
        sim.close()
