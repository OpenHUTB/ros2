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
from sklearn.metrics import mean_squared_error
from sklearn.neural_network import MLPRegressor
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import train_test_split
import joblib

from .dataset import generate_grouped_samples, split_by_episode

MODULE_DIR = Path(__file__).resolve().parent
MODEL_PATH = Path(os.environ.get("MUJOCO_LANDING_MODEL", MODULE_DIR / "output" / "landing_controller.joblib")).expanduser()


def _resolve_scene_path() -> Path:
    source_asset = MODULE_DIR / "assets" / "moving_pad.xml"
    if source_asset.is_file():
        return source_asset
    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError:
        return source_asset
    return Path(get_package_share_directory("mujoco_uav_moving_pad_landing")) / "assets" / "moving_pad.xml"


SCENE_PATH = _resolve_scene_path()
DT = 0.02
DEFAULT_PAD_AMPLITUDE = (0.34, 0.22)
DEFAULT_PAD_FREQUENCY = (0.42, 0.31)
DEFAULT_PAD_PHASE = (0.0, 0.0)


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
    features = np.asarray(features, dtype=np.float32)
    if features.shape != (6,):
        raise ValueError("expected six visual and odometry features")
    if not np.isfinite(features).all() or float(features[5]) < 0.5:
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


def train_controller(
    path: Path = MODEL_PATH,
    samples: int = 5000,
    seed: int = 7,
    steps_per_episode: int = 20,
) -> dict[str, float | int]:
    """Fit the expert-policy MLP on rendered MuJoCo rollouts and grouped splits."""
    if isinstance(samples, bool) or samples < 40:
        raise ValueError("samples must be at least 40 to create independent training and test episodes")
    if isinstance(steps_per_episode, bool) or steps_per_episode < 1:
        raise ValueError("steps_per_episode must be positive")
    x, y, groups = generate_grouped_samples(
        expert_command,
        samples=samples,
        steps_per_episode=steps_per_episode,
        seed=seed,
    )
    x_train, y_train, x_test, y_test, train_episodes, test_episodes = split_by_episode(
        x, y, groups, test_size=0.2, seed=seed
    )

    def new_model():
        return make_pipeline(
            StandardScaler(),
            MLPRegressor(hidden_layer_sizes=(24, 16), activation="tanh", solver="adam", alpha=0.0005,
                         learning_rate_init=0.003, max_iter=650, random_state=seed, early_stopping=True,
                         n_iter_no_change=20),
        )

    model = new_model()
    model.fit(x_train, y_train)
    predicted = model.predict(x_test)
    rmse = float(np.sqrt(mean_squared_error(y_test, predicted)))

    iid_train, iid_test = train_test_split(
        np.arange(len(x)), test_size=0.2, random_state=seed
    )
    iid_model = new_model()
    iid_model.fit(x[iid_train], y[iid_train])
    iid_rmse = float(np.sqrt(mean_squared_error(y[iid_test], iid_model.predict(x[iid_test]))))
    path.parent.mkdir(parents=True, exist_ok=True)
    joblib.dump(model, path)
    return {
        "train_samples": len(x_train),
        "test_samples": len(x_test),
        "train_episodes": train_episodes,
        "test_episodes": test_episodes,
        "steps_per_episode": steps_per_episode,
        "test_rmse_m_s": rmse,
        "iid_test_rmse_m_s": iid_rmse,
    }


def load_controller(path: Path = MODEL_PATH):
    return joblib.load(path) if path.exists() else None


class LandingSimulation:
    """MuJoCo plant with a sinusoidally moving mocap landing platform."""

    def __init__(
        self,
        width: int = 320,
        height: int = 320,
        pad_amplitude: Iterable[float] = DEFAULT_PAD_AMPLITUDE,
        pad_frequency: Iterable[float] = DEFAULT_PAD_FREQUENCY,
        pad_phase: Iterable[float] = DEFAULT_PAD_PHASE,
    ):
        self.pad_amplitude = np.asarray(tuple(pad_amplitude), dtype=float)
        self.pad_frequency = np.asarray(tuple(pad_frequency), dtype=float)
        self.pad_phase = np.asarray(tuple(pad_phase), dtype=float)
        if self.pad_amplitude.shape != (2,) or self.pad_frequency.shape != (2,) or self.pad_phase.shape != (2,):
            raise ValueError("pad trajectory parameters must each contain x and y values")
        if np.any(self.pad_amplitude < 0.0) or np.any(self.pad_frequency < 0.0):
            raise ValueError("pad amplitude and frequency must be non-negative")
        self.model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
        self.data = mujoco.MjData(self.model)
        self.renderer = mujoco.Renderer(self.model, height=height, width=width)
        self.pad_id = self.model.body("landing_pad").mocapid[0]
        self.pad_geom_id = self.model.geom("pad_surface").id
        self.uav_geom_id = self.model.geom("uav_body").id
        self.qpos_ids = [self.model.joint(name).qposadr[0] for name in ("uav_x", "uav_y", "uav_z")]
        self.qvel_ids = [self.model.joint(name).dofadr[0] for name in ("uav_x", "uav_y", "uav_z")]
        self.ctrl_ids = [self.model.actuator(name).id for name in ("x_servo", "y_servo", "z_servo")]
        self.targets = np.zeros(3, dtype=float)
        z_servo = self.model.actuator("z_servo").id
        uav_body_id = self.model.body("uav").id
        self.gravity_compensation = float(
            self.model.body_mass[uav_body_id] * abs(self.model.opt.gravity[2])
            / self.model.actuator_gainprm[z_servo, 0]
        )
        self.reset()

    def reset(self, x: float = 0.65, y: float = 0.5, z: float = 1.5) -> None:
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[self.qpos_ids] = [x, y, z]
        self._move_pad(0.0)
        self.targets = np.array([x, y, z], dtype=float)
        self.data.ctrl[self.ctrl_ids] = self.targets + np.array([0.0, 0.0, self.gravity_compensation])
        mujoco.mj_forward(self.model, self.data)

    def _move_pad(self, time_s: float) -> np.ndarray:
        angle = self.pad_frequency * time_s + self.pad_phase
        position = np.array([
            self.pad_amplitude[0] * math.sin(angle[0]),
            self.pad_amplitude[1] * math.sin(angle[1]),
            0.09,
        ])
        velocity = self.pad_amplitude * self.pad_frequency * np.cos(angle)
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

    def step(self, command: np.ndarray, time_s: float) -> None:
        self._move_pad(time_s)
        self.targets += np.asarray(command, dtype=float) * DT * 3.0
        self.targets[0:2] = np.clip(self.targets[0:2], -1.3, 1.3)
        self.targets[2] = np.clip(self.targets[2], 0.22, 2.0)
        self.data.ctrl[self.ctrl_ids] = self.targets + np.array([0.0, 0.0, self.gravity_compensation])
        mujoco.mj_step(self.model, self.data)

    def pad_contact(self) -> bool:
        """Return whether any current contact involves the landing-pad surface."""
        return any(
            self.pad_geom_id in (self.data.contact[index].geom1, self.data.contact[index].geom2)
            for index in range(self.data.ncon)
        )

    def contact_metrics(self) -> dict[str, float | int | bool]:
        """Expose contact count, vertical speed, and model-derived contact height."""
        pad_top = float(self.data.geom_xpos[self.pad_geom_id][2] + self.model.geom_size[self.pad_geom_id][2])
        uav_half_height = float(self.model.geom_size[self.uav_geom_id][2])
        return {
            "pad_contact": self.pad_contact(),
            "ncon": int(self.data.ncon),
            "vertical_speed_m_s": float(self.data.qvel[self.qvel_ids[2]]),
            "contact_height_m": pad_top + uav_half_height,
        }

    def close(self) -> None:
        self.renderer.close()


def run_demo(
    output_dir: Path,
    steps: int = 180,
    controller_path: Path = MODEL_PATH,
    pad_amplitude: Iterable[float] = DEFAULT_PAD_AMPLITUDE,
    pad_frequency: Iterable[float] = DEFAULT_PAD_FREQUENCY,
    pad_phase: Iterable[float] = DEFAULT_PAD_PHASE,
) -> dict[str, object]:
    output_dir.mkdir(parents=True, exist_ok=True)
    model = load_controller(controller_path)
    sim = LandingSimulation(pad_amplitude=pad_amplitude, pad_frequency=pad_frequency, pad_phase=pad_phase)
    frames: list[Image.Image] = []
    trace: list[dict[str, float]] = []
    first_contact: dict[str, object] | None = None
    try:
        for index in range(steps):
            time_s = index * DT
            image, pad_velocity = sim.render(time_s)
            features = vision_features(image, sim.position[2], pad_velocity)
            command = controller_command(features, model)
            sim.step(command, time_s)
            metrics = sim.contact_metrics()
            if first_contact is None and bool(metrics["pad_contact"]):
                offset = sim.position[:2] - sim.data.mocap_pos[sim.pad_id][:2]
                first_contact = {
                    "step": index + 1,
                    "time_s": round((index + 1) * DT, 3),
                    "offset_x_m": float(offset[0]),
                    "offset_y_m": float(offset[1]),
                    "offset_norm_m": float(np.linalg.norm(offset)),
                    "altitude_m": float(sim.position[2]),
                    "vertical_speed_m_s": float(metrics["vertical_speed_m_s"]),
                    "ncon": int(metrics["ncon"]),
                    "contact_height_m": float(metrics["contact_height_m"]),
                }
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
        approach_error = None if first_contact is None else float(first_contact["offset_norm_m"])
        landing_success = False
        if first_contact is not None:
            inside_pad = max(abs(float(first_contact["offset_x_m"])), abs(float(first_contact["offset_y_m"]))) <= (
                float(sim.model.geom_size[sim.pad_geom_id][0]) - float(sim.model.geom_size[sim.uav_geom_id][0])
            )
            altitude_ok = abs(float(first_contact["altitude_m"]) - float(first_contact["contact_height_m"])) <= 0.03
            speed_ok = abs(float(first_contact["vertical_speed_m_s"])) <= 0.6
            landing_success = bool(inside_pad and altitude_ok and speed_ok and int(first_contact["ncon"]) > 0)
        result = {"steps": steps, "duration_s": round(steps * DT, 3),
                  "landing_approach_error_m": approach_error, "landing_success": landing_success,
                  "final_altitude_m": float(sim.position[2]), "controller": "mlp" if model else "expert"}
        result["first_contact"] = first_contact
        result["pad_amplitude_m"] = [float(value) for value in sim.pad_amplitude]
        result["pad_angular_frequency_rad_s"] = [float(value) for value in sim.pad_frequency]
        result["pad_phase_rad"] = [float(value) for value in sim.pad_phase]
        (output_dir / "metrics.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
        return result
    finally:
        sim.close()
