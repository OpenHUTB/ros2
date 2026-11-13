"""MuJoCo forklift simulation with camera-guided pallet docking."""

from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np

from .policy import ImagePolicy, teacher_action

MODEL_PATH = Path(__file__).with_name("model.xml")
PALLET_FACE_X = 1.5
FORK_TIP = 0.72


class ForkliftDockingSimulation:
    def __init__(self, start_x: float = -4.0, speed: float = 2.0, width: int = 64, height: int = 64, brake_scale: float = 1.0) -> None:
        if not np.isfinite(brake_scale) or brake_scale <= 0.0:
            raise ValueError("brake_scale must be finite and positive")
        self.brake_scale = brake_scale
        self.model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
        self.data = mujoco.MjData(self.model)
        self.data.qpos[0] = start_x
        self.data.qvel[0] = speed
        mujoco.mj_forward(self.model, self.data)
        self.camera_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "front_camera")
        self.renderer = mujoco.Renderer(self.model, height=height, width=width)
        self.steps = 0

    @property
    def x(self) -> float:
        return float(self.data.qpos[0])

    @property
    def speed(self) -> float:
        return float(self.data.qvel[0])

    @property
    def distance(self) -> float:
        return PALLET_FACE_X - (self.x + FORK_TIP)

    def image(self) -> np.ndarray:
        self.renderer.update_scene(self.data, camera=self.camera_id)
        rgb = self.renderer.render()
        gray = np.rint(rgb[:, :, :3].mean(axis=2)).astype(np.uint8)
        return gray

    def step(self, action: np.ndarray) -> None:
        throttle, brake = np.clip(np.asarray(action, dtype=float), 0.0, 1.0)
        self.data.ctrl[0] = 5.0 * throttle - 10.0 * brake
        mujoco.mj_step(self.model, self.data)
        self.steps += 1

    def run(self, policy: ImagePolicy, max_steps: int = 450) -> dict[str, float | int | bool]:
        over_inserted = False
        for _ in range(max_steps):
            if self.distance < -0.5:
                over_inserted = True
                break
            action = policy.predict(self.image())[0].copy()
            action[1] *= self.brake_scale
            self.step(action)
            if self.speed <= 0.025:
                break
        return {
            "steps": self.steps,
            "insertion_depth_m": round(max(0.0, -self.distance), 4),
            "over_inserted": over_inserted,
            "final_speed_mps": round(self.speed, 4),
            "pallet_clearance_m": round(self.distance, 4),
        }

    def manual_step(self, key: str) -> bool:
        """Apply one keyboard command; return False when the user requests exit."""
        key = key.lower()
        if key == "q":
            return False
        actions = {"w": (1.0, 0.0), "s": (0.0, 1.0), " ": (0.0, 0.0)}
        if key not in actions:
            return True
        action = actions[key]
        if key == "s" and self.speed <= 0.0:
            action = (0.0, 0.0)
        speed_before = self.speed
        x_before = self.x
        self.step(np.asarray(action))
        if key == "s" and speed_before > 0.0 and self.speed < 0.0:
            self.data.qpos[0] = x_before + speed_before * self.model.opt.timestep * 0.5
            self.data.qvel[0] = 0.0
            self.data.ctrl[0] = 0.0
            mujoco.mj_forward(self.model, self.data)
        return True

    def close(self) -> None:
        self.renderer.close()


def collect_training_data(samples: int = 500, seed: int = 17) -> tuple[np.ndarray, np.ndarray]:
    rng = np.random.default_rng(seed)
    images, actions = [], []
    sim = ForkliftDockingSimulation()
    try:
        for _ in range(samples):
            sim.data.qpos[0] = rng.uniform(-5.5, 0.75) if _ % 2 else rng.uniform(-0.5, 0.75)
            sim.data.qvel[0] = rng.uniform(0.4, 3.2)
            mujoco.mj_forward(sim.model, sim.data)
            images.append(sim.image())
            actions.append(teacher_action(sim.distance, sim.speed))
    finally:
        sim.close()
    return np.stack(images), np.stack(actions)


def train_policy(samples: int = 500, seed: int = 17) -> tuple[ImagePolicy, float]:
    images, actions = collect_training_data(samples, seed)
    model = ImagePolicy(seed=seed)
    split = int(samples * 0.8)
    model.train(images[:split], actions[:split], epochs=900, lr=0.18)
    mse = float(np.mean((model.predict(images[split:]) - actions[split:]) ** 2))
    return model, mse
