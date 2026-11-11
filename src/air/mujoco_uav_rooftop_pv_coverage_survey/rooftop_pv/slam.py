"""Planar RGB-D landmark SLAM and navigation / 平面 RGB-D 地标 SLAM 与导航。

The filter receives only relative camera measurements and commanded displacement.
Landmark positions are unknown until observed. True poses are used only to move
the simulated camera and calculate errors, never as filter observations.
滤波器仅接收相机相对观测与指令里程计，地标位置由观测初始化；真值仅用于仿真及误差统计。
"""

from __future__ import annotations

import argparse
import colorsys
import json
import math
from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image, ImageDraw

from .scene import scenario_path, set_drone_pose


class PlanarLandmarkSLAM:
    """Joint position/landmark Kalman filter in a fixed-yaw local frame.

    RGB-D gives relative Cartesian measurements z = landmark - vehicle.
    The initial vehicle position defines the otherwise unobservable map origin.
    固定航向局部坐标系联合位置/地标滤波，初始车辆位置定义地图原点。
    """

    def __init__(self, observation_std: float = 0.04) -> None:
        if observation_std <= 0:
            raise ValueError("observation_std must be positive")
        self.state = np.zeros(2)
        self.covariance = np.eye(2) * 1e-8
        self.landmarks: dict[int, int] = {}
        self.measurement_covariance = np.eye(2) * observation_std**2
        self.reobservations = 0

    @property
    def position(self) -> np.ndarray:
        return self.state[:2].copy()

    def predict(self, displacement: np.ndarray, motion_std: float = 0.025) -> None:
        displacement = np.asarray(displacement, dtype=float)
        if displacement.shape != (2,) or not np.isfinite(displacement).all() or motion_std < 0:
            raise ValueError("finite planar displacement and nonnegative motion_std required")
        self.state[:2] += displacement
        self.covariance[:2, :2] += np.eye(2) * motion_std**2

    def observe(self, identifier: int, relative_xy: np.ndarray) -> None:
        relative_xy = np.asarray(relative_xy, dtype=float)
        if relative_xy.shape != (2,) or not np.isfinite(relative_xy).all():
            raise ValueError("finite planar landmark measurement required")
        if identifier not in self.landmarks:
            index = len(self.state)
            old_covariance = self.covariance
            covariance = np.zeros((index + 2, index + 2))
            covariance[:index, :index] = old_covariance
            covariance[index:, :index] = old_covariance[:2, :]
            covariance[:index, index:] = old_covariance[:, :2]
            covariance[index:, index:] = old_covariance[:2, :2] + self.measurement_covariance
            self.state = np.r_[self.state, self.state[:2] + relative_xy]
            self.covariance = covariance
            self.landmarks[identifier] = index
            return
        index = self.landmarks[identifier]
        h = np.zeros((2, len(self.state)))
        h[:, :2] = -np.eye(2)
        h[:, index:index + 2] = np.eye(2)
        innovation = relative_xy - (self.state[index:index + 2] - self.state[:2])
        innovation_covariance = h @ self.covariance @ h.T + self.measurement_covariance
        gain = np.linalg.solve(innovation_covariance, h @ self.covariance).T
        self.state += gain @ innovation
        residual = np.eye(len(self.state)) - gain @ h
        self.covariance = residual @ self.covariance @ residual.T + gain @ self.measurement_covariance @ gain.T
        self.covariance = (self.covariance + self.covariance.T) / 2
        self.reobservations += 1

    def landmark_map(self) -> dict[int, list[float]]:
        return {identifier: self.state[index:index + 2].tolist() for identifier, index in self.landmarks.items()}


def _marker_colors() -> np.ndarray:
    return np.array([colorsys.hsv_to_rgb(index / 9, 1.0, 1.0) for index in range(9)])


def _model_with_markers(scenario: str) -> Any:
    import mujoco

    root = ET.parse(scenario_path(scenario)).getroot()
    asset, world = root.find("asset"), root.find("worldbody")
    assert asset is not None and world is not None
    for index, (x, y) in enumerate((x, y) for y in (-3.5, 0.0, 3.5) for x in (-3.5, 0.0, 3.5)):
        rgba = " ".join(str(float(value)) for value in _marker_colors()[index]) + " 1"
        ET.SubElement(asset, "material", name=f"slam_color_{index}", rgba=rgba, emission="1", specular="0")
        ET.SubElement(world, "geom", name=f"slam_marker_{index}", type="cylinder", pos=f"{x} {y} 1.9", size="0.18 0.02", material=f"slam_color_{index}", contype="0", conaffinity="0")
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))


def detect_landmarks(rgb: np.ndarray, depth: np.ndarray, fovy_degrees: float) -> dict[int, np.ndarray]:
    """Detect color marker centers and back-project RGB-D, without world poses.

    Fully visible saturated markers provide identity from color and metric XY
    from pixel coordinates/depth. The camera is calibrated and faces downward.
    通过颜色识别完整可见标记，用像素/深度反投影相对XY；不读取世界位姿。
    """
    if rgb.shape[:2] != depth.shape or rgb.ndim != 3 or rgb.shape[2] != 3:
        raise ValueError("aligned RGB and depth frames required")
    height, width = depth.shape
    pixels = rgb.astype(float) / 255
    saturation = pixels.max(axis=2) - pixels.min(axis=2)
    normalized = pixels / np.maximum(pixels.max(axis=2, keepdims=True), 1e-6)
    errors = ((normalized[:, :, None, :] - _marker_colors()) ** 2).sum(axis=3)
    labels = errors.argmin(axis=2)
    error = errors.min(axis=2)
    focal = height / (2 * math.tan(math.radians(fovy_degrees) / 2))
    observations = {}
    for identifier in range(9):
        ys, xs = np.nonzero((labels == identifier) & (error < 0.05) & (saturation > 0.6))
        if len(xs) < 4 or min(xs) <= 1 or max(xs) >= width - 2 or min(ys) <= 1 or max(ys) >= height - 2:
            continue
        distances = depth[ys, xs]
        x = np.mean((xs - (width - 1) / 2) * distances / focal)
        y = np.mean(((height - 1) / 2 - ys) * distances / focal)
        observations[identifier] = np.array([x, y])
    return observations


def run_slam_navigation(output: Path, scenario: str = "sparse", max_steps: int = 180, seed: int = 319) -> dict[str, Any]:
    """Navigate a closed local route with RGB-D SLAM pose feedback.

    Uses fixed-altitude, fixed-yaw kinematic motion with biased velocity actuation.
    Motion commands are filter inputs; colored landmark locations are learned.
    固定高度/航向运动学模型含执行偏差，利用RGB-D地标SLAM估计位姿闭环导航。
    """
    import mujoco

    if max_steps <= 0:
        raise ValueError("max_steps must be positive")
    output.mkdir(parents=True, exist_ok=True)
    model = _model_with_markers(scenario)
    data = mujoco.MjData(model)
    camera = model.camera("downward_rgb")
    renderer = mujoco.Renderer(model, height=180, width=240)
    depth_renderer = mujoco.Renderer(model, height=180, width=240)
    depth_renderer.enable_depth_rendering()
    generator = np.random.default_rng(seed)
    slam = PlanarLandmarkSLAM()
    origin = np.array([-2.0, -2.0])
    truth = origin.copy()
    odometry = np.zeros(2)
    route = [np.array(point) for point in ((3.5, 0), (3.5, 3.5), (0, 3.5), (0, 0))]
    target = 0
    trace, frames = [], []
    dt = 0.2
    try:
        for step in range(max_steps):
            set_drone_pose(model, data, float(truth[0]), float(truth[1]), 8.0, 0.0)
            renderer.update_scene(data, camera="downward_rgb")
            rgb = renderer.render().copy()
            depth_renderer.update_scene(data, camera="downward_rgb")
            depth = depth_renderer.render().copy()
            observations = detect_landmarks(rgb, depth, float(camera.fovy[0]))
            for identifier, relative_xy in observations.items():
                slam.observe(identifier, relative_xy + generator.normal(0, 0.006, 2))
            error = route[target] - slam.position
            distance = float(np.linalg.norm(error))
            if distance < 0.2:
                target += 1
                if target == len(route):
                    trace.append({"step": step, "truth_xy": (truth - origin).tolist(), "estimated_xy": slam.position.tolist(), "odometry_xy": odometry.tolist(), "observations": len(observations), "target": target})
                    frames.append(Image.fromarray(rgb))
                    break
                error = route[target] - slam.position
                distance = float(np.linalg.norm(error))
            velocity = error / max(distance, 1e-9) * min(1.0, distance)
            trace.append({"step": step, "truth_xy": (truth - origin).tolist(), "estimated_xy": slam.position.tolist(), "odometry_xy": odometry.tolist(), "observations": len(observations), "target": target})
            if step % 2 == 0:
                frames.append(Image.fromarray(rgb))
            displacement = velocity * dt
            # The simulator's biased actuation is deliberately not supplied to SLAM.
            truth += displacement * np.array([1.07, 0.94]) + generator.normal(0, 0.004, 2)
            odometry += displacement
            slam.predict(displacement)
    finally:
        renderer.close()
        depth_renderer.close()
    slam_errors = [np.linalg.norm(np.array(row["truth_xy"]) - row["estimated_xy"]) for row in trace]
    odometry_errors = [np.linalg.norm(np.array(row["truth_xy"]) - row["odometry_xy"]) for row in trace]
    result = {
        "mode": "planar_rgbd_landmark_slam_navigation", "motion_model": "fixed_altitude_fixed_yaw_kinematic",
        "landmark_map_source": "joint_filter_estimation", "pose_source": "rgbd_landmarks_and_command_odometry",
        "scenario": scenario, "steps": len(trace), "landmarks": len(slam.landmarks),
        "landmark_reobservations": slam.reobservations, "completed_waypoints": target,
        "total_waypoints": len(route), "navigation_completed": target == len(route),
        "position_rmse_m": float(np.sqrt(np.mean(np.square(slam_errors)))),
        "odometry_rmse_m": float(np.sqrt(np.mean(np.square(odometry_errors)))),
        "final_position_error_m": float(slam_errors[-1]),
    }
    (output / "slam_metrics.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    (output / "slam_trace.json").write_text(json.dumps(trace, indent=2) + "\n", encoding="utf-8")
    (output / "slam_landmarks.json").write_text(json.dumps(slam.landmark_map(), indent=2) + "\n", encoding="utf-8")
    if frames:
        frames[0].save(output / "slam_camera.png")
        frames[0].save(output / "slam_navigation.gif", save_all=True, append_images=frames[1:], duration=120, loop=0)
    canvas = Image.new("RGB", (480, 480), "white")
    draw = ImageDraw.Draw(canvas)
    mapped_landmarks = slam.landmark_map()
    plotted_points = [row["truth_xy"] for row in trace] + [row["estimated_xy"] for row in trace] + list(mapped_landmarks.values())
    minimum = np.min(plotted_points, axis=0) - 0.4
    maximum = np.max(plotted_points, axis=0) + 0.4
    scale = 400 / max(maximum - minimum)
    def pixel(xy):
        return (int(40 + (xy[0] - minimum[0]) * scale), int(440 - (xy[1] - minimum[1]) * scale))
    draw.line([pixel(row["truth_xy"]) for row in trace], fill="black", width=3)
    draw.line([pixel(row["estimated_xy"]) for row in trace], fill="blue", width=2)
    for point in mapped_landmarks.values():
        x, y = pixel(point)
        draw.ellipse((x-3, y-3, x+3, y+3), fill="orange")
    draw.text((12, 12), "RGB-D SLAM: blue estimate / black truth / orange landmarks", fill="black")
    canvas.save(output / "slam_map.png")
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Planar RGB-D landmark SLAM navigation / 平面 RGB-D 地标SLAM导航")
    parser.add_argument("--output", type=Path, default=Path("artifacts/slam"))
    parser.add_argument("--scenario", choices=("sparse", "dense", "occluded"), default="sparse")
    parser.add_argument("--max-steps", type=int, default=180)
    args = parser.parse_args()
    print(json.dumps(run_slam_navigation(args.output, args.scenario, args.max_steps), ensure_ascii=False))
