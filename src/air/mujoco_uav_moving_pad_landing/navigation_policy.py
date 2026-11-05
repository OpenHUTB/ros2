"""Neural lidar navigation with a MuJoCo plane-motion plant."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image
from sklearn.metrics import accuracy_score
from sklearn.model_selection import GroupShuffleSplit
from sklearn.neural_network import MLPClassifier
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler
import joblib

from .navigation import Cell, GridWorld, OccupancyGrid, astar, cast_ray, render_map


MODULE_DIR = Path(__file__).resolve().parent
POLICY_PATH = MODULE_DIR / "output" / "navigation_policy.joblib"
IMAGE_SIZE = 16
CELL_SIZE = 0.04
ACTION_DELTAS: tuple[Cell, ...] = ((1, 0), (-1, 0), (0, 1), (0, -1))
SENSOR_ANGLES = tuple(index * math.pi / 4.0 for index in range(8))


@dataclass(frozen=True)
class Scenario:
    """One fixed obstacle layout used for training or evaluation."""

    name: str
    world: GridWorld
    start: Cell = (2, 2)
    goal: Cell = (21, 13)


def evaluation_scenarios() -> tuple[Scenario, ...]:
    """Return three distinct obstacle layouts with shared start and goal."""
    return (
        Scenario("corridor", GridWorld()),
        Scenario(
            "offset_barriers",
            GridWorld(obstacles=((5, 4, 10, 5), (13, 2, 14, 10), (18, 9, 19, 13))),
        ),
        Scenario(
            "staggered_blocks",
            GridWorld(obstacles=((4, 7, 9, 8), (11, 2, 12, 8), (16, 6, 18, 10))),
        ),
    )


def _cell_world_position(cell: Cell, width: int, height: int) -> tuple[float, float]:
    return ((cell[0] - (width - 1) / 2.0) * CELL_SIZE,
            (cell[1] - (height - 1) / 2.0) * CELL_SIZE)


def lidar_features(world: GridWorld, pose: Cell, goal: Cell, max_range: float = 24.0) -> np.ndarray:
    """Normalize eight simulated lidar ranges and the relative goal vector."""
    ranges = []
    for angle in SENSOR_ANGLES:
        endpoint, _hit = cast_ray(world, pose, angle, max_range)
        ranges.append(min(max_range, math.dist(pose, endpoint)) / max_range)
    dx = (goal[0] - pose[0]) / world.width
    dy = (goal[1] - pose[1]) / world.height
    return np.asarray([*ranges, dx, dy], dtype=np.float32)


def _complete_map(world: GridWorld) -> OccupancyGrid:
    grid = OccupancyGrid(world.width, world.height)
    for y in range(world.height):
        for x in range(world.width):
            grid.data[y][x] = 100 if world.occupied((x, y)) else 0
    return grid


def _episode_path(scenario: Scenario) -> list[Cell]:
    return astar(_complete_map(scenario.world), scenario.start, scenario.goal)


def collect_policy_samples(episodes_per_scene: int = 30, seed: int = 11):
    """Build lidar/action examples from expert paths in complete maps."""
    rng = np.random.default_rng(seed)
    features: list[np.ndarray] = []
    labels: list[int] = []
    groups: list[int] = []
    group_id = 0

    for scene_index, scenario in enumerate(evaluation_scenarios()):
        free_cells = [
            (x, y)
            for y in range(1, scenario.world.height - 1)
            for x in range(1, scenario.world.width - 1)
            if not scenario.world.occupied((x, y))
        ]
        for _episode in range(episodes_per_scene):
            start, goal = rng.choice(len(free_cells), size=2, replace=False)
            episode = Scenario(
                scenario.name,
                scenario.world,
                free_cells[int(start)],
                free_cells[int(goal)],
            )
            path = astar(_complete_map(episode.world), episode.start, episode.goal)
            for pose, next_pose in zip(path, path[1:]):
                features.append(lidar_features(episode.world, pose, episode.goal))
                delta = (next_pose[0] - pose[0], next_pose[1] - pose[1])
                labels.append(ACTION_DELTAS.index(delta))
                groups.append(group_id)
            group_id += 1

    return np.stack(features), np.asarray(labels, dtype=np.int64), np.asarray(groups, dtype=np.int32)


def train_navigation_policy(
    path: Path = POLICY_PATH,
    episodes_per_scene: int = 30,
    seed: int = 11,
) -> dict[str, float | int | str]:
    """Train an MLP action policy and evaluate on held-out navigation episodes."""
    if episodes_per_scene < 4:
        raise ValueError("episodes_per_scene must be at least four")
    x, y, groups = collect_policy_samples(episodes_per_scene=episodes_per_scene, seed=seed)
    splitter = GroupShuffleSplit(n_splits=1, test_size=0.2, random_state=seed)
    train_rows, test_rows = next(splitter.split(x, y, groups=groups))
    train_groups = np.unique(groups[train_rows])
    test_groups = np.unique(groups[test_rows])
    if np.intersect1d(train_groups, test_groups).size:
        raise RuntimeError("episode leakage detected between navigation train and test sets")
    model = make_pipeline(
        StandardScaler(),
        MLPClassifier(
            hidden_layer_sizes=(32, 16),
            activation="tanh",
            solver="adam",
            alpha=0.001,
            learning_rate_init=0.003,
            max_iter=500,
            random_state=seed,
            early_stopping=True,
            n_iter_no_change=20,
        ),
    )
    model.fit(x[train_rows], y[train_rows])
    accuracy = float(accuracy_score(y[test_rows], model.predict(x[test_rows])))
    path.parent.mkdir(parents=True, exist_ok=True)
    joblib.dump(model, path)
    return {
        "input": "8 simulated lidar ranges + relative goal",
        "train_samples": int(train_rows.size),
        "test_samples": int(test_rows.size),
        "train_episodes": int(train_groups.size),
        "test_episodes": int(test_groups.size),
        "test_action_accuracy": accuracy,
        "scenes": len(evaluation_scenarios()),
        "model_path": str(path),
    }


class MuJoCoGridPlant:
    """MuJoCo slide-joint plant that physically advances between grid cells."""

    def __init__(self, scenario: Scenario):
        self.scenario = scenario
        self.model = mujoco.MjModel.from_xml_string(self._scene_xml(scenario))
        self.data = mujoco.MjData(self.model)
        self.x_qpos = self.model.joint("uav_x").qposadr[0]
        self.y_qpos = self.model.joint("uav_y").qposadr[0]
        self.x_ctrl = self.model.actuator("x_servo").id
        self.y_ctrl = self.model.actuator("y_servo").id
        self.data.qpos[self.x_qpos], self.data.qpos[self.y_qpos] = _cell_world_position(
            scenario.start, scenario.world.width, scenario.world.height
        )
        self._set_target(scenario.start)
        mujoco.mj_forward(self.model, self.data)

    @staticmethod
    def _scene_xml(scenario: Scenario) -> str:
        world = scenario.world
        half_x = world.width * CELL_SIZE / 2.0
        half_y = world.height * CELL_SIZE / 2.0
        geoms = [
            '<geom name="floor" type="plane" pos="0 0 0" size="2 2 0.1" rgba="0.9 0.9 0.9 1" contype="0" conaffinity="0"/>',
        ]
        for y in range(world.height):
            for x in range(world.width):
                if not world.occupied((x, y)):
                    continue
                pos_x, pos_y = _cell_world_position((x, y), world.width, world.height)
                geoms.append(
                    f'<geom name="obstacle_{x}_{y}" type="box" pos="{pos_x:.4f} {pos_y:.4f} 0.12" '
                    f'size="{CELL_SIZE / 2:.4f} {CELL_SIZE / 2:.4f} 0.12" rgba="0.18 0.22 0.24 1"/>'
                )
        return (
            '<mujoco model="neural_grid_navigation"><option timestep="0.02" gravity="0 0 0" '
            'integrator="implicitfast"/><worldbody>'
            + "".join(geoms)
            + '<body name="uav" pos="0 0 0.12">'
            '<joint name="uav_x" type="slide" axis="1 0 0" damping="4" limited="true" '
            f'range="{-half_x:.3f} {half_x:.3f}"/>'
            '<joint name="uav_y" type="slide" axis="0 1 0" damping="4" limited="true" '
            f'range="{-half_y:.3f} {half_y:.3f}"/>'
            '<geom type="sphere" size="0.012" mass="0.08" rgba="0.85 0.28 0.2 1"/>'
            '</body></worldbody><actuator>'
            f'<position name="x_servo" joint="uav_x" kp="500" ctrlrange="{-half_x:.3f} {half_x:.3f}"/>'
            f'<position name="y_servo" joint="uav_y" kp="500" ctrlrange="{-half_y:.3f} {half_y:.3f}"/>'
            '</actuator></mujoco>'
        )

    def _set_target(self, cell: Cell) -> None:
        target_x, target_y = _cell_world_position(cell, self.scenario.world.width, self.scenario.world.height)
        self.data.ctrl[self.x_ctrl] = target_x
        self.data.ctrl[self.y_ctrl] = target_y

    @property
    def pose(self) -> Cell:
        x_world = float(self.data.qpos[self.x_qpos])
        y_world = float(self.data.qpos[self.y_qpos])
        x = int(round(x_world / CELL_SIZE + (self.scenario.world.width - 1) / 2.0))
        y = int(round(y_world / CELL_SIZE + (self.scenario.world.height - 1) / 2.0))
        return (min(max(x, 0), self.scenario.world.width - 1),
                min(max(y, 0), self.scenario.world.height - 1))

    def move_to(self, cell: Cell, steps: int = 18) -> Cell:
        self._set_target(cell)
        for _ in range(steps):
            mujoco.mj_step(self.model, self.data)
        return self.pose


def _safe_policy_action(
    model,
    world: GridWorld,
    grid: OccupancyGrid,
    pose: Cell,
    goal: Cell,
    recent_poses: list[Cell],
) -> tuple[int, bool]:
    action = int(model.predict(lidar_features(world, pose, goal).reshape(1, -1))[0])
    dx, dy = ACTION_DELTAS[action]
    candidate = (pose[0] + dx, pose[1] + dy)
    is_safe = grid.contains(candidate) and grid.data[candidate[1]][candidate[0]] != 100
    is_progress = candidate == goal or candidate not in recent_poses[-8:]
    if is_safe and is_progress:
        return action, False
    route = astar(grid, pose, goal)
    if len(route) < 2:
        return action, True
    delta = (route[1][0] - pose[0], route[1][1] - pose[1])
    return ACTION_DELTAS.index(delta), True


def run_navigation_suite(
    output_dir: Path,
    model_path: Path = POLICY_PATH,
    episodes_per_scene: int = 30,
) -> dict[str, object]:
    """Evaluate the neural policy over three MuJoCo lidar-mapping scenes."""
    if not model_path.exists():
        train_navigation_policy(model_path, episodes_per_scene=episodes_per_scene)
    model = joblib.load(model_path)
    output_dir.mkdir(parents=True, exist_ok=True)
    scene_metrics: list[dict[str, object]] = []
    frames: list[Image.Image] = []

    for scene_index, scenario in enumerate(evaluation_scenarios()):
        grid = OccupancyGrid(scenario.world.width, scenario.world.height)
        plant = MuJoCoGridPlant(scenario)
        visited = [scenario.start]
        fallbacks = 0
        try:
            pose = plant.pose
            grid.update_scan(scenario.world, pose)
            for _ in range(scenario.world.width * scenario.world.height):
                if pose == scenario.goal:
                    break
                action, used_fallback = _safe_policy_action(
                    model, scenario.world, grid, pose, scenario.goal, visited
                )
                fallbacks += int(used_fallback)
                dx, dy = ACTION_DELTAS[action]
                target = (pose[0] + dx, pose[1] + dy)
                if not grid.contains(target) or scenario.world.occupied(target):
                    break
                measured_pose = plant.move_to(target)
                if measured_pose == pose:
                    break
                pose = measured_pose
                visited.append(pose)
                grid.update_scan(scenario.world, pose)
                frames.append(render_map(grid, [pose], visited, scenario.start, scenario.goal))
            reached = pose == scenario.goal
            route = astar(_complete_map(scenario.world), scenario.start, scenario.goal)
            scene_map = render_map(grid, route, visited, scenario.start, scenario.goal)
            scene_map.save(output_dir / f"navigation_{scenario.name}.png")
            scene_metrics.append({
                "scene": scenario.name,
                "goal_reached": reached,
                "navigation_steps": len(visited) - 1,
                "map_coverage": round(grid.coverage(), 4),
                "policy_fallbacks": fallbacks,
                "final_pose_cell": list(pose),
            })
        finally:
            del plant

    if frames:
        frames[::2][0].save(
            output_dir / "navigation_run.gif",
            save_all=True,
            append_images=frames[::2][1:],
            duration=90,
            loop=0,
            optimize=True,
        )
    aggregate: dict[str, object] = {
        "planner": "neural_mlp",
        "pose_source": "MuJoCo slide-joint state",
        "sensor": "simulated 2D lidar",
        "scenes": scene_metrics,
        "success_rate": sum(bool(item["goal_reached"]) for item in scene_metrics) / len(scene_metrics),
        "mean_map_coverage": round(
            sum(float(item["map_coverage"]) for item in scene_metrics) / len(scene_metrics), 4
        ),
    }
    (output_dir / "navigation_metrics.json").write_text(
        json.dumps(aggregate, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    return aggregate
