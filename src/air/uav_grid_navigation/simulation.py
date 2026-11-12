"""MuJoCo UAV motion, virtual lidar mapping, and A* replanning loop."""

from __future__ import annotations

import json
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image

if __package__:
    from .navigation import Cell, bresenham, mark_blocked_waypoint, plan_route, render_route
else:
    from navigation import Cell, bresenham, mark_blocked_waypoint, plan_route, render_route

CELL_SIZE = 0.25
FLIGHT_HEIGHT = 0.38
LIDAR_RAYS = 180
LIDAR_MAX_RANGE = 8.0


@dataclass(frozen=True)
class GridWorld:
    """World dimensions and fixed obstacle rectangles for the sample route."""

    width: int = 24
    height: int = 16
    obstacles: tuple[tuple[int, int, int, int], ...] = (
        (7, 1, 8, 9),
        (12, 6, 16, 7),
        (19, 7, 20, 14),
    )

    @property
    def start(self) -> Cell:
        return (2, 2)

    @property
    def goal(self) -> Cell:
        return (21, 13)

    def occupied(self, cell: Cell) -> bool:
        x, y = cell
        if x <= 0 or y <= 0 or x >= self.width - 1 or y >= self.height - 1:
            return True
        return any(x0 <= x <= x1 and y0 <= y <= y1 for x0, y0, x1, y1 in self.obstacles)


def _cell_center(cell: Cell) -> tuple[float, float]:
    return ((cell[0] + 0.5) * CELL_SIZE, (cell[1] + 0.5) * CELL_SIZE)


def _build_model(world: GridWorld) -> mujoco.MjModel:
    """Build a planar MuJoCo scene with a controllable UAV and box obstacles."""
    origin_x, origin_y = _cell_center(world.start)
    root = ET.Element("mujoco", model="uav_grid_navigation")
    ET.SubElement(root, "compiler", angle="radian", autolimits="true")
    ET.SubElement(root, "option", timestep="0.01", gravity="0 0 0", integrator="implicitfast")
    worldbody = ET.SubElement(root, "worldbody")
    ET.SubElement(
        worldbody,
        "geom",
        name="floor",
        type="plane",
        pos=f"{world.width * CELL_SIZE / 2} {world.height * CELL_SIZE / 2} 0",
        size=f"{world.width * CELL_SIZE} {world.height * CELL_SIZE} 0.1",
        rgba="0.88 0.90 0.89 1",
        contype="0",
        conaffinity="0",
    )

    for y in range(world.height):
        for x in range(world.width):
            if not world.occupied((x, y)):
                continue
            center_x, center_y = _cell_center((x, y))
            ET.SubElement(
                worldbody,
                "geom",
                name=f"obstacle_{x}_{y}",
                type="box",
                pos=f"{center_x} {center_y} 0.35",
                size=f"{CELL_SIZE / 2} {CELL_SIZE / 2} 0.35",
                rgba="0.19 0.25 0.27 1",
            )

    uav = ET.SubElement(worldbody, "body", name="uav", pos=f"{origin_x} {origin_y} {FLIGHT_HEIGHT}")
    ET.SubElement(uav, "joint", name="x_axis", type="slide", axis="1 0 0", range="-8 8", damping="8")
    ET.SubElement(uav, "joint", name="y_axis", type="slide", axis="0 1 0", range="-8 8", damping="8")
    ET.SubElement(uav, "geom", name="uav_body", type="sphere", size="0.07", mass="0.35", rgba="0.82 0.25 0.19 1")
    ET.SubElement(root, "actuator")
    actuator = root.find("actuator")
    ET.SubElement(actuator, "position", name="x_motor", joint="x_axis", kp="120", kv="18", ctrlrange="-8 8")
    ET.SubElement(actuator, "position", name="y_motor", joint="y_axis", kp="120", kv="18", ctrlrange="-8 8")
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    return model


class MuJoCoGridSimulation:
    """Read MuJoCo pose, move between waypoints, and scan the surrounding map."""

    def __init__(self, world: GridWorld):
        self.world = world
        self.model = _build_model(world)
        self.data = mujoco.MjData(self.model)
        self.origin = np.asarray(_cell_center(world.start), dtype=float)
        self.uav_body_id = self.model.body("uav").id
        self.uav_geom_id = self.model.geom("uav_body").id
        self.qpos_ids = [self.model.joint(name).qposadr[0] for name in ("x_axis", "y_axis")]
        self.qvel_ids = [self.model.joint(name).dofadr[0] for name in ("x_axis", "y_axis")]
        self.actuator_ids = [self.model.actuator(name).id for name in ("x_motor", "y_motor")]
        self.grid = [
            [
                1 if x == 0 or y == 0 or x == world.width - 1 or y == world.height - 1 else -1
                for x in range(world.width)
            ]
            for y in range(world.height)
        ]
        mujoco.mj_forward(self.model, self.data)

    @property
    def position(self) -> np.ndarray:
        return self.origin + self.data.qpos[self.qpos_ids].copy()

    @property
    def cell(self) -> Cell:
        x, y = np.floor(self.position / CELL_SIZE).astype(int)
        return (min(max(int(x), 0), self.world.width - 1), min(max(int(y), 0), self.world.height - 1))

    def move_to(self, cell: Cell, max_steps: int = 120) -> bool:
        target = np.asarray(_cell_center(cell), dtype=float)
        target_qpos = target - self.origin
        self.data.ctrl[self.actuator_ids] = target_qpos
        for _ in range(max_steps):
            mujoco.mj_step(self.model, self.data)
            position_error = np.linalg.norm(self.position - target)
            speed = np.linalg.norm(self.data.qvel[self.qvel_ids])
            if position_error < 0.035 and speed < 0.12:
                return True
        return False

    def has_obstacle_contact(self) -> bool:
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            if self.uav_geom_id in (contact.geom1, contact.geom2):
                return True
        return False

    def update_lidar_map(self, ray_count: int = LIDAR_RAYS, max_range: float = LIDAR_MAX_RANGE) -> int:
        """Cast MuJoCo rays and update free and occupied cells from measured ranges."""
        origin = self.position
        pose_cell = self.cell
        geomgroup = np.ones(6, dtype=np.uint8)
        occupied_hits = 0
        for ray_index in range(ray_count):
            angle = 2.0 * math.pi * ray_index / ray_count
            direction = np.asarray([math.cos(angle), math.sin(angle), 0.0], dtype=float)
            geom_id = np.full(1, -1, dtype=np.int32)
            distance = mujoco.mj_ray(
                self.model,
                self.data,
                np.asarray([origin[0], origin[1], FLIGHT_HEIGHT]),
                direction,
                geomgroup,
                True,
                self.uav_body_id,
                geom_id,
            )
            hit = 0 <= distance <= max_range and geom_id[0] >= 0
            measured_range = min(max_range, distance + CELL_SIZE * 0.08) if hit else max_range
            endpoint = origin + direction[:2] * measured_range
            endpoint_cell = tuple(np.floor(endpoint / CELL_SIZE).astype(int))
            cells = bresenham(pose_cell, endpoint_cell)
            for x, y in cells[:-1] if hit else cells:
                if 0 <= x < self.world.width and 0 <= y < self.world.height and self.grid[y][x] != 1:
                    self.grid[y][x] = 0
            if hit and 0 <= endpoint_cell[0] < self.world.width and 0 <= endpoint_cell[1] < self.world.height:
                self.grid[endpoint_cell[1]][endpoint_cell[0]] = 1
                occupied_hits += 1
        return occupied_hits


def _run_navigation(sim: MuJoCoGridSimulation, output_dir: Path) -> dict[str, object]:
    """Follow A* waypoints using MuJoCo pose feedback and virtual lidar scans."""
    world = sim.world
    pose = world.start
    visited = [pose]
    frames: list[Image.Image] = []
    scan_count = 0
    replan_count = 0
    contact_count = 0
    trajectory: list[dict[str, float | int]] = []
    sim.update_lidar_map()
    scan_count += 1
    for _ in range(world.width * world.height * 2):
        if pose == world.goal:
            break
        replan_count += 1
        route = plan_route(sim.grid, pose, world.goal)
        if len(route) < 2:
            raise RuntimeError("No route remains in the lidar occupancy map")
        next_cell = route[1]
        arrived = sim.move_to(next_cell)
        if not arrived:
            if not mark_blocked_waypoint(sim.grid, next_cell, sim.has_obstacle_contact()):
                raise RuntimeError(f"MuJoCo waypoint controller did not reach cell {next_cell}")
            contact_count += 1
        else:
            pose = sim.cell
            if pose != visited[-1]:
                visited.append(pose)
                position = sim.position
                trajectory.append({"step": len(visited) - 1, "x_m": float(position[0]), "y_m": float(position[1]),
                                   "cell_x": pose[0], "cell_y": pose[1]})
        sim.update_lidar_map()
        scan_count += 1
        route = plan_route(sim.grid, pose, world.goal)
        frames.append(
            render_route(
                sim.grid,
                route,
                world.start,
                world.goal,
                cell_size=18,
                visited=visited,
            )
        )
    else:
        raise RuntimeError("Navigation step limit reached before the goal")

    final_map = render_route(
        sim.grid,
        visited,
        world.start,
        world.goal,
        cell_size=18,
        visited=visited,
    )
    final_map.save(output_dir / "navigation_map.png")
    if frames:
        frames[::2][0].save(
            output_dir / "navigation_run.gif",
            save_all=True,
            append_images=frames[::2][1:],
            duration=100,
            loop=0,
            optimize=True,
        )
    interior = [
        sim.grid[y][x]
        for y in range(1, world.height - 1)
        for x in range(1, world.width - 1)
    ]
    interior_cell_count = (world.width - 2) * (world.height - 2)
    result: dict[str, object] = {
        "goal_reached": pose == world.goal,
        "navigation_steps": len(visited) - 1,
        "travelled_route_length_cells": len(visited) - 1,
        "map_coverage": round(sum(value >= 0 for value in interior) / interior_cell_count, 4),
        "interior_cell_count": interior_cell_count,
        "boundary_cells_preseeded": world.width * world.height - interior_cell_count,
        "lidar_scans": scan_count,
        "replans": replan_count,
        "obstacle_contacts": contact_count,
        "final_x_cell": pose[0],
        "final_y_cell": pose[1],
        "final_x_m": round(float(sim.position[0]), 3),
        "final_y_m": round(float(sim.position[1]), 3),
    }
    (output_dir / "trajectory.json").write_text(json.dumps(trajectory, ensure_ascii=False, indent=2), encoding="utf-8")
    (output_dir / "navigation_metrics.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    return result


def run_mujoco_navigation(output_dir: Path) -> dict[str, object]:
    """Run the mapped, feedback-controlled MuJoCo navigation example."""
    output_dir.mkdir(parents=True, exist_ok=True)
    sim = MuJoCoGridSimulation(GridWorld())
    return _run_navigation(sim, output_dir)


if __name__ == "__main__":
    print(json.dumps(run_mujoco_navigation(Path("output")), ensure_ascii=False, indent=2))
