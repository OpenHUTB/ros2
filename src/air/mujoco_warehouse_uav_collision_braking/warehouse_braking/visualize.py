"""绘制真实 MuJoCo 画面与测距射线。 / Render actual MuJoCo frames and range beams."""
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image

from .simulation import ANGLES, DT, WarehouseSimulation, expert_target


def render_run(output: Path, scenario="static", duration=8.0):
    """在相同参考控制条件下保存动画。 / Save an animation under the reference controller."""
    output.mkdir(parents=True, exist_ok=True)
    sim = WarehouseSimulation(scenario, speed=1.2)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [5, 0, 0.5]
    camera.distance, camera.azimuth, camera.elevation = 13, 90, -65
    frames = []
    with mujoco.Renderer(sim.model, height=480, width=640) as renderer:
        for step in range(round(duration / DT)):
            ranges = sim.ranges()
            target, _ = expert_target(ranges, sim.data.qvel[0], 1.2)
            if step % 5 == 0:
                renderer.update_scene(sim.data, camera=camera)
                for angle, distance in zip(ANGLES, ranges):
                    scene = renderer.scene
                    geom = scene.geoms[scene.ngeom]
                    scene.ngeom += 1
                    mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_LINE, np.zeros(3), np.zeros(3),
                                       np.eye(3).ravel(), np.array([0.2, 1.0, 0.6, 1.0]))
                    endpoint = sim.position + distance * np.array([np.cos(angle), np.sin(angle), 0])
                    mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_LINE, 2.0, sim.position, endpoint)
                frames.append(Image.fromarray(renderer.render().copy()))
            sim.step(target)
    frames[len(frames) // 2].save(output / "warehouse_ranging.png")
    frames[0].save(output / "warehouse_ranging.gif", save_all=True, append_images=frames[1:],
                   duration=100, loop=0, optimize=True)
