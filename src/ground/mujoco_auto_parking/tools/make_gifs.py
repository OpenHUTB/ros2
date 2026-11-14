#!/usr/bin/env python3
"""Generate the four demonstration GIFs from real MuJoCo runs."""

import math
import os
import sys
from pathlib import Path

os.environ.setdefault('MUJOCO_GL', 'osmesa')

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(PACKAGE_ROOT))

import cv2  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402

from mujoco_auto_parking.camera_sensor import CameraSensor  # noqa: E402
from mujoco_auto_parking.end_to_end_networks import (  # noqa: E402
    VisuomotorController,
)
from mujoco_auto_parking.kinematics import DriveGeometry  # noqa: E402
from mujoco_auto_parking.lidar_sensor import Lidar2D  # noqa: E402
from mujoco_auto_parking.navigation import NavigationCore  # noqa: E402
from mujoco_auto_parking.simulator import MujocoSimulation  # noqa: E402
from mujoco_auto_parking.trajectory_control import (  # noqa: E402
    ReferenceTrajectory,
    TrajectoryController,
    build_s_curve,
)

OUTPUT_DIR = (
    Path(os.environ['AUTO_PARKING_DOCS_DIR'])
    / 'img' / 'ground' / 'mujoco_auto_parking'
    if os.environ.get('AUTO_PARKING_DOCS_DIR')
    else REPO_ROOT / 'docs' / 'img' / 'ground' / 'mujoco_auto_parking'
)
MODEL_PATH = PACKAGE_ROOT / 'models' / 'parking_lot.xml'
WEIGHTS = PACKAGE_ROOT / 'models'
CONTROL_PERIOD = 0.02
STEP = 4


def frame_from(renderer, label):
    rgb = renderer.render_rgb()
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    cv2.putText(
        bgr, label, (8, 20), cv2.FONT_HERSHEY_SIMPLEX,
        0.45, (255, 255, 255), 1, cv2.LINE_AA,
    )
    bgr = cv2.resize(bgr, (240, 180), interpolation=cv2.INTER_AREA)
    return Image.fromarray(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))


def save_gif(frames, name):
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    path = OUTPUT_DIR / name
    frames[0].save(
        path,
        save_all=True,
        append_images=frames[1:],
        duration=100,
        loop=0,
        optimize=True,
    )
    print(f'{path}  {path.stat().st_size / 1024 / 1024:.2f} MB  {len(frames)} 帧')


def make_task1(simulation, overview):
    simulation.reset()
    simulation.set_pose(-6.0, 0.0, 0.0)
    frames = [frame_from(overview, 'Task 1: keyboard motion (scripted)')]
    segments = [
        ('forward', 4.0, 4.0, 1.2),
        ('left turn', -3.0, 3.0, 1.2),
        ('forward', 4.0, 4.0, 1.2),
        ('stop', 0.0, 0.0, 0.6),
    ]
    for _, left, right, duration in segments:
        steps = int(duration / CONTROL_PERIOD)
        for index in range(steps):
            for _ in range(STEP):
                simulation.apply_wheel_speeds(left, right)
                simulation.step()
            if index % 5 == 0:
                frames.append(frame_from(overview, 'Task 1: keyboard motion (scripted)'))
    save_gif(frames, 'task1_teleop.gif')


def make_task2(simulation, overview):
    controller = TrajectoryController(WEIGHTS / 'trajectory.npz')
    geometry = DriveGeometry(0.08, 0.48, 8.0)
    start = (-6.0, 0.0, 0.0)
    trajectory = ReferenceTrajectory(build_s_curve(*start))
    simulation.reset()
    simulation.set_pose(*start)
    frames = [frame_from(overview, 'Task 2: NN S-curve tracking')]
    for counter in range(700):
        state = simulation.planar_state()
        index, distance, reference = trajectory.lookahead(state.x, state.y)
        speed, yaw_rate = controller.predict(
            (state.x, state.y, state.yaw, state.linear_velocity),
            reference,
            0.45,
        )
        left, right = geometry.wheel_speeds(speed, yaw_rate)
        for _ in range(STEP):
            simulation.apply_wheel_speeds(left, right)
            simulation.step()
        if counter % 5 == 0:
            frames.append(frame_from(overview, 'Task 2: NN S-curve tracking'))
        if index >= len(trajectory.points) - 4 and distance < 0.25:
            break
    save_gif(frames, 'task2_s_curve.gif')


def make_task3(simulation, overview):
    lidar = Lidar2D(simulation.model, simulation.data, num_beams=120)
    core = NavigationCore(
        WEIGHTS / 'planner.npz', WEIGHTS / 'trajectory.npz'
    )
    goal = (0.25, 1.2)
    simulation.reset()
    simulation.set_pose(-6.0, 0.0, 0.0)
    frames = [frame_from(overview, 'Task 3: mapping + Neural A*')]
    last_plan = -1e9
    time_elapsed = 0.0
    for counter in range(1000):
        state = simulation.planar_state()
        scan_dict = lidar.scan()
        ranges = scan_dict['ranges']

        class _Scan:
            pass

        scan = _Scan()
        scan.ranges = ranges
        scan.angle_min = lidar.angle_min
        scan.angle_increment = lidar.angle_increment
        scan.range_min = lidar.min_range
        scan.range_max = lidar.max_range
        core.update_map(state, scan)
        if time_elapsed - last_plan >= 1.0:
            core.plan(state, goal)
            last_plan = time_elapsed
        speed, yaw_rate = core.control(state)
        if float(np.min(ranges)) < 0.35:
            speed = 0.0
        left, right = core.geometry.wheel_speeds(speed, yaw_rate)
        for _ in range(STEP):
            simulation.apply_wheel_speeds(left, right)
            simulation.step()
        time_elapsed += CONTROL_PERIOD
        if counter % 5 == 0:
            frames.append(frame_from(overview, 'Task 3: mapping + Neural A*'))
        if math.hypot(state.x - goal[0], state.y - goal[1]) < 0.35:
            break
    save_gif(frames, 'task3_navigation.gif')


def make_task4(simulation, overview):
    front = CameraSensor(
        simulation.model, simulation.data, camera_name='front_camera',
        width=160, height=120,
    )
    controller = VisuomotorController(
        architecture='small_cnn',
        weights_path=WEIGHTS / 'e2e_small_cnn_bc.npz',
    )
    geometry = DriveGeometry(0.08, 0.48, 8.0)
    start = (-6.0, 0.0, 0.0)
    trajectory = ReferenceTrajectory(build_s_curve(*start))
    simulation.reset()
    simulation.set_pose(*start)
    frames = [frame_from(overview, 'Task 4: image -> control CNN')]
    for counter in range(700):
        state = simulation.planar_state()
        index, distance, _ = trajectory.lookahead(state.x, state.y)
        result = controller.predict_action(front.render_rgb())
        left, right = geometry.wheel_speeds(result['v'], result['wz'])
        for _ in range(STEP):
            simulation.apply_wheel_speeds(left, right)
            simulation.step()
        if counter % 5 == 0:
            frames.append(frame_from(overview, 'Task 4: image -> control CNN'))
        if index >= len(trajectory.points) - 4 and distance < 0.25:
            break
    save_gif(frames, 'task4_end_to_end.gif')


def main() -> None:
    simulation = MujocoSimulation(MODEL_PATH)
    overview = CameraSensor(
        simulation.model, simulation.data, camera_name='follow_camera',
        width=320, height=240,
    )
    make_task1(simulation, overview)
    make_task2(simulation, overview)
    make_task3(simulation, overview)
    make_task4(simulation, overview)


if __name__ == '__main__':
    main()
