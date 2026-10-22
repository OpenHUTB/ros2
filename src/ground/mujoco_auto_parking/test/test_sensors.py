#!/usr/bin/env python3
"""任务②传感器测试：前视相机、深度图、2D 激光雷达、车位感知网络。"""

import math
import os
import sys
from pathlib import Path

os.environ.setdefault('MUJOCO_GL', 'osmesa')

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

import numpy as np  # noqa: E402

from mujoco_auto_parking.camera_sensor import CameraSensor  # noqa: E402
from mujoco_auto_parking.lidar_sensor import Lidar2D  # noqa: E402
from mujoco_auto_parking.parking_perception import (  # noqa: E402
    IMAGE_SIZES,
    ImagePerceptionNetwork,
    image_features,
    relative_slot_pose,
    wrap_angle,
)
from mujoco_auto_parking.simulator import MujocoSimulation  # noqa: E402

MODEL_PATH = PACKAGE_ROOT / 'models' / 'parking_lot.xml'
WEIGHTS = PACKAGE_ROOT / 'models' / 'perception.npz'


def check_camera() -> None:
    simulation = MujocoSimulation(MODEL_PATH)
    simulation.set_pose(0.25, -1.0, math.pi / 2.0)
    camera = CameraSensor(
        simulation.model, simulation.data, width=160, height=120
    )
    rgb = camera.render_rgb()
    depth = camera.render_depth()
    assert rgb.shape == (120, 160, 3), rgb.shape
    assert rgb.std() > 1.0, rgb.std()
    assert depth.shape == (120, 160), depth.shape
    assert np.isfinite(depth).all()
    assert float(depth.max()) > 0.0
    camera.close()
    print(f'[PASS] 前视相机：{rgb.shape}, 亮度均值 {rgb.mean():.1f}, 标准差 {rgb.std():.1f}')
    print(f'[PASS] 深度图：{depth.shape}, 有效范围 {depth.min():.2f}~{depth.max():.2f} m')
    features = image_features(rgb)
    assert features.shape == (IMAGE_SIZES[0],), features.shape
    print(f'[PASS] 图像特征：{features.shape[0]} 维')


def check_lidar() -> None:
    simulation = MujocoSimulation(MODEL_PATH)
    simulation.set_pose(0.25, -1.0, math.pi / 2.0)
    lidar = Lidar2D(simulation.model, simulation.data, num_beams=120)
    scan = lidar.scan()
    ranges = scan['ranges']
    assert ranges.shape == (120,), ranges.shape
    assert np.isfinite(ranges).all()
    assert float(ranges.min()) < 8.0
    assert float(ranges.max()) <= 8.0
    print(f'[PASS] 2D 激光雷达：120 束，最近 {ranges.min():.2f} m，最远 {ranges.max():.2f} m')


def check_perception_network() -> None:
    assert WEIGHTS.is_file(), f'缺少感知权值: {WEIGHTS}'
    network = ImagePerceptionNetwork(WEIGHTS)
    simulation = MujocoSimulation(MODEL_PATH)
    camera = CameraSensor(
        simulation.model, simulation.data, width=160, height=120
    )
    simulation.set_pose(0.25, -1.0, math.pi / 2.0)
    rgb = camera.render_rgb()
    dx, dy, dtheta = network.predict(rgb)
    true_dx, true_dy, true_dtheta = relative_slot_pose(
        0.25, -1.0, math.pi / 2.0
    )
    assert abs(dx - true_dx) < 0.8, (dx, true_dx)
    assert abs(dy - true_dy) < 0.8, (dy, true_dy)
    assert abs(wrap_angle(dtheta - true_dtheta)) < 0.6, (dtheta, true_dtheta)
    print(
        '[PASS] 车位感知网络：'
        f'预测 ({dx:.2f}, {dy:.2f}, {dtheta:.2f})，'
        f'真值 ({true_dx:.2f}, {true_dy:.2f}, {true_dtheta:.2f})'
    )
    camera.close()


def main() -> None:
    check_camera()
    check_lidar()
    check_perception_network()
    print('\n任务②传感器测试全部通过。')


if __name__ == '__main__':
    main()
