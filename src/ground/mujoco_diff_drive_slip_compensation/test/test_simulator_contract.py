"""Smoke tests for the MJCF and importable simulation contract."""

import importlib
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

from mujoco_diff_drive_slip_compensation.simulator import (
    MujocoSimulation,
    quaternion_to_yaw,
)


PACKAGE_ROOT = Path(__file__).parents[1]


def test_mjcf_declares_required_named_objects():
    root = ET.parse(PACKAGE_ROOT / 'models' / 'diff_drive.xml').getroot()
    actuator_names = {
        element.attrib['name'] for element in root.findall('./actuator/*')
    }
    body_names = {
        element.attrib['name'] for element in root.findall('.//body')
    }

    assert actuator_names == {
        'left_front',
        'left_rear',
        'right_front',
        'right_rear',
    }
    assert 'chassis' in body_names


def test_configuration_matches_model_geometry():
    config = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'simulation.yaml').read_text(encoding='utf-8')
    )['mujoco_diff_drive_simulation']['ros__parameters']

    assert config['model_file'] == 'models/diff_drive.xml'
    assert config['wheel_radius'] == pytest.approx(0.08)
    assert config['track_width'] == pytest.approx(0.42)
    assert config['command_timeout'] > 0.0


def test_quaternion_to_yaw_uses_mujoco_wxyz_order():
    half_angle = np.pi / 4.0
    quaternion = np.array([np.cos(half_angle), 0.0, 0.0, np.sin(half_angle)])

    assert quaternion_to_yaw(quaternion) == pytest.approx(np.pi / 2.0)


def test_main_module_import_does_not_require_ros_installation():
    module = importlib.import_module('mujoco_diff_drive_slip_compensation.main')

    assert callable(module.main)


def test_mujoco_model_loads_and_steps():
    pytest.importorskip('mujoco')
    simulation = MujocoSimulation(PACKAGE_ROOT / 'models' / 'diff_drive.xml')

    simulation.apply_wheel_speeds(2.0, 2.0)
    for _ in range(20):
        simulation.step()
    state = simulation.planar_state()

    assert simulation.timestep == pytest.approx(0.005)
    assert state.time == pytest.approx(0.1)
    assert np.isfinite([state.x, state.y, state.yaw]).all()


def test_planar_state_reports_velocity_in_chassis_frame():
    mujoco = pytest.importorskip('mujoco')
    simulation = MujocoSimulation(PACKAGE_ROOT / 'models' / 'diff_drive.xml')

    half_angle = np.pi / 4.0
    simulation.data.qpos[3:7] = [
        np.cos(half_angle),
        0.0,
        0.0,
        np.sin(half_angle),
    ]
    simulation.data.qvel[:3] = [0.0, 2.0, 0.0]
    mujoco.mj_forward(simulation.model, simulation.data)

    state = simulation.planar_state()

    assert state.yaw == pytest.approx(np.pi / 2.0)
    assert state.linear_velocity == pytest.approx(2.0)
