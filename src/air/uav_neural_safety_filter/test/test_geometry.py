import numpy as np
import pytest

from uav_mujoco_sim.core.scenes import Scene, SceneObject
from uav_neural_safety_filter.core.geometry import (DRONE_RADIUS, clearance_at,
                                                    rollout_clearance_many,
                                                    swept_clearance)


@pytest.fixture
def scene():
    return Scene(name="t", objects=[
        SceneObject("obstacle_box0", "box", (2.0, 0.0, 1.0), (0.5, 0.5, 1.0)),
        SceneObject("obstacle_cyl1", "cylinder", (0.0, 2.0, 1.0), (0.3, 1.0)),
        SceneObject("obstacle_sph2", "sphere", (-2.0, 0.0, 1.0), (0.4,)),
    ])


def test_box_sdf(scene):
    assert abs(clearance_at(scene, [1.0, 0.0, 1.0]) - (0.5 - DRONE_RADIUS)) < 1e-9


def test_cylinder_sdf(scene):
    assert abs(clearance_at(scene, [0.0, 1.3, 1.0]) - (0.4 - DRONE_RADIUS)) < 1e-9


def test_sphere_sdf(scene):
    assert abs(clearance_at(scene, [-1.2, 0.0, 1.0]) - (0.4 - DRONE_RADIUS)) < 1e-9


def test_ground_sdf(scene):
    assert abs(clearance_at(scene, [5.0, 5.0, 0.5]) - (0.5 - DRONE_RADIUS)) < 1e-9


def test_swept_decreases_with_speed(scene):
    a = swept_clearance(scene, [0.0, 0.0, 1.0], [0.0, 0.0, 0.0])
    b = swept_clearance(scene, [0.0, 0.0, 1.0], [2.0, 0.0, 0.0])
    assert b < a


def test_rollout_matches_braking(scene):
    q0 = rollout_clearance_many(scene, [0.0, 0.0, 1.0], np.zeros((1, 3)), 0.0,
                                np.zeros((1, 3)))[0]
    q1 = rollout_clearance_many(scene, [0.0, 0.0, 1.0], np.zeros((1, 3)), 0.0,
                                np.array([[2.0, 0.0, 0.0]]))[0]
    assert q0 > q1
