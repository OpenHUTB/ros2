# -*- coding: utf-8 -*-
"""几何间距与候选外推的单测。"""
import numpy as np
import pytest

from uav_dynamic_safety_guard.core import geometry as G
from uav_dynamic_safety_guard.core.field import Mover


def test_box_sdf_outside_and_inside():
    half = np.array([0.5, 0.5, 0.5])
    center = np.array([0.0, 0.0, 0.0])
    outside = G.box_sdf(np.array([[1.5, 0.0, 0.0]]), center, half)[0]
    inside = G.box_sdf(np.array([[0.1, 0.0, 0.0]]), center, half)[0]
    assert outside == pytest.approx(1.0)
    assert inside == pytest.approx(-0.4)


def test_cylinder_sdf_top_and_side():
    pos = np.array([0.0, 0.0, 1.0])
    side = G.cylinder_sdf(np.array([[1.0, 0.0, 1.0]]), pos, 0.25, 1.0)[0]
    top = G.cylinder_sdf(np.array([[0.0, 0.0, 2.5]]), pos, 0.25, 1.0)[0]
    assert side == pytest.approx(0.75)
    assert top == pytest.approx(0.5)


def test_sphere_sdf_matches_norm():
    d = G.sphere_sdf(np.array([[3.0, 4.0, 0.0]]), np.zeros(3), 0.5)[0]
    assert d == pytest.approx(4.5)


def test_clearance_subtracts_drone_radius_and_floor():
    prims = [("box", np.array([10.0, 0.0, 0.5]), np.array([0.5, 0.5, 0.5]))]
    assert G.clearance_at(prims, [0.0, 0.0, 1.0]) == pytest.approx(
        1.0 - G.DRONE_RADIUS)


def test_batch_and_scalar_paths_agree():
    rng = np.random.default_rng(7)
    prims = [("box", np.array([1.0, 0.5, 0.6]), np.array([0.3, 0.4, 0.6])),
             ("cylinder", np.array([-1.2, 0.2, 0.5]), np.array([0.25, 0.5])),
             ("sphere", np.array([0.3, -1.0, 1.2]), np.array([0.35]))]
    pts = rng.uniform(-2.5, 2.5, (40, 3))
    a = G.clearance_points(prims, pts)
    b = G.clearance_points_batch(G.stack_batch(prims), pts)
    assert np.allclose(a, b, atol=1e-9)


def test_merge_batch_keeps_counts():
    a = G.stack_batch([("box", np.zeros(3), np.ones(3))])
    b = G.stack_batch([("cylinder", np.zeros(3), np.array([0.2, 0.5]))])
    m = G.merge_batch(a, b)
    assert len(m["box_pos"]) == 1 and len(m["cyl_pos"]) == 1


def test_rollout_reaches_further_with_higher_speed():
    wall = G.stack_batch([("box", np.array([1.6, 0.0, 1.0]),
                          np.array([0.2, 6.0, 1.0]))])
    p = np.array([0.0, 0.0, 1.6])
    vs = np.zeros((2, 3))
    us = np.array([[0.0, 0.0, 0.0], [3.0, 0.0, 0.0]])
    q = G.rollout_clearance(lambda t: wall, p, vs, 0.0, us, horizon=0.6,
                            acc_max=4.0)
    assert q[1] < q[0]


def test_rollout_tracks_moving_obstacle_time():
    mover = Mover("m", (0.0, 0.0), 1.0, np.pi, 0.0, (0.2, 0.9))
    # t=0 时障碍在 (1,0)，t=1 时在 (-1,0)
    batch0 = G.stack_batch([mover.prim_at(0.0)])
    batch1 = G.stack_batch([mover.prim_at(1.0)])
    d0 = G.clearance_points_batch(batch0, np.array([[1.0, 0.0, 1.6]]))[0]
    d1 = G.clearance_points_batch(batch1, np.array([[1.0, 0.0, 1.6]]))[0]
    assert d0 < d1


def test_unknown_primitive_raises():
    with pytest.raises(ValueError):
        G.primitive_distance(np.zeros((1, 3)), ("cone", np.zeros(3), np.ones(3)))
