#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Closed-loop simulation test for the sonar + VFH pipeline.

The loop mirrors ``sonar_vfh_node`` but without ROS: the same sonar model, the
same histogram and steering code, a kinematic integrator instead of the node
timer.  It verifies that the controller actually reaches a goal behind blocks
without entering one.

Run with: python3 test/test_closed_loop.py
"""
import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from underwater_sonar_mapping.sonar import ScanningSonar           # noqa: E402
from underwater_sonar_mapping.terrain import (BoxObstacle, Seabed,  # noqa: E402
                                              World)
from underwater_sonar_mapping.vfh import (angle_difference_deg,    # noqa: E402
                                          clearance_along,
                                          polar_histogram,
                                          select_direction,
                                          smooth_histogram, speed_scale)

DEPTH = 6.0
INFLATION = 1.1
MAX_RANGE = 30.0
SECTORS = 72
V_MAX = 1.2
DT = 0.1


def build_scene(boxes=True):
    """Three blocks on an undulating bed, matching the demo scenario."""
    obstacles = []
    if boxes:
        obstacles = [BoxObstacle(6.0, 0.0, 1.5, 1.5, 8.0),
                     BoxObstacle(12.0, 4.5, 1.2, 1.2, 7.5),
                     BoxObstacle(14.0, -4.0, 1.2, 1.2, 8.0)]
    return World(Seabed(), obstacles), obstacles


def simulate(goal, start=(0.0, 0.0), steps=400, tolerance=0.8, boxes=True, seed=7):
    """Run the avoidance loop and return (reached, step_count, trace, obstacles)."""
    world, obstacles = build_scene(boxes)
    sonar = ScanningSonar(beams=SECTORS, fov_deg=360.0, max_range=MAX_RANGE)
    rng = np.random.default_rng(seed)
    x, y, yaw = float(start[0]), float(start[1]), 0.0
    previous = None
    trace = []

    for step in range(steps):
        bearings, ranges = sonar.scan(world, x, y, DEPTH, yaw, rng)
        hist = smooth_histogram(polar_histogram(
            bearings, ranges, sectors=SECTORS, max_range=MAX_RANGE))
        world_bearing = math.degrees(math.atan2(goal[1] - y, goal[0] - x))
        steering = select_direction(
            hist, angle_difference_deg(world_bearing, math.degrees(yaw)),
            previous_deg=previous)

        if steering.blocked:
            vx, wz = 0.0, 0.8
            previous = None
        else:
            error = math.radians(steering.angle_deg)
            wz = max(-0.8, min(0.8, 1.5 * error))
            clearance = clearance_along(bearings, ranges, steering.angle_deg,
                                        max_range=MAX_RANGE) - INFLATION
            vx = V_MAX * speed_scale(clearance) * max(0.0, math.cos(error))
            previous = steering.angle_deg

        yaw = math.atan2(math.sin(yaw + wz * DT), math.cos(yaw + wz * DT))
        x += vx * math.cos(yaw) * DT
        y += vx * math.sin(yaw) * DT
        trace.append((x, y, yaw, vx, wz))

        if math.hypot(goal[0] - x, goal[1] - y) <= tolerance:
            return True, step + 1, trace, obstacles
    return False, steps, trace, obstacles


class ClosedLoopTests(unittest.TestCase):
    def test_reaches_goal_through_the_block_field(self):
        reached, steps, _, _ = simulate(goal=(20.0, 0.0))
        self.assertTrue(reached, f'did not reach the goal within {steps} steps')
        self.assertLess(steps, 400)

    def test_never_enters_an_obstacle(self):
        _, _, trace, obstacles = simulate(goal=(20.0, 0.0))
        for x, y, *_ in trace:
            for box in obstacles:
                self.assertFalse(box.contains(x, y),
                                 f'vehicle entered block at ({x:.2f}, {y:.2f})')

    def test_keeps_a_safety_distance_to_blocks(self):
        _, _, trace, obstacles = simulate(goal=(20.0, 0.0))
        for x, y, *_ in trace:
            for box in obstacles:
                gap_x = abs(x - box.cx) - box.half_x
                gap_y = abs(y - box.cy) - box.half_y
                self.assertGreater(max(gap_x, gap_y), 0.0)

    def test_run_is_deterministic(self):
        first = simulate(goal=(20.0, 0.0))
        second = simulate(goal=(20.0, 0.0))
        self.assertEqual(first[1], second[1])
        np.testing.assert_allclose(np.array(first[2]), np.array(second[2]))

    def test_open_water_goes_almost_straight(self):
        reached, steps, trace, _ = simulate(goal=(20.0, 0.0), boxes=False)
        self.assertTrue(reached)
        self.assertLess(max(abs(point[1]) for point in trace), 2.0)

    def test_vehicle_stays_inside_the_scene(self):
        _, _, trace, _ = simulate(goal=(20.0, 0.0))
        for x, y, *_ in trace:
            self.assertLess(abs(x), 40.0)
            self.assertLess(abs(y), 40.0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
