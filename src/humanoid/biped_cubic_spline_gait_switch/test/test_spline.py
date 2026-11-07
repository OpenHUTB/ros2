#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""三次样条插值测试。"""

import numpy as np

from biped_cubic_spline_gait_switch.spline import (
    hermite_scalar, hermite_vector, natural_cubic_spline)


def test_natural_spline_interpolates_knots():
    x = [0.0, 0.4, 0.8, 1.2, 1.6]
    y = [0.0, 0.6, 0.2, 0.9, 0.4]
    fn = natural_cubic_spline(x, y)
    for xi, yi in zip(x, y):
        assert abs(fn(xi) - yi) < 1e-9


def test_natural_spline_is_c2_continuous():
    x = np.linspace(0.0, 2.0, 7)
    y = np.sin(x)
    fn = natural_cubic_spline(x, y)
    h = 1e-5
    # 在内部节点两侧比较二阶导
    for xi in x[1:-1]:
        left = (fn(xi) - 2 * fn(xi - h) + fn(xi - 2 * h)) / (h * h)
        right = (fn(xi + 2 * h) - 2 * fn(xi + h) + fn(xi)) / (h * h)
        assert abs(left - right) < 0.05


def test_hermite_endpoints_and_slopes():
    assert abs(hermite_scalar(0.0, 0.0, 1.0, 0.5, 1.0, 3.0, -0.2) - 1.0) < 1e-9
    assert abs(hermite_scalar(1.0, 0.0, 1.0, 0.5, 1.0, 3.0, -0.2) - 3.0) < 1e-9
    h = 1e-4
    slope0 = (hermite_scalar(h, 0.0, 1.0, 0.5, 1.0, 3.0, -0.2) - 1.0) / h
    slope1 = (3.0 - hermite_scalar(1.0 - h, 0.0, 1.0, 0.5, 1.0, 3.0, -0.2)) / h
    assert abs(slope0 - 0.5) < 1e-2
    assert abs(slope1 - (-0.2)) < 1e-2


def test_hermite_vector_matches_scalar():
    q0 = np.array([0.0, 1.0, 2.0])
    dq0 = np.array([0.1, -0.2, 0.3])
    q1 = np.array([1.0, 0.5, 0.0])
    dq1 = np.array([0.0, 0.1, -0.1])
    got = hermite_vector(0.5, 0.0, q0, dq0, 1.0, q1, dq1)
    expect = np.array([hermite_scalar(0.5, 0.0, q0[i], dq0[i], 1.0, q1[i], dq1[i])
                       for i in range(3)])
    assert np.allclose(got, expect)
