#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""三次 Hermite 样条插值与解析导数测试。"""

import numpy as np

from biped_cubic_spline_gait_switch.spline import (
    hermite_scalar, hermite_scalar_derivative, hermite_vector,
    hermite_vector_derivative)


def test_hermite_endpoints_and_slopes():
    assert abs(hermite_scalar(0.0, 0.0, 1.0, 0.5, 1.0, 3.0, -0.2) - 1.0) < 1e-9
    assert abs(hermite_scalar(1.0, 0.0, 1.0, 0.5, 1.0, 3.0, -0.2) - 3.0) < 1e-9
    assert abs(hermite_scalar_derivative(0.0, 0.0, 1.0, 0.5, 1.0, 3.0, -0.2)
               - 0.5) < 1e-9
    assert abs(hermite_scalar_derivative(1.0, 0.0, 1.0, 0.5, 1.0, 3.0, -0.2)
               - (-0.2)) < 1e-9


def test_hermite_derivative_matches_finite_difference():
    h = 1e-6
    for t in np.linspace(0.01, 0.99, 20):
        fd = (hermite_scalar(t + h, 0.0, 1.0, 0.5, 1.0, 3.0, -0.2)
              - hermite_scalar(t - h, 0.0, 1.0, 0.5, 1.0, 3.0, -0.2)) / (2 * h)
        got = hermite_scalar_derivative(t, 0.0, 1.0, 0.5, 1.0, 3.0, -0.2)
        assert abs(got - fd) < 1e-5


def test_hermite_vector_matches_scalar():
    q0 = np.array([0.0, 1.0, 2.0])
    dq0 = np.array([0.1, -0.2, 0.3])
    q1 = np.array([1.0, 0.5, 0.0])
    dq1 = np.array([0.0, 0.1, -0.1])
    got = hermite_vector(0.5, 0.0, q0, dq0, 1.0, q1, dq1)
    expect = np.array([hermite_scalar(0.5, 0.0, q0[i], dq0[i], 1.0, q1[i], dq1[i])
                       for i in range(3)])
    assert np.allclose(got, expect)
    got_d = hermite_vector_derivative(0.5, 0.0, q0, dq0, 1.0, q1, dq1)
    expect_d = np.array([
        hermite_scalar_derivative(0.5, 0.0, q0[i], dq0[i], 1.0, q1[i], dq1[i])
        for i in range(3)])
    assert np.allclose(got_d, expect_d)
