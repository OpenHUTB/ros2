#!/usr/bin/env python3
"""Numerical gradient checks for the hand-written NumPy layers.

The pure-NumPy implementation is the selling point of this project, so every
analytical backward pass is verified against central finite differences:

    rel_err = |g_analytic - g_numeric| / max(1e-8, |g_analytic| + |g_numeric|)

A correct float64 implementation stays below 1e-6; the assertion threshold is
1e-5 (the standard sanity bound used by deep-learning frameworks).
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

import numpy as np

from humanoid_walk_nn.nn.layers import (Conv2D, Dense, ReLU, Sigmoid, Tanh)

EPS = 1e-5
TOL = 1e-5


def _rel_err(a: np.ndarray, b: np.ndarray) -> float:
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    denom = np.maximum(1e-8, np.abs(a) + np.abs(b))
    return float(np.max(np.abs(a - b) / denom))


def _numeric(fn, arr: np.ndarray, count: int = 40) -> np.ndarray:
    """Central-difference gradient of ``sum(fn())`` w.r.t. ``arr`` (first n)."""
    flat = arr.reshape(-1)
    n = min(flat.size, count)
    out = np.zeros(n, dtype=np.float64)
    for i in range(n):
        original = flat[i]
        flat[i] = original + EPS
        plus = float(fn())
        flat[i] = original - EPS
        minus = float(fn())
        flat[i] = original
        out[i] = (plus - minus) / (2.0 * EPS)
    return out


def check_affine_layer(layer, x: np.ndarray, upstream: np.ndarray) -> None:
    y = layer.forward(x)
    dx = layer.backward(upstream)
    loss = lambda: float(np.sum(upstream * layer.forward(x)))  # noqa: E731
    errors = {"dx": _rel_err(dx.reshape(-1)[:40], _numeric(loss, x))}
    for name in ("W", "b"):
        if hasattr(layer, name):
            param = getattr(layer, name)
            grad = getattr(layer, "d" + name)
            numeric = _numeric(loss, param)
            errors[name] = _rel_err(grad.reshape(-1)[:numeric.size], numeric)
    for key, err in errors.items():
        if err > TOL:
            raise AssertionError("%s.%s gradient mismatch rel_err=%.3e"
                                 % (type(layer).__name__, key, err))
    print("  ok  %-10s dx=%.2e W/b<=%.2e"
          % (type(layer).__name__, errors["dx"], max(errors.values())))


def check_activation(act, x: np.ndarray, upstream: np.ndarray) -> None:
    act.forward(x)
    dx = act.backward(upstream)
    loss = lambda: float(np.sum(upstream * act.forward(x)))  # noqa: E731
    err = _rel_err(dx.reshape(-1)[:40], _numeric(loss, x))
    if err > TOL:
        raise AssertionError("%s activation mismatch rel_err=%.3e"
                             % (type(act).__name__, err))
    print("  ok  %-10s dx=%.2e" % (type(act).__name__, err))


def main() -> int:
    rng = np.random.default_rng(0)
    print("dense layers (analytic vs central differences)")
    for in_dim, out_dim, batch in ((7, 5, 4), (41, 3, 16)):
        layer = Dense(in_dim, out_dim, seed=1)
        x = rng.standard_normal((batch, in_dim))
        check_affine_layer(layer, x, rng.standard_normal((batch, out_dim)))

    print("activation layers")
    for act in (ReLU(), Tanh(), Sigmoid()):
        x = rng.standard_normal((6, 9))
        y = act.forward(x)
        check_activation(act, x, rng.standard_normal(y.shape))

    print("conv2d layers (im2col + stride variants)")
    for stride, pad, k in ((1, 1, 3), (2, 1, 3), (2, 0, 2)):
        layer = Conv2D(3, 4, k=k, stride=stride, pad=pad, seed=2)
        x = rng.standard_normal((2, 3, 7, 7))
        y = layer.forward(x)
        check_affine_layer(layer, x, rng.standard_normal(y.shape))

    print("all gradient checks passed (rel_err < %.0e)" % TOL)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
