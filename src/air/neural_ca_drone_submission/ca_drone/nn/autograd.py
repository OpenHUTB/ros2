"""A very small reverse-mode autodiff engine (NumPy only).

The course assignment asks for neural networks in every module, but the demo
machine has no GPU and no PyTorch.  This module therefore implements just the
subset of operations needed by the four networks in :mod:`ca_drone.models`:
dense layers, 2-D convolutions, pooling, up-sampling, concatenation and the
usual activations.  It is intentionally compact (< 450 lines) and is used both
for training and for on-board inference.
"""

from __future__ import annotations

import math
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np

DTYPE = np.float32

def _unbroadcast(grad: np.ndarray, shape: Tuple[int, ...]) -> np.ndarray:
    while grad.ndim > len(shape):
        grad = grad.sum(axis=0)
    for i, s in enumerate(shape):
        if s == 1 and grad.shape[i] != 1:
            grad = grad.sum(axis=i, keepdims=True)
    return grad


class Tensor:
    __slots__ = ("data", "grad", "requires_grad", "_backward", "_prev", "_op")

    def __init__(self, data, requires_grad: bool = False,
                 _children: Iterable["Tensor"] = (), _op: str = ""):
        self.data = np.asarray(data, dtype=DTYPE)
        self.grad: Optional[np.ndarray] = None
        self.requires_grad = bool(requires_grad)
        self._backward = lambda: None
        self._prev = tuple(_children)
        self._op = _op

    # ------------------------------------------------------------------ utils
    @property
    def shape(self):
        return self.data.shape

    def __repr__(self):
        return f"Tensor(shape={self.data.shape}, op={self._op!r})"

    def zero_grad(self):
        self.grad = None

    def backward(self, grad: Optional[np.ndarray] = None):
        if not self.requires_grad:
            return
        if grad is None:
            grad = np.ones_like(self.data)
        self.grad = np.asarray(grad, dtype=DTYPE)
        topo: List[Tensor] = []
        seen = set()

        def visit(t: Tensor):
            if id(t) in seen:
                return
            seen.add(id(t))
            for p in t._prev:
                visit(p)
            topo.append(t)

        visit(self)
        for t in reversed(topo):
            if t.grad is None:
                continue
            t._backward()

    def _acc(self, g: np.ndarray):
        g = _unbroadcast(np.asarray(g, dtype=DTYPE), self.data.shape)
        self.grad = g if self.grad is None else self.grad + g

    # ------------------------------------------------------------- arithmetic
    def __add__(self, other):
        other = other if isinstance(other, Tensor) else Tensor(other)
        out = Tensor(self.data + other.data, self.requires_grad or other.requires_grad,
                     (self, other), "add")

        def _bw():
            if self.requires_grad:
                self._acc(out.grad)
            if other.requires_grad:
                other._acc(out.grad)
        out._backward = _bw
        return out

    def __radd__(self, other):
        return self + other

    def __sub__(self, other):
        other = other if isinstance(other, Tensor) else Tensor(other)
        out = Tensor(self.data - other.data, self.requires_grad or other.requires_grad,
                     (self, other), "sub")

        def _bw():
            if self.requires_grad:
                self._acc(out.grad)
            if other.requires_grad:
                other._acc(-out.grad)
        out._backward = _bw
        return out

    def __rsub__(self, other):
        return Tensor(other) - self

    def __mul__(self, other):
        other = other if isinstance(other, Tensor) else Tensor(other)
        out = Tensor(self.data * other.data, self.requires_grad or other.requires_grad,
                     (self, other), "mul")

        def _bw():
            if self.requires_grad:
                self._acc(out.grad * other.data)
            if other.requires_grad:
                other._acc(out.grad * self.data)
        out._backward = _bw
        return out

    def __rmul__(self, other):
        return self * other

    def __truediv__(self, other):
        other = other if isinstance(other, Tensor) else Tensor(other)
        out = Tensor(self.data / other.data, self.requires_grad or other.requires_grad,
                     (self, other), "div")

        def _bw():
            if self.requires_grad:
                self._acc(out.grad / other.data)
            if other.requires_grad:
                other._acc(-out.grad * self.data / (other.data ** 2))
        out._backward = _bw
        return out

    def __neg__(self):
        return self * -1.0

    def __matmul__(self, other):
        other = other if isinstance(other, Tensor) else Tensor(other)
        out = Tensor(self.data @ other.data, self.requires_grad or other.requires_grad,
                     (self, other), "matmul")

        def _bw():
            if self.requires_grad:
                self._acc(out.grad @ np.swapaxes(other.data, -1, -2))
            if other.requires_grad:
                other._acc(np.swapaxes(self.data, -1, -2) @ out.grad)
        out._backward = _bw
        return out

    # ------------------------------------------------------------- reductions
    def sum(self, axis=None, keepdims=False):
        out = Tensor(self.data.sum(axis=axis, keepdims=keepdims),
                     self.requires_grad, (self,), "sum")

        def _bw():
            g = out.grad
            if axis is not None and not keepdims:
                g = np.expand_dims(g, axis)
            self._acc(np.broadcast_to(g, self.data.shape))
        out._backward = _bw
        return out

    def mean(self, axis=None, keepdims=False):
        n = self.data.size if axis is None else self.data.shape[axis]
        return self.sum(axis=axis, keepdims=keepdims) / float(n)

    def reshape(self, *shape):
        shape = shape[0] if len(shape) == 1 and isinstance(shape[0], (tuple, list)) else shape
        out = Tensor(self.data.reshape(shape), self.requires_grad, (self,), "reshape")

        def _bw():
            self._acc(out.grad.reshape(self.data.shape))
        out._backward = _bw
        return out

    def transpose(self, *axes):
        axes = axes[0] if len(axes) == 1 and isinstance(axes[0], (tuple, list)) else axes
        if not axes:
            axes = tuple(range(self.data.ndim - 1, -1, -1))
        out = Tensor(self.data.transpose(axes), self.requires_grad, (self,), "transpose")

        def _bw():
            self._acc(out.grad.transpose(np.argsort(axes)))
        out._backward = _bw
        return out

    # ------------------------------------------------------------ activations
    def relu(self):
        out = Tensor(np.maximum(self.data, 0.0), self.requires_grad, (self,), "relu")

        def _bw():
            self._acc(out.grad * (self.data > 0.0))
        out._backward = _bw
        return out

    def sigmoid(self):
        s = 1.0 / (1.0 + np.exp(-np.clip(self.data, -60, 60)))
        out = Tensor(s, self.requires_grad, (self,), "sigmoid")

        def _bw():
            self._acc(out.grad * s * (1.0 - s))
        out._backward = _bw
        return out

    def tanh(self):
        t = np.tanh(self.data)
        out = Tensor(t, self.requires_grad, (self,), "tanh")

        def _bw():
            self._acc(out.grad * (1.0 - t * t))
        out._backward = _bw
        return out

    def exp(self):
        e = np.exp(np.clip(self.data, -60, 60))
        out = Tensor(e, self.requires_grad, (self,), "exp")

        def _bw():
            self._acc(out.grad * e)
        out._backward = _bw
        return out

    def log(self):
        out = Tensor(np.log(np.clip(self.data, 1e-12, None)), self.requires_grad, (self,), "log")

        def _bw():
            self._acc(out.grad / np.clip(self.data, 1e-12, None))
        out._backward = _bw
        return out


# --------------------------------------------------------------------- ops
def concat(tensors: Sequence[Tensor], axis: int = 1) -> Tensor:
    out = Tensor(np.concatenate([t.data for t in tensors], axis=axis),
                 any(t.requires_grad for t in tensors), tuple(tensors), "concat")
    sizes = [t.data.shape[axis] for t in tensors]

    def _bw():
        start = 0
        for t, s in zip(tensors, sizes):
            if t.requires_grad:
                sl = [slice(None)] * out.grad.ndim
                sl[axis] = slice(start, start + s)
                t._acc(out.grad[tuple(sl)])
            start += s
    out._backward = _bw
    return out


def conv2d(x: Tensor, w: Tensor, b: Optional[Tensor], stride: int = 1,
           padding: int = 0) -> Tensor:
    """NCHW convolution via im2col."""
    B, C, H, W = x.data.shape
    Cout, Cin, kh, kw = w.data.shape
    assert C == Cin
    xp = np.pad(x.data, ((0, 0), (0, 0), (padding, padding), (padding, padding))) \
        if padding else x.data
    Hp, Wp = xp.shape[2], xp.shape[3]
    Ho = (Hp - kh) // stride + 1
    Wo = (Wp - kw) // stride + 1
    cols = np.empty((B, C, kh, kw, Ho, Wo), dtype=DTYPE)
    for i in range(kh):
        for j in range(kw):
            cols[:, :, i, j] = xp[:, :, i:i + Ho * stride:stride, j:j + Wo * stride:stride]
    cols = cols.reshape(B, C * kh * kw, Ho * Wo)
    w2 = w.data.reshape(Cout, -1)
    out_data = np.einsum("ok,bkn->bon", w2, cols).reshape(B, Cout, Ho, Wo)
    if b is not None:
        out_data = out_data + b.data.reshape(1, Cout, 1, 1)
    out = Tensor(out_data, any(t.requires_grad for t in (x, w, b) if t is not None),
                 tuple(t for t in (x, w, b) if t is not None), "conv2d")

    def _bw():
        g = out.grad.reshape(B, Cout, Ho * Wo)
        if w.requires_grad:
            gw = np.einsum("bon,bkn->ok", g, cols).reshape(w.data.shape)
            w._acc(gw)
        if b is not None and b.requires_grad:
            b._acc(g.sum(axis=(0, 2)))
        if x.requires_grad:
            gcols = np.einsum("ok,bon->bkn", w2, g).reshape(B, C, kh, kw, Ho, Wo)
            gxp = np.zeros_like(xp)
            for i in range(kh):
                for j in range(kw):
                    gxp[:, :, i:i + Ho * stride:stride, j:j + Wo * stride:stride] += gcols[:, :, i, j]
            if padding:
                gx = gxp[:, :, padding:padding + H, padding:padding + W]
            else:
                gx = gxp
            x._acc(gx)
    out._backward = _bw
    return out


def maxpool2d(x: Tensor, k: int = 2) -> Tensor:
    B, C, H, W = x.data.shape
    Ho, Wo = H // k, W // k
    xr = x.data.reshape(B, C, Ho, k, Wo, k)
    out_data = xr.max(axis=(3, 5))
    out = Tensor(out_data, x.requires_grad, (x,), "maxpool")

    def _bw():
        mask = (xr == out_data[:, :, :, None, :, None])
        g = out.grad[:, :, :, None, :, None] * mask
        # divide gradient among tied maxima to keep the sum correct
        cnt = mask.sum(axis=(3, 5), keepdims=True)
        x._acc((g / np.maximum(cnt, 1)).reshape(B, C, H, W))
    out._backward = _bw
    return out


def upsample2d(x: Tensor, size: Optional[Tuple[int, int]] = None) -> Tensor:
    B, C, H, W = x.data.shape
    Ho, Wo = size if size is not None else (H * 2, W * 2)
    ry, rx = Ho // H, Wo // W
    out_data = np.repeat(np.repeat(x.data, ry, axis=2), rx, axis=3)
    out_data = out_data[:, :, :Ho, :Wo]
    out = Tensor(out_data, x.requires_grad, (x,), "upsample")

    def _bw():
        g = out.grad[:, :, :Ho, :Wo]
        if (ry, rx) != (1, 1):
            g = g.reshape(B, C, Ho // ry, ry, Wo // rx, rx).sum(axis=(3, 5))
        x._acc(g)
    out._backward = _bw
    return out


def avgpool2d(x: Tensor, k: int = 2) -> Tensor:
    B, C, H, W = x.data.shape
    Ho, Wo = H // k, W // k
    xr = x.data.reshape(B, C, Ho, k, Wo, k)
    out_data = xr.mean(axis=(3, 5))
    out = Tensor(out_data, x.requires_grad, (x,), "avgpool")

    def _bw():
        g = np.repeat(np.repeat(out.grad / (k * k), k, axis=2), k, axis=3)
        x._acc(g.reshape(B, C, H, W))
    out._backward = _bw
    return out


# --------------------------------------------------------------------- losses
def bce_with_logits(logits: Tensor, target: np.ndarray, weight: Optional[np.ndarray] = None,
                    reduce: bool = False) -> Tensor:
    t = np.asarray(target, dtype=DTYPE)
    z = logits.data
    # stable: max(z,0) - z*t + log(1+exp(-|z|))
    loss = np.maximum(z, 0) - z * t + np.log1p(np.exp(-np.abs(z)))
    if weight is not None:
        loss = loss * weight
    n = float(loss.size) if reduce else 1.0
    if reduce:
        loss = np.asarray(loss.mean(), dtype=DTYPE)
    out = Tensor(loss, logits.requires_grad, (logits,), "bce")

    def _bw():
        p = 1.0 / (1.0 + np.exp(-np.clip(z, -60, 60)))
        g = p - t
        if weight is not None:
            g = g * weight
        logits._acc(out.grad * g / n)
    out._backward = _bw
    return out


def mse(pred: Tensor, target: np.ndarray, reduce: bool = False) -> Tensor:
    t = np.asarray(target, dtype=DTYPE)
    diff = pred.data - t
    loss = diff ** 2
    if reduce:
        loss = np.asarray(loss.mean(), dtype=DTYPE)
    out = Tensor(loss, pred.requires_grad, (pred,), "mse")

    def _bw():
        g = 2.0 * diff
        if reduce:
            g = g / diff.size
        pred._acc(out.grad * g)
    out._backward = _bw
    return out


# ------------------------------------------------------------------ parameter init
def he_normal(shape: Tuple[int, ...], gain: float = 2.0, rng=None) -> np.ndarray:
    rng = rng or np.random.default_rng(0)
    fan_in = int(np.prod(shape[1:])) if len(shape) > 1 else shape[0]
    std = math.sqrt(gain / max(1, fan_in))
    return rng.normal(0.0, std, size=shape)


def xavier(shape: Tuple[int, ...], rng=None) -> np.ndarray:
    rng = rng or np.random.default_rng(0)
    if len(shape) == 2:
        fan_in, fan_out = shape
    else:
        fan_in, fan_out = int(np.prod(shape[1:])), shape[0]
    lim = math.sqrt(6.0 / (fan_in + fan_out))
    return rng.uniform(-lim, lim, size=shape)
