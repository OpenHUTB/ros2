#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
三次多项式关节空间插值器
特点：起止速度均为 0，运动平滑无冲击。
"""
import numpy as np


class CubicInterpolator:
    def __init__(self, q0, qf, T, dt):
        """
        q0: 起始关节角 (nj,)
        qf: 目标关节角 (nj,)
        T : 运动总时长 (s)
        dt: 采样间隔 (s)
        """
        q0 = np.asarray(q0, dtype=float)
        qf = np.asarray(qf, dtype=float)
        n = max(2, int(round(T / dt)))
        t = np.linspace(0.0, T, n)
        nj = q0.shape[0]
        traj = np.zeros((n, nj))
        for i in range(nj):
            d = qf[i] - q0[i]
            a0 = q0[i]
            a1 = 0.0
            a2 = 3.0 * d / (T * T)
            a3 = -2.0 * d / (T * T * T)
            traj[:, i] = a0 + a1 * t + a2 * t * t + a3 * t * t * t
        self.qs = traj
        self.n = n

    def __len__(self):
        return self.n

    def __iter__(self):
        return iter(self.qs)
