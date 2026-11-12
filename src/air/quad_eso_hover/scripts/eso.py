"""四旋翼高度通道 PD + 扩张状态观测器（ESO / ADRC 入门）。

被控对象（高度通道，质量 m，重力 g，未知扰动 d(t)）：
    m * z_ddot = u - m*g + d(t)

记控制增益 b0 = 1/m，把"除输入外的一切"（重力 + 未知扰动）合并为总扰动 f：
    z_ddot = f + b0 * u ,     f = -g + d/m

扩张状态观测器（线性三阶 LESO）把 f 当作"扩张状态"在线估计：
    e   = z1_hat - y
    z1_hat += dt*( z2_hat - beta1*e )
    z2_hat += dt*( z3_hat + b0*u - beta2*e )
    z3_hat += dt*( -beta3*e )
取 beta1=3*w0, beta2=3*w0^2, beta3=w0^3（观测器三重极点 -w0）。

ADRC 控制律：用 z3_hat 前馈补偿总扰动，再对"纯净积分链"做 PD：
    u0 = kp*(z_ref - z1_hat) + kd*(0 - z2_hat)
    u  = (u0 - z3_hat) / b0

对照基线（PD + 已知重力前馈，不估计扰动）：
    u = m*( g + kp*(z_ref - z) + kd*(0 - z_dot) )
在常值扰动 d 下稳态高度误差 e_ss = -d/(m*kp)，无法消除。
"""

import math


class Plant:
    def __init__(self, m=1.0, g=9.81):
        self.m = m
        self.g = g
        self.b0 = 1.0 / m

    def zdd(self, u, d):
        # z_ddot = -g + d/m + u/m
        return -self.g + d / self.m + u / self.m

    def total_disturbance(self, d):
        """总扰动 f = -g + d/m（ESO 应估计到它）。"""
        return -self.g + d / self.m


def _rk4_z(plant, z, zd, u, d, dt):
    k1z = zd
    k1v = plant.zdd(u, d)
    k2z = zd + 0.5 * dt * k1v
    k2v = plant.zdd(u, d)
    k3z = zd + 0.5 * dt * k2v
    k3v = plant.zdd(u, d)
    k4z = zd + dt * k3v
    k4v = plant.zdd(u, d)
    z_new = z + dt / 6.0 * (k1z + 2 * k2z + 2 * k3z + k4z)
    zd_new = zd + dt / 6.0 * (k1v + 2 * k2v + 2 * k3v + k4v)
    return z_new, zd_new


def simulate_pd(plant, ref_fn, d_fn, kp, kd, T, dt, z0=0.0):
    """PD + 已知重力前馈（无扰动估计）。返回 t, z, zd, u。"""
    n = int(T / dt)
    z, zd = z0, 0.0
    ts, zs, zds, us = [0.0], [z], [zd], []
    for k in range(n):
        t = k * dt
        ref = ref_fn(t)
        d = d_fn(t)
        u = plant.m * (plant.g + kp * (ref - z) + kd * (0.0 - zd))
        us.append(u)
        z, zd = _rk4_z(plant, z, zd, u, d, dt)
        ts.append(t + dt)
        zs.append(z)
        zds.append(zd)
    return ts, zs, zds, us


def simulate_eso(plant, ref_fn, d_fn, kp, kd, w0, T, dt, z0=0.0):
    """PD + ESO 前馈补偿（ADRC）。返回 t, z, zd, u, z3_hat, f_true。"""
    n = int(T / dt)
    z, zd = z0, 0.0
    b0 = plant.b0
    beta1 = 3.0 * w0
    beta2 = 3.0 * w0 * w0
    beta3 = w0 * w0 * w0
    z1, z2, z3 = z, 0.0, 0.0   # ESO 估计初值
    ts, zs, zds, us, z3s, fts = [0.0], [z], [zd], [], [z3], [plant.total_disturbance(d_fn(0.0))]
    for k in range(n):
        t = k * dt
        ref = ref_fn(t)
        d = d_fn(t)
        y = z  # 测量高度
        # ADRC 控制（用当前 ESO 估计）
        u0 = kp * (ref - z1) + kd * (0.0 - z2)
        u = (u0 - z3) / b0
        us.append(u)
        # 推进一步被控对象
        z, zd = _rk4_z(plant, z, zd, u, d, dt)
        # 更新 ESO
        e = z1 - y
        z1 = z1 + dt * (z2 - beta1 * e)
        z2 = z2 + dt * (z3 + b0 * u - beta2 * e)
        z3 = z3 + dt * (-beta3 * e)
        ts.append(t + dt)
        zs.append(z)
        zds.append(zd)
        z3s.append(z3)
        fts.append(plant.total_disturbance(d))
    return ts, zs, zds, us, z3s, fts
