#!/usr/bin/env python3
"""二连杆机械臂笛卡尔阻抗控制器。"""
import numpy as np


class ImpedanceController:
    """F = Kd*(x_des - x) + Bd*(xdot_des - xdot)，tau = J^T * F"""

    def __init__(self, L1=0.3, L2=0.28, Kp=None, Bd=None):
        self.L1 = L1
        self.L2 = L2
        self.Kp = np.diag(Kp if Kp is not None else [80.0, 50.0])
        self.Bd = np.diag(Bd if Bd is not None else [8.0, 5.0])

    def fk(self, q):
        q1, q2 = q[0], q[1]
        x = self.L1 * np.cos(q1) + self.L2 * np.cos(q1 + q2)
        y = self.L1 * np.sin(q1) + self.L2 * np.sin(q1 + q2)
        return np.array([x, y])

    def jac(self, q):
        q1, q2 = q[0], q[1]
        J = np.zeros((2, 2))
        J[0, 0] = -self.L1 * np.sin(q1) - self.L2 * np.sin(q1 + q2)
        J[0, 1] = -self.L2 * np.sin(q1 + q2)
        J[1, 0] = self.L1 * np.cos(q1) + self.L2 * np.cos(q1 + q2)
        J[1, 1] = self.L2 * np.cos(q1 + q2)
        return J

    def compute_torque(self, q, qdot, x_des, xdot_des=None):
        x = self.fk(q)
        J = self.jac(q)
        xdot = J @ qdot
        if xdot_des is None:
            xdot_des = np.zeros(2)
        F = self.Kp @ (x_des - x) + self.Bd @ (xdot_des - xdot)
        tau = J.T @ F
        return tau, F, x
