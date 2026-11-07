"""倒立摆 LQR 模块的单元测试（仅标准库 unittest）。"""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

import cartpole as cp
import minilinalg as la


class TestLinearAlgebra(unittest.TestCase):
    def test_inverse(self):
        A = [[4.0, 7.0], [2.0, 6.0]]
        Ainv = la.inverse(A)
        prod = la.matmul(A, Ainv)
        self.assertAlmostEqual(prod[0][0], 1.0, places=9)
        self.assertAlmostEqual(prod[0][1], 0.0, places=9)
        self.assertAlmostEqual(prod[1][0], 0.0, places=9)
        self.assertAlmostEqual(prod[1][1], 1.0, places=9)

    def test_rank(self):
        self.assertEqual(la.rank([[1, 2], [2, 4]]), 1)
        self.assertEqual(la.rank([[1, 0], [0, 1]]), 2)


class TestCartPole(unittest.TestCase):
    def setUp(self):
        self.model, self.K, self.Q, self.R, self.dt = cp.default_problem(0.01)

    def test_controllable(self):
        ok, r = self.model.is_controllable()
        self.assertTrue(ok)
        self.assertEqual(r, 4)

    def test_upright_is_unstable(self):
        # 线性化后摆角方程系数为正 => 竖直向上是不稳定平衡点
        A, B = self.model.linearize()
        self.assertGreater(A[3][2], 0.0)

    def test_lqr_stabilizes(self):
        x0 = [0.0, 0.0, 0.1, 0.0]
        ts, st, f = cp.simulate(self.model, self.K, x0, T=8.0, dt=self.dt, control=True)
        self.assertLess(abs(st[-1][2]), 0.01)   # 摆角收敛
        self.assertLess(abs(st[-1][0]), 0.05)   # 位置回中

    def test_passive_diverges(self):
        x0 = [0.0, 0.0, 0.1, 0.0]
        ts, st, f = cp.simulate(self.model, self.K, x0, T=8.0, dt=self.dt, control=False)
        self.assertGreater(abs(st[-1][2]), 0.3)  # 无控时摆倒下

    def test_dare_residual(self):
        A, B = self.model.linearize()
        Ad, Bd = self.model.discretize(A, B, self.dt)
        P = la.solve_dare(Ad, Bd, self.Q, self.R)
        At = la.transpose(Ad)
        Bt = la.transpose(Bd)
        # P = Q + At P Ad - At P Bd (R + Bt P Bd)^-1 Bt P Ad
        AtP = la.matmul(At, P)
        BtP = la.matmul(Bt, P)
        BtPB = la.matmul(BtP, Bd)
        S = la.matadd(self.R, BtPB)
        Sinv = la.inverse(S)
        BtPA = la.matmul(BtP, Ad)
        AtPB = la.matmul(AtP, Bd)
        term = la.matmul(la.matmul(AtPB, Sinv), BtPA)
        rhs = la.matsub(la.matadd(self.Q, la.matmul(AtP, Ad)), term)
        for i in range(4):
            for j in range(4):
                self.assertAlmostEqual(P[i][j], rhs[i][j], places=6)


if __name__ == "__main__":
    unittest.main()
