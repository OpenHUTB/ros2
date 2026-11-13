"""倒立摆（Cart-Pole）: 动力学、线性化、LQR 设计、闭环仿真。

状态 x = [p, p_dot, theta, theta_dot]
  p     : 小车水平位置 (m)
  theta : 摆杆与竖直向上方向的夹角 (rad)，正方向见下
输入 u = 作用在小车上的水平力 F (N)

采用"质点摆"模型：摆杆质量 m 集中于距转轴 l 处；小车质量 M。
非线性方程（拉格朗日推导，见文档）：
  (M+m) p_ddot + m l cosθ θ_ddot = F + m l sinθ θ_dot^2
  m l cosθ p_ddot + m l^2 θ_ddot   = m g l sinθ
在竖直向上平衡点 θ=0 处线性化得 A, B。
"""

import math
import minilinalg as la


class CartPole:
    def __init__(self, M=1.0, m=0.2, l=0.5, g=9.81, b=0.1):
        self.M = M      # 小车质量
        self.m = m      # 摆杆质量（集中于距转轴 l 处）
        self.l = l      # 转轴到摆杆质心距离
        self.g = g
        self.b = b      # 摆杆转动粘性阻尼 (N·m·s/rad)，贴近真实系统

    # ---------- 非线性动力学 ----------
    def deriv(self, state, u):
        p, p_dot, th, th_dot = state
        M, m, l, g, b = self.M, self.m, self.l, self.g, self.b
        c = math.cos(th)
        s = math.sin(th)
        # 方程组（含摆杆阻尼 -b th_dot）:
        # (M+m) pdd + m l c thdd = F + m l s th_dot^2
        # m l c pdd + m l^2 thdd = m g l s - b th_dot
        D = m * l * l * (M + m - m * c * c)   # 行列式
        rhs_th = m * g * l * s - b * th_dot
        num_p = m * l * l * (u + m * l * s * th_dot * th_dot) - m * l * c * rhs_th
        num_th = (M + m) * rhs_th - m * l * c * (u + m * l * s * th_dot * th_dot)
        p_dd = num_p / D
        th_dd = num_th / D
        return [p_dot, p_dd, th_dot, th_dd]

    # ---------- 线性化（竖直向上平衡点） ----------
    def linearize(self):
        M, m, l, g, b = self.M, self.m, self.l, self.g, self.b
        # x_ddot     = -(m g / M) theta + (b/(M l)) theta_dot + (1/M) u
        # theta_ddot = ((M+m) g /(l M)) theta - ((M+m) b/(m l^2 M)) theta_dot - (1/(l M)) u
        A = [[0.0, 1.0, 0.0, 0.0],
             [0.0, 0.0, -m * g / M, b / (M * l)],
             [0.0, 0.0, 0.0, 1.0],
             [0.0, 0.0, (M + m) * g / (l * M), -((M + m) * b) / (m * l * l * M)]]
        B = [[0.0],
             [1.0 / M],
             [0.0],
             [-1.0 / (l * M)]]
        return A, B

    # ---------- 离散化（零阶保持近似，Euler 足够小步长） ----------
    def discretize(self, A, B, dt):
        n = len(A)
        Ad = [[(A[i][j] * dt + (1.0 if i == j else 0.0)) for j in range(n)] for i in range(n)]
        Bd = [[B[i][0] * dt] for i in range(n)]
        return Ad, Bd

    # ---------- LQR 设计 ----------
    def design_lqr(self, Q, R, dt):
        A, B = self.linearize()
        Ad, Bd = self.discretize(A, B, dt)
        K, P = la.lqr_gain(Ad, Bd, Q, R)
        return K, P, A, B

    def is_controllable(self):
        A, B = self.linearize()
        C = la.controllability_matrix(A, B)
        return la.rank(C) == 4, la.rank(C)


def rk4(model, state, u, dt):
    """四阶龙格-库塔积分一步（u 在该步内视为常数）。"""
    k1 = model.deriv(state, u)
    s2 = [state[i] + 0.5 * dt * k1[i] for i in range(4)]
    k2 = model.deriv(s2, u)
    s3 = [state[i] + 0.5 * dt * k2[i] for i in range(4)]
    k3 = model.deriv(s3, u)
    s4 = [state[i] + dt * k3[i] for i in range(4)]
    k4 = model.deriv(s4, u)
    return [state[i] + dt / 6.0 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) for i in range(4)]


def simulate(model, K, x0, T, dt, control=True):
    """闭环/开环仿真，返回 t, states, forces。"""
    n = int(T / dt)
    state = list(x0)
    ts = [0.0]
    states = [state[:]]
    forces = []
    for k in range(n):
        if control:
            # u = -K x
            u = -(K[0][0] * state[0] + K[0][1] * state[1]
                  + K[0][2] * state[2] + K[0][3] * state[3])
        else:
            u = 0.0
        forces.append(u)
        state = rk4(model, state, u, dt)
        ts.append((k + 1) * dt)
        states.append(state[:])
    forces.append(0.0)
    return ts, states, forces


def default_problem(dt=0.01):
    """默认配置：返回 model, K, Q, R, dt。"""
    model = CartPole()
    # 角度权重高（不希望摆倒），力权重较小
    Q = la.zeros(4, 4)
    Q[0][0] = 1.0     # p
    Q[1][1] = 0.1     # p_dot
    Q[2][2] = 100.0   # theta  (重点)
    Q[3][3] = 1.0     # theta_dot
    R = [[1.0]]
    K, P, A, B = model.design_lqr(Q, R, dt)
    return model, K, Q, R, dt
