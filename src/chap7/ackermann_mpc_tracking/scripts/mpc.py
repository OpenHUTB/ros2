"""阿克曼小车的运动学模型与线性 MPC（模型预测控制）。

被控对象（运动学自行车模型，状态量为车体后轴中心）

    x' = v cos(psi)
    y' = v sin(psi)
    psi' = v tan(delta) / L

其中 v 为纵向速度、delta 为前轮转角、L 为轴距。为了做跟踪，把状态换成
相对参考路径的误差 e = [e_y, e_psi]（横向误差、航向误差）：

    e_y'   = v sin(e_psi)              ~ v * e_psi
    e_psi' = v * tan(delta)/L - v*kappa_ref

在参考转角 delta_ref = atan(L*kappa_ref) 处线性化，并取实际转角与参考之差
u = delta - delta_ref，误差动力学就变成线性时不变：

    e' = A e + B u,   A = [[0, v], [0, 0]],   B = [[0], [v/L]]

MPC 在长度为 N 的预测时域上最小化

    J = sum_{k=1..N} e_k' Q e_k
        + sum_{k=0..N-1} ( R u_k^2 + R_rate (u_k - u_{k-1})^2 )

并满足前轮转角限幅 |delta_k| <= delta_max。因为只有箱式约束，QP 用带
FISTA 加速的投影梯度求解——不需要引入 cvxpy / numpy，几百行标准库就能
在毫秒级解完。

矩阵一律用 list[list[float]]，规模 N <= 30，够教学用。
"""

import math


# ---------------------------------------------------------------------------
# 极简矩阵工具（只覆盖本模块需要的几种运算）
# ---------------------------------------------------------------------------

def mat_zeros(n, m):
    return [[0.0] * m for _ in range(n)]


def mat_eye(n, scale=1.0):
    out = mat_zeros(n, n)
    for i in range(n):
        out[i][i] = scale
    return out


def mat_mul(a, b):
    n, k, m = len(a), len(b), len(b[0])
    out = [[0.0] * m for _ in range(n)]
    for i in range(n):
        ai = a[i]
        oi = out[i]
        for t in range(k):
            v = ai[t]
            if v == 0.0:
                continue
            bt = b[t]
            for j in range(m):
                oi[j] += v * bt[j]
    return out


def mat_vec(a, x):
    return [sum(a[i][j] * x[j] for j in range(len(x))) for i in range(len(a))]


def mat_add(a, b):
    return [[a[i][j] + b[i][j] for j in range(len(a[0]))] for i in range(len(a))]


def mat_T(a):
    return [[a[i][j] for i in range(len(a))] for j in range(len(a[0]))]


def diag(vals):
    n = len(vals)
    out = mat_zeros(n, n)
    for i, v in enumerate(vals):
        out[i][i] = v
    return out


# ---------------------------------------------------------------------------
# 车辆模型
# ---------------------------------------------------------------------------

class KinematicBicycle(object):
    """运动学自行车模型（后轴中心为参考点）。"""

    def __init__(self, wheelbase=2.5, delta_max=0.6):
        self.L = float(wheelbase)
        self.delta_max = float(delta_max)

    def step(self, state, v, delta, dt):
        """四阶 Runge-Kutta 积分一步。"""
        x, y, psi = state
        delta = max(-self.delta_max, min(self.delta_max, delta))

        def f(s):
            _x, _y, _psi = s
            return (v * math.cos(_psi), v * math.sin(_psi),
                    v * math.tan(delta) / self.L)

        k1 = f((x, y, psi))
        k2 = f((x + 0.5 * dt * k1[0], y + 0.5 * dt * k1[1], psi + 0.5 * dt * k1[2]))
        k3 = f((x + 0.5 * dt * k2[0], y + 0.5 * dt * k2[1], psi + 0.5 * dt * k2[2]))
        k4 = f((x + dt * k3[0], y + dt * k3[1], psi + dt * k3[2]))
        return (x + dt / 6.0 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]),
                y + dt / 6.0 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]),
                psi + dt / 6.0 * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2]))

    def delta_of_curvature(self, kappa):
        """曲率对应的前轮转角：delta = atan(L * kappa)。"""
        return math.atan(self.L * kappa)


# ---------------------------------------------------------------------------
# 参考路径投影
# ---------------------------------------------------------------------------

def wrap_pi(a):
    w = math.fmod(a, 2.0 * math.pi)
    if w > math.pi:
        w -= 2.0 * math.pi
    elif w <= -math.pi:
        w += 2.0 * math.pi
    return w


class PathTracker(object):
    """沿 Dubins 路径追踪弧长参数 s，给出横向误差、航向误差与参考曲率。"""

    def __init__(self, path, s0=0.0, window=2.0):
        self.path = path
        self.s = float(s0)
        self.window = float(window)

    def update(self, x, y):
        """在上一时刻 s 附近搜最近点（局部搜索，避免全局 argmin 跳变）。"""
        total = self.path.length
        best_s, best_d2 = self.s, float('inf')
        s_lo = max(0.0, self.s - self.window)
        s_hi = min(total, self.s + self.window)
        n = int(min(4000, max(8, (s_hi - s_lo) / 0.02)))
        for i in range(n + 1):
            s = s_lo + (s_hi - s_lo) * i / n
            px, py, _pth, _k = self.path.pose_at(s)
            d2 = (px - x) ** 2 + (py - y) ** 2
            if d2 < best_d2:
                best_d2, best_s = d2, s
        self.s = best_s
        px, py, th_ref, kappa = self.path.pose_at(best_s)
        # 路径法向（左侧为正）：n = (-sin th, cos th)
        e_y = -(x - px) * math.sin(th_ref) + (y - py) * math.cos(th_ref)
        return {'s': best_s, 'e_y': e_y, 'th_ref': th_ref, 'kappa': kappa,
                'px': px, 'py': py}

    def reset(self, s0=0.0):
        self.s = float(s0)


def initial_error(path, x, y, psi, s0=0.0, window=1e9):
    """给定初始位姿，算出相对路径的初始误差（供仿真初始化用）。"""
    tr = PathTracker(path, s0=s0, window=window)
    info = tr.update(x, y)
    return info['e_y'], wrap_pi(psi - info['th_ref']), tr.s


# ---------------------------------------------------------------------------
# 线性 MPC
# ---------------------------------------------------------------------------

class LinearMPC(object):
    """误差动力学上的线性 MPC，决策变量取转角增量。

    这样「转角幅值」和「转角速率」两类约束都能表达：
        |delta_k| <= delta_max      （幅值，事后兜底）
        |d_delta_k| <= rate_max*dt  （速率，箱式，直接进 QP）

    把 delta_k = delta_prev + sum_{i<=k} d_delta_i 代入误差预测，
    代价函数仍是决策变量的二次型，箱式约束用 FISTA 加速投影梯度求解。
    """

    def __init__(self, dt=0.1, horizon=15, wheelbase=2.5, q=(10.0, 5.0),
                 r=0.5, r_rate=8.0, delta_max=0.6, v_ref=2.0,
                 rate_max=1.6):
        self.dt = float(dt)
        self.N = int(horizon)
        self.L = float(wheelbase)
        self.q = (float(q[0]), float(q[1]))
        self.R = float(r)
        self.R_rate = float(r_rate)
        self.delta_max = float(delta_max)
        self.v_ref = float(v_ref)
        self.du_max = float(rate_max) * self.dt
        self._prepare()

    def _prepare(self):
        v, dt, N = self.v_ref, self.dt, self.N
        A = [[1.0, v * dt], [0.0, 1.0]]
        B = [[0.0], [v * dt / self.L]]
        A_pows = [mat_eye(2)]
        for _k in range(1, N + 1):
            A_pows.append(mat_mul(A_pows[-1], A))
        # G[k][j]：delta_j 对 e_{k+1} 的贡献（二维列向量）
        G = [[(0.0, 0.0)] * N for _ in range(N)]
        for k in range(1, N + 1):
            for j in range(0, k):
                gv = mat_mul(A_pows[k - 1 - j], B)
                G[k - 1][j] = (gv[0][0], gv[1][0])
        # S[k]：delta_prev 单独作用在 e_k 上的系数
        S = [(0.0, 0.0)] * N
        for k in range(1, N + 1):
            sx = sy = 0.0
            for j in range(k):
                sx += G[k - 1][j][0]
                sy += G[k - 1][j][1]
            S[k - 1] = (sx, sy)
        # P[k][i]：d_delta_i 对 e_k 的贡献
        P = [[(0.0, 0.0)] * N for _ in range(N)]
        for k in range(1, N + 1):
            for i in range(0, k):
                sx = sy = 0.0
                for j in range(i, k):
                    sx += G[k - 1][j][0]
                    sy += G[k - 1][j][1]
                P[k - 1][i] = (sx, sy)
        self.A_pows = A_pows
        self.G = G
        self.S = S
        self.P = P
        self.H = self._hessian()
        self.Lipschitz = self._power_iteration(self.H)

    def _hessian(self):
        N, (qy, qp) = self.N, self.q
        H = mat_zeros(N, N)
        for k in range(1, N + 1):
            row = self.P[k - 1]
            for i in range(N):
                ri = row[i]
                if ri[0] == 0.0 and ri[1] == 0.0:
                    continue
                for j in range(N):
                    rj = row[j]
                    H[i][j] += 2.0 * (qy * ri[0] * rj[0] + qp * ri[1] * rj[1])
        # R * delta_k^2，delta_k 含 d_0..d_k
        for i in range(N):
            for j in range(N):
                H[i][j] += 2.0 * self.R * (N - max(i, j))
        # R_rate * d_delta_k^2
        for i in range(N):
            H[i][i] += 2.0 * self.R_rate
        return H

    def _power_iteration(self, H, iters=80):
        n = len(H)
        x = [1.0] * n
        lam = 1.0
        for _ in range(iters):
            y = mat_vec(H, x)
            norm = math.sqrt(sum(v * v for v in y))
            if norm < 1e-12:
                return 1.0
            lam = norm
            x = [v / norm for v in y]
        return max(lam, 1e-6)

    def solve(self, e0, delta_ref, delta_prev=0.0, sweeps=400, tol=1e-12):
        """求解一次 MPC（坐标下降 + 可行集投影）。

        e0          当前误差 [e_y, e_psi]
        delta_ref   预测时域内的参考转角序列（长度 N）
        delta_prev  上一时刻实际前轮转角
        返回 (d_seq, info)，d_seq 为转角增量序列
        """
        N, (qy, qp) = self.N, self.q
        # f_k = A^k e0 - sum_j G[k-1][j] * delta_ref_j
        f = []
        for k in range(1, N + 1):
            base = mat_vec(self.A_pows[k], e0)
            fx, fy = base[0], base[1]
            for j in range(k):
                gx, gy = self.G[k - 1][j]
                fx -= gx * delta_ref[j]
                fy -= gy * delta_ref[j]
            f.append((fx, fy))

        g = [0.0] * N
        for i in range(N):
            s = 0.0
            for k in range(1, N + 1):
                fi = f[k - 1]
                dpx, dpy = self.P[k - 1][i]
                sx, sy = self.S[k - 1]
                s += qy * ((fi[0] + delta_prev * sx) * dpx
                           + (fi[1] + delta_prev * sy) * dpy)
            g[i] = 2.0 * s
        for i in range(N):
            g[i] += 2.0 * self.R * delta_prev * (N - i)

        H = self.H
        d = [0.0] * N
        info = {'sweeps': 0, 'diff': float('inf')}
        for sweep in range(sweeps):
            worst = 0.0
            for i in range(N):
                acc = g[i]
                Hi = H[i]
                for j in range(N):
                    if j != i:
                        acc += Hi[j] * d[j]
                cand = -acc / Hi[i]
                cand = min(max(cand, -self.du_max), self.du_max)
                delta_i = cand - d[i]
                d[i] = cand
                if abs(delta_i) > worst:
                    worst = abs(delta_i)
            info = {'sweeps': sweep + 1, 'diff': worst}
            if worst < tol:
                break
        return d, info

    def delta_sequence(self, d_seq, delta_prev):
        """把增量序列还原成绝对转角序列（并做幅值兜底）。"""
        out = []
        acc = delta_prev
        for d in d_seq:
            acc += d
            acc = min(max(acc, -self.delta_max), self.delta_max)
            out.append(acc)
        return out

    def cost(self, e0, d_seq, delta_ref, delta_prev=0.0):
        """按 J 的定义重算代价，用于单元测试验证 QP 确实求到了更小的值。"""
        N, (qy, qp) = self.N, self.q
        deltas = self.delta_sequence(d_seq, delta_prev)
        e = [e0[0], e0[1]]
        J = 0.0
        prev_d = delta_prev
        for k in range(N):
            nxt = mat_vec(self.A_pows[1], e)
            b = self.v_ref * self.dt / self.L
            nxt[0] += 0.0
            nxt[1] += b * (deltas[k] - delta_ref[k])
            e = nxt
            J += qy * e[0] ** 2 + qp * e[1] ** 2
            J += self.R * deltas[k] ** 2
            J += self.R_rate * (deltas[k] - prev_d) ** 2
            prev_d = deltas[k]
        return J
