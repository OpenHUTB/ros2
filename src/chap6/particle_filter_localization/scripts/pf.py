"""一维蒙特卡洛定位（粒子滤波）最小实现，仅依赖标准库。

场景
----
机器人在一条 20 m 长的走廊里前后运动，已知三个固定信标的坐标。机器人**只能
测到与各信标的距离**（range-only），且自身轮式里程计存在系统性偏差。

- 里程计（航迹推算）：只用速度积分，误差随时间累积、最终发散。
- 粒子滤波：用 N 个带权假设表示位置分布，按测距似然重新加权，退化时重采样。
  它能不断用测距"纠正"里程计的漂移，因此误差有界。

粒子滤波流程
------------
1. 预测：每个粒子按运动模型加噪声采样   x_i ← x_i + v·dt + ε
2. 更新：递推贝叶斯加权（**乘上上一轮权重**，不能覆盖）
         W_i⁽ᵗ⁾ ∝ W_i⁽ᵗ⁻¹⁾ · Π_j N(r_j; |x_i - b_j|, σ_r)，随后归一化
3. 估计：加权均值（或加权中位数）
4. 重采样：有效粒子数 Neff < N/2 时做系统重采样，抑制权值退化
"""

import math

# ---------------------------------------------------------------------------
# 伪随机（与 kalman 模块同款 LCG + Box-Muller，保证可复现）
# ---------------------------------------------------------------------------


class Rng:
    def __init__(self, seed=1234):
        self.state = seed & 0x7FFFFFFF
        self._spare = None

    def uniform(self):
        self.state = (1103515245 * self.state + 12345) & 0x7FFFFFFF
        return (self.state + 0.5) / 2147483648.0

    def normal(self, sigma=1.0):
        if self._spare is not None:
            v = self._spare
            self._spare = None
            return v * sigma
        u1 = max(self.uniform(), 1e-12)
        u2 = self.uniform()
        r = math.sqrt(-2.0 * math.log(u1))
        self._spare = r * math.sin(2.0 * math.pi * u2)
        return r * math.cos(2.0 * math.pi * u2) * sigma


def normal_pdf(z, mu, sigma):
    d = z - mu
    return math.exp(-0.5 * (d / sigma) ** 2) / (sigma * math.sqrt(2.0 * math.pi))


# ---------------------------------------------------------------------------
# 粒子滤波
# ---------------------------------------------------------------------------


class ParticleFilter1D:
    def __init__(self, n_particles=300, x_range=(0.0, 20.0), sigma_move=0.06,
                 sigma_range=0.35, seed=7, rng=None):
        self.n = int(n_particles)
        self.rng = rng or Rng(seed)
        lo, hi = x_range
        self.particles = [lo + (hi - lo) * self.rng.uniform() for _ in range(self.n)]
        self.weights = [1.0 / self.n] * self.n
        self.sigma_move = sigma_move
        self.sigma_range = sigma_range

    # -- 1) 预测 ----------------------------------------------------------
    def predict(self, v, dt):
        s = self.sigma_move * math.sqrt(dt)
        self.particles = [p + v * dt + self.rng.normal(s) for p in self.particles]

    # -- 2) 更新 ----------------------------------------------------------
    def update(self, ranges, beacons):
        """贝叶斯更新：新权重 ∝ **旧权重 × 当前观测似然**。

        这里必须乘上 ``self.weights`` 而不是只用当前帧的似然 ``lik``。
        原因：``step()`` 只在 ``Neff < N/2`` 时才重采样。若中间几轮没有触发
        重采样，粒子集合完全没变（运动模型已把先验搬到了新位置），此时
        "之前所有观测的证据"只能靠权重携带；如果每轮都拿当前似然覆盖一遍，
        就等于把历史观测全丢了，滤波器退化成"只看最后一帧"。

        重采样之后 ``weights`` 被重置为均匀，历史信息已体现在粒子的复制次数
        里，此时乘 ``1/N`` 只是个常数因子，归一化后不影响结果，所以统一写成
        递推式是安全的。
        """
        w = []
        for p, w_old in zip(self.particles, self.weights):
            lik = 1.0
            for r, b in zip(ranges, beacons):
                lik *= normal_pdf(r, abs(p - b), self.sigma_range)
            w.append(w_old * lik + 1e-300)   # 防止所有权重同时下溢为 0
        s = sum(w)
        if s <= 0 or not math.isfinite(s):
            self.weights = [1.0 / self.n] * self.n
            return
        self.weights = [x / s for x in w]

    # -- 3) 估计 ----------------------------------------------------------
    def estimate(self):
        return sum(p * w for p, w in zip(self.particles, self.weights))

    def effective_n(self):
        return 1.0 / sum(w * w for w in self.weights)

    # -- 4) 系统重采样 ----------------------------------------------------
    def resample(self):
        n = self.n
        positions = [(i + self.rng.uniform()) / n for i in range(n)]
        out, cum, i = [], 0.0, 0
        cdf = []
        for w in self.weights:
            cum += w
            cdf.append(cum)
        cdf[-1] = 1.0
        for pos in positions:
            while i < n - 1 and pos > cdf[i]:
                i += 1
            out.append(self.particles[i])
        self.particles = out
        self.weights = [1.0 / n] * n

    def step(self, v, dt, ranges, beacons):
        self.predict(v, dt)
        self.update(ranges, beacons)
        if self.effective_n() < self.n / 2.0:
            self.resample()
            resampled = True
        else:
            resampled = False
        return self.estimate(), resampled


# ---------------------------------------------------------------------------
# 仿真：航迹推算 vs 粒子滤波
# ---------------------------------------------------------------------------


def simulate(dt=0.05, duration=30.0, beacons=(2.0, 10.0, 18.0),
             v_nominal=0.55, odom_bias=1.06, seed=11, n_particles=300,
             x_start=1.0):
    """返回 dict：ts / x_true / x_odom / x_pf / err_odom / err_pf / resamples

    机器人从 x_start 出发沿走廊**单向巡航**约 16 m。里程计速度被系统高估 6%，
    因此航迹推算误差会随行驶距离线性累积；粒子滤波靠测距把误差压回来。
    """
    rng = Rng(seed)
    pf = ParticleFilter1D(n_particles=n_particles, x_range=(0.0, 20.0),
                          sigma_move=0.06, sigma_range=0.35,
                          rng=Rng(seed + 1))
    x_true = x_start
    x_odom = x_start         # 航迹推算：同一起点，但速度有系统偏差
    ts, xt, xo, xp, eo, ep = [], [], [], [], [], []
    resamples = 0
    n = int(round(duration / dt))
    for k in range(n):
        t = k * dt
        # 真实速度：巡航速度上叠加轻微起伏
        v_true = v_nominal + 0.05 * math.sin(2.0 * math.pi * t / 7.0)
        x_true = min(x_true + v_true * dt, 20.0)
        # 里程计：速度被系统高估 6%（典型轮径误差），并有随机噪声
        v_odo = v_true * odom_bias + rng.normal(0.02)
        x_odom += v_odo * dt
        # 测距（三信标，含噪声）
        ranges = [abs(x_true - b) + rng.normal(0.35) for b in beacons]
        # 粒子滤波用里程计读数做运动输入（不是真值！）
        est, rs = pf.step(v_odo, dt, ranges, beacons)
        if rs:
            resamples += 1
        ts.append(t)
        xt.append(x_true)
        xo.append(x_odom)
        xp.append(est)
        eo.append(x_odom - x_true)
        ep.append(est - x_true)
    return {"ts": ts, "x_true": xt, "x_odom": xo, "x_pf": xp,
            "err_odom": eo, "err_pf": ep, "resamples": resamples,
            "particles": list(pf.particles), "weights": list(pf.weights)}


def rmse(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)) / len(a))
