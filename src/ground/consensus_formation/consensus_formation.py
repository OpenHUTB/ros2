"""
多机器人 Consensus 编队控制
==========================

6 台差动轮移动机器人，通过分布式一致性算法 + 形状偏移达成全局编队。
与 Leader-Follower 固定编队不同：本模块 **无固定 leader**，每台机器人只
通过局部邻居通信（连通性 3m）逐步收敛到期望形状。

核心算法：
  Consensus + Shape Offset
  ẋ_i = u_i * cos(θ_i)
  ẏ_i = u_i * sin(θ_i)
  u_i = -Kp * sum_{j∈N_i} [ ((x_i - o_i) - (x_j - o_j)) ]
  + 避障项（若距离 < 安全半径）
"""

import numpy as np


# ---------------------------------------------------------------------------
# 形状定义（相对目标位置，单位 m）
# ---------------------------------------------------------------------------
def build_hexagon_offsets(n):
    """正六边形形状偏移（n=6 时每个点占 60°）。"""
    R = 1.25  # 半径
    offsets = []
    for k in range(n):
        theta_k = 2.0 * np.pi * k / n
        offsets.append(np.array([R * np.cos(theta_k), R * np.sin(theta_k)]))
    return offsets


def build_line_offsets(n):
    """直线编队形状偏移。"""
    L = 2.0  # 总长
    offsets = []
    for k in range(n):
        offsets.append(np.array([-L / 2 + L * k / (n - 1), 0.0]))
    return offsets


def build_diamond_offsets(n):
    """菱形编队形状偏移（n=6：上下各 1，左右各 2）。"""
    positions = []
    if n >= 6:
        positions = [
            [0.0, 1.3],
            [1.0, 0.6],
            [1.0, -0.6],
            [0.0, -1.3],
            [-1.0, -0.6],
            [-1.0, 0.6],
        ]
    else:
        for k in range(n):
            positions.append(
                [np.cos(2 * np.pi * k / n), np.sin(2 * np.pi * k / n)]
            )
    return [np.array(p) for p in positions]


SHAPES = {
    "hexagon": build_hexagon_offsets,
    "line": build_line_offsets,
    "diamond": build_diamond_offsets,
}


# ---------------------------------------------------------------------------
# 机器人动力学
# ---------------------------------------------------------------------------
class DifferentialRobot:
    def __init__(self, x0=np.zeros(3), idx=0):
        self.idx = idx
        self.state = np.asarray(x0, dtype=float)  # [x, y, θ]
        self.target_offset = np.zeros(2)
        self.neighbors = []  # 当前邻居 index 列表

    @property
    def pos(self):
        return self.state[:2].copy()

    @property
    def heading(self):
        return self.state[2]

    def step(self, u, dt=0.05):
        """单轮积分器近似 + 角速度调整（朝向与速度一致）。"""
        dx = np.array([u[0] * np.cos(self.state[2]),
                       u[0] * np.sin(self.state[2]),
                       u[1]])
        self.state += dx * dt
        # 归一化角度
        self.state[2] = (self.state[2] + np.pi) % (2 * np.pi) - np.pi


# ---------------------------------------------------------------------------
# Consensus 控制器
# ---------------------------------------------------------------------------
class ConsensusController:
    def __init__(self, Kp=1.5, Kd=0.3, comm_range=3.0,
                 safety_radius=0.5, max_linear=0.6, max_angular=1.5):
        self.Kp = Kp
        self.Kd = Kd
        self.comm_range = comm_range
        self.safety_radius = safety_radius
        self.max_linear = max_linear
        self.max_angular = max_angular

    def update_neighbors(self, robots):
        """基于通信范围建立邻居集合。"""
        for r in robots:
            r.neighbors = []
        for i, ri in enumerate(robots):
            for j, rj in enumerate(robots):
                if i == j:
                    continue
                if np.linalg.norm(ri.pos - rj.pos) < self.comm_range:
                    ri.neighbors.append(j)

    def compute(self, robots):
        """每台机器人的控制输入 u = [线速度, 角速度]。"""
        self.update_neighbors(robots)
        u_out = []
        # 计算编队质心（估计目标）
        centroid = np.mean([r.pos - r.target_offset for r in robots], axis=0)

        for ri in robots:
            # --- Consensus 项 ---
            consensus_err = np.zeros(2)
            for j in ri.neighbors:
                rj = robots[j]
                # 误差 = (ri - offset_i) - (rj - offset_j)
                diff = (ri.pos - ri.target_offset) - (rj.pos - rj.target_offset)
                consensus_err += diff
            if len(ri.neighbors) > 0:
                consensus_err /= len(ri.neighbors)

            # --- 目标牵引项（保证质心收敛） ---
            target_center = centroid
            centroid_err = (ri.pos - ri.target_offset) - target_center

            # --- 避障项 ---
            avoid = np.zeros(2)
            for j, rj in enumerate(robots):
                if j == ri.idx:
                    continue
                d = np.linalg.norm(ri.pos - rj.pos)
                if d < self.safety_radius:
                    avoid += (ri.pos - rj.pos) / (d + 1e-3) * \
                             (self.safety_radius - d)

            # 合成期望速度
            v_des = (
                -self.Kp * consensus_err
                - self.Kp * centroid_err * 0.1
                + self.Kp * avoid * 2.0
            )

            # 朝向期望方向
            speed = np.linalg.norm(v_des)
            if speed < 1e-3:
                u_lin = 0.0
                u_ang = 0.0
            else:
                dir_err = np.arctan2(v_des[1], v_des[0]) - ri.heading
                dir_err = (dir_err + np.pi) % (2 * np.pi) - np.pi
                u_lin = np.clip(speed, 0.0, self.max_linear)
                u_ang = np.clip(self.Kd * dir_err,
                                -self.max_angular, self.max_angular)

            u_out.append(np.array([u_lin, u_ang]))
        return u_out


# ---------------------------------------------------------------------------
# 仿真
# ---------------------------------------------------------------------------
def simulate(n_robots=6, shape="hexagon", duration=40.0, dt=0.05,
             seed=42):
    rng = np.random.default_rng(seed)
    offsets = SHAPES[shape](n_robots)
    offsets = [off - np.mean(offsets, axis=0) for off in offsets]

    robots = []
    for i in range(n_robots):
        # 随机初始位置（质心附近 ±2.5m）
        x0 = np.array([rng.uniform(-2.5, 2.5),
                       rng.uniform(-2.5, 2.5),
                       rng.uniform(-np.pi, np.pi)])
        r = DifferentialRobot(x0, idx=i)
        r.target_offset = offsets[i]
        robots.append(r)

    ctrl = ConsensusController()
    steps = int(duration / dt)

    history = {
        "time": np.linspace(0, duration, steps + 1),
        "centroid_err": np.zeros(steps + 1),
        "shape_err": np.zeros(steps + 1),
        "trajectories": [r.pos.copy() for r in robots],
    }

    def compute_errors():
        actual_centroid = np.mean([r.pos - r.target_offset
                                   for r in robots], axis=0)
        centroid_err = np.linalg.norm(actual_centroid)
        # 形状误差 = 相对位置的方差（与目标 offset 比较）
        shape_err = 0.0
        for r in robots:
            rel = r.pos - np.mean([rr.pos for rr in robots], axis=0)
            shape_err += np.linalg.norm(rel - r.target_offset) ** 2
        shape_err /= n_robots
        return centroid_err, np.sqrt(shape_err)

    history["centroid_err"][0], history["shape_err"][0] = compute_errors()

    for t in range(steps):
        u_list = ctrl.compute(robots)
        for r, u in zip(robots, u_list):
            r.step(u, dt=dt)
        history["centroid_err"][t + 1], history["shape_err"][t + 1] = \
            compute_errors()

    return robots, history, offsets


# ---------------------------------------------------------------------------
# 控制台输出
# ---------------------------------------------------------------------------
def run_demo():
    print("=" * 60)
    print("  Consensus 多机器人编队控制 (无固定 Leader)")
    print("=" * 60)
    print()

    for shape in ["hexagon", "line", "diamond"]:
        n = 6
        print(f"  📐 形状: {shape:8s}  |  机器人: {n}  |  仿真: 40s")
        robots, hist, offsets = simulate(n_robots=n, shape=shape)

        final_centroid = hist["centroid_err"][-1]
        final_shape = hist["shape_err"][-1]

        # 判断收敛（形状误差 < 0.08m 算成功）
        converged = final_shape < 0.08
        print(f"    🎯 质心误差: {final_centroid:.3f} m  "
              f"形状误差: {final_shape:.3f} m  "
              f"{'✅ 收敛' if converged else '⚠️ 未收敛'}")

        # 初始邻居密度
        ctrl = ConsensusController()
        ctrl.update_neighbors(robots)
        nbr_counts = [len(r.neighbors) for r in robots]
        print(f"    🔗 每台邻居数: {nbr_counts}  "
              f"(通信半径 3.0m)")

        print()

    print("-" * 60)
    print("  💡 核心机制:")
    print("    · 分布式 Consensus (局部邻居 → 全局收敛)")
    print("    · Shape offset 保持相对位置")
    print("    · 圆形避障 (距离 < 0.5m 时排斥)")
    print("=" * 60)


if __name__ == "__main__":
    run_demo()
