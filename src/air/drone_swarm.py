"""无人机集群编队控制核心实现.

采用 Leader-Follower 架构 + Consensus 一致性算法，
支持三角形、菱形、直线等多种编队队形切换。
"""
import json
import math
import numpy as np
from pathlib import Path


class DroneAgent:
    """单个无人机智能体."""

    def __init__(self, agent_id, init_pos, velocity=(0.0, 0.0, 0.0)):
        self.id = agent_id
        self.position = np.array(init_pos, dtype=float)
        self.velocity = np.array(velocity, dtype=float)
        self.target_offset = np.zeros(3)  # 相对于编队中心的期望偏移
        self.neighbors = []
        self.is_leader = False

    def update_position(self, dt=0.1):
        self.position += self.velocity * dt
        return self.position


class FormationController:
    """编队控制器 —— Consensus + 队形误差反馈."""

    def __init__(self, formation_type="triangle", k_consensus=0.5, k_shape=1.2):
        self.formation_type = formation_type
        self.k_consensus = k_consensus
        self.k_shape = k_shape

    def get_desired_offsets(self, n_agents):
        """根据编队类型返回期望的相对偏移."""
        if self.formation_type == "triangle":
            # 等边三角形
            offsets = []
            r = 3.0
            for i in range(n_agents):
                theta = 2 * math.pi * i / n_agents
                offsets.append([r * math.cos(theta), r * math.sin(theta), 0.0])
            return offsets
        elif self.formation_type == "diamond":
            # 菱形（4 架）
            return [
                [0.0, 0.0, 0.0],
                [2.0, -2.0, 0.0],
                [-2.0, -2.0, 0.0],
                [0.0, 2.0, 0.0],
            ][:n_agents]
        elif self.formation_type == "line":
            # 直线编队
            return [[i * 2.0 - (n_agents - 1), 0.0, 0.0] for i in range(n_agents)]
        else:
            # 默认随机散布
            return [[np.random.uniform(-2, 2), np.random.uniform(-2, 2), 0.0]
                    for _ in range(n_agents)]

    def consensus_step(self, drones, leader_pos):
        """执行一轮 Consensus + 队形保持."""
        offsets = self.get_desired_offsets(len(drones))

        for i, drone in enumerate(drones):
            if drone.is_leader:
                # 领导者只跟踪全局目标，不受队形约束
                continue

            # 计算期望位置 = 领导者位置 + 期望偏移
            desired = leader_pos + np.array(offsets[i])

            # 队形误差
            shape_error = desired - drone.position

            # Consensus 项：与邻居保持一致
            consensus_term = np.zeros(3)
            n_neighbors = 0
            for j, other in enumerate(drones):
                if i != j and j in drone.neighbors:
                    consensus_term += other.position - drone.position - (
                        np.array(offsets[j]) - np.array(offsets[i])
                    )
                    n_neighbors += 1
            if n_neighbors > 0:
                consensus_term /= n_neighbors

            # 合成控制量
            control = (
                self.k_shape * shape_error + self.k_consensus * consensus_term
            )
            # 限幅
            max_ctrl = 1.5
            norm = np.linalg.norm(control)
            if norm > max_ctrl:
                control = control / norm * max_ctrl
            drone.velocity = control

        # 领导者匀速前进
        for drone in drones:
            if drone.is_leader:
                drone.velocity = np.array([0.3, 0.0, 0.0])


def build_comm_topology(n_agents, topology="ring"):
    """构建通信拓扑."""
    neighbors = {i: [] for i in range(n_agents)}
    if topology == "ring":
        for i in range(n_agents):
            neighbors[i] = [(i - 1) % n_agents, (i + 1) % n_agents]
    elif topology == "fully_connected":
        for i in range(n_agents):
            neighbors[i] = [j for j in range(n_agents) if j != i]
    return neighbors


def compute_formation_error(drones, leader_pos, offsets):
    """计算队形误差（所有无人机到期望位置的距离均值）."""
    total = 0.0
    for i, d in enumerate(drones):
        if d.is_leader:
            continue
        desired = leader_pos + np.array(offsets[i])
        total += np.linalg.norm(d.position - desired)
    n_followers = sum(1 for d in drones if not d.is_leader)
    return total / max(n_followers, 1)


def run_demo():
    """运行编队控制演示."""
    cfg_path = Path(__file__).parent / "scenario.json"
    if cfg_path.exists():
        with open(cfg_path) as f:
            cfg = json.load(f)
    else:
        cfg = {
            "n_agents": 5,
            "formation": "triangle",
            "topology": "ring",
            "max_steps": 500,
            "dt": 0.1,
            "k_consensus": 0.5,
            "k_shape": 1.2,
        }

    n = cfg["n_agents"]
    dt = cfg.get("dt", 0.1)
    max_steps = cfg.get("max_steps", 500)

    # 创建无人机
    drones = []
    for i in range(n):
        init = [np.random.uniform(-5, 5), np.random.uniform(-5, 5), 5.0]
        drones.append(DroneAgent(i, init))
    drones[0].is_leader = True  # 第 0 架为领导者

    # 通信拓扑
    neighbors = build_comm_topology(n, cfg.get("topology", "ring"))
    for i, d in enumerate(drones):
        d.neighbors = neighbors[i]

    # 控制器
    ctrl = FormationController(
        formation_type=cfg.get("formation", "triangle"),
        k_consensus=cfg.get("k_consensus", 0.5),
        k_shape=cfg.get("k_shape", 1.2),
    )
    offsets = ctrl.get_desired_offsets(n)

    print("=" * 55)
    print(f"  无人机集群编队控制演示")
    print(f"  数量: {n}  队形: {ctrl.formation_type}  拓扑: {cfg.get('topology', 'ring')}")
    print("=" * 55)

    error_history = []
    for step in range(max_steps):
        leader_pos = drones[0].position.copy()
        ctrl.consensus_step(drones, leader_pos)

        for d in drones:
            d.update_position(dt)

        if step % 50 == 0:
            err = compute_formation_error(drones, drones[0].position, offsets)
            error_history.append(err)
            print(f"  Step {step:4d} | 队形误差: {err:.4f} m | 领导者: {drones[0].position[:2]}")

    print("\n✅ 演示结束")
    if error_history:
        print(f"  初始队形误差: {error_history[0]:.4f} m")
        print(f"  最终队形误差: {error_history[-1]:.4f} m")


if __name__ == "__main__":
    run_demo()
