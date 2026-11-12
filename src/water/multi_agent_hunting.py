"""
多 AUV 协同围捕与动态任务分配
================================

利用 HoloOcean 的多智能体支持和水声通信仿真 (Acoustic Modem)，
演示通信受限环境下的分布式拍卖算法与协同围捕策略。

模块组成:
    - AuctionAllocator : 拍卖算法任务分配器
    - PIDController    : 简易 PID 控制器
    - AcousticModem    : 水声通信模拟
    - AUVAgent         : 单艘 AUV 封装
    - HuntingScenario  : 场景管理 + 主循环

运行:
    python main.py
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np


# ---------------------------------------------------------------------------
# PID 控制器
# ---------------------------------------------------------------------------
class PIDController:
    """三轴 PID 控制器，输出方向指令 [surge, sway, heave, roll, pitch, yaw]。"""

    def __init__(
        self,
        kp: float = 0.5,
        ki: float = 0.01,
        kd: float = 0.1,
        output_limit: float = 1.0,
    ) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self._integral = np.zeros(3)
        self._prev_error = np.zeros(3)

    def compute(self, target_pos: np.ndarray, current_pos: np.ndarray, dt: float = 0.1) -> np.ndarray:
        """返回 6 维控制指令 [surge, sway, heave, roll=0, pitch=0, yaw]。"""
        error = target_pos - current_pos
        self._integral += error * dt
        derivative = (error - self._prev_error) / max(dt, 1e-6)
        self._prev_error = error.copy()

        # 三轴 PID
        cmd = self.kp * error + self.ki * self._integral + self.kd * derivative

        # 限幅
        cmd = np.clip(cmd, -self.output_limit, self.output_limit)

        # [surge, sway, heave, roll, pitch, yaw]
        return np.array([cmd[0], cmd[1], cmd[2], 0.0, 0.0, math.atan2(cmd[1], cmd[0])])


# ---------------------------------------------------------------------------
# 水声通信模拟
# ---------------------------------------------------------------------------
@dataclass
class AcousticMessage:
    sender: str
    receiver: str
    payload: dict
    timestamp: float = field(default_factory=time.time)


class AcousticModem:
    """模拟水声通信：固定延迟 + 丢包率。"""

    def __init__(self, delay_ms: int = 200, packet_loss: float = 0.05) -> None:
        self.delay_ms = delay_ms
        self.packet_loss = packet_loss
        self._queue: List[AcousticMessage] = []

    def broadcast(self, sender: str, payload: dict) -> None:
        """广播消息（不指定接收者，模拟"所有人都能听到"的水声信道）。"""
        msg = AcousticMessage(sender=sender, receiver="*", payload=payload)
        self._queue.append(msg)
        print(f"  [AcousticModem] {sender} 广播: {payload}  (延迟={self.delay_ms}ms, 丢包率={self.packet_loss:.0%})")

    def poll(self) -> List[AcousticMessage]:
        """取出队列中所有消息（在真实系统中应考虑延迟和丢包）。"""
        ready = self._queue[:]
        self._queue.clear()
        # 模拟丢包
        return [m for m in ready if np.random.random() >= self.packet_loss]


# ---------------------------------------------------------------------------
# 拍卖算法
# ---------------------------------------------------------------------------
@dataclass
class Task:
    name: str
    position: np.ndarray      # 三维目标位置
    priority: int = 1
    assigned_to: Optional[str] = None


class AuctionAllocator:
    """
    分布式拍卖算法：AUV 根据距离 + 剩余电量评估任务代价，
    代价最低的 AUV 认领该任务。
    """

    def __init__(self, energy_weight: float = 0.3) -> None:
        self.energy_weight = energy_weight  # 电量在代价中的权重

    def _cost(self, agent_pos: np.ndarray, agent_energy: float, task: Task) -> float:
        """代价函数 = 距离代价 - 电量奖励。"""
        dist_cost = float(np.linalg.norm(agent_pos - task.position))
        energy_reward = self.energy_weight * agent_energy
        return dist_cost - energy_reward

    def allocate(
        self,
        agents: Dict[str, Dict],   # agent_id -> {"position": np.ndarray, "energy": float}
        tasks: List[Task],
    ) -> Dict[str, Task]:
        """返回 {agent_id: Task} 分配结果。"""
        assignments: Dict[str, Task] = {}
        available_agents = set(agents.keys())
        available_tasks = list(tasks)

        # 任务按优先级降序处理
        available_tasks.sort(key=lambda t: t.priority, reverse=True)

        for task in available_tasks:
            best_agent: Optional[str] = None
            best_cost = float("inf")

            for agent_id in available_agents:
                info = agents[agent_id]
                cost = self._cost(info["position"], info["energy"], task)
                if cost < best_cost:
                    best_cost = cost
                    best_agent = agent_id

            if best_agent is not None:
                assignments[best_agent] = task
                task.assigned_to = best_agent
                available_agents.remove(best_agent)

        return assignments


# ---------------------------------------------------------------------------
# AUV 封装
# ---------------------------------------------------------------------------
@dataclass
class AUVAgent:
    agent_id: str
    controller: PIDController = field(default_factory=PIDController)
    energy: float = 100.0
    state: Dict = field(default_factory=dict)

    def position(self) -> np.ndarray:
        loc = self.state.get("Location", np.zeros(3))
        return np.array(loc, dtype=float)

    def update_energy(self, dt: float, energy_drain_rate: float = 0.05) -> None:
        self.energy = max(0.0, self.energy - energy_drain_rate * dt)


# ---------------------------------------------------------------------------
# 场景管理 + 主循环
# ---------------------------------------------------------------------------
class HuntingScenario:
    """多 AUV 协同围捕场景管理器。"""

    def __init__(self, config_path: str = "scenario.json") -> None:
        self.config = self._load_config(config_path)
        self.modem = AcousticModem(
            delay_ms=self.config.get("comm_delay_ms", 200),
            packet_loss=self.config.get("packet_loss", 0.05),
        )
        self.allocator = AuctionAllocator(
            energy_weight=self.config.get("energy_weight", 0.3),
        )
        self.auction_interval = self.config.get("auction_interval_steps", 50)
        self.max_steps = self.config.get("max_steps", 1000)
        self.dt = self.config.get("dt", 0.1)

        # 初始化 AUV
        self.auvs: Dict[str, AUVAgent] = {}
        for ag_cfg in self.config.get("agents", []):
            self.auvs[ag_cfg["name"]] = AUVAgent(agent_id=ag_cfg["name"])

        # 初始化任务 / 逃逸目标
        self.tasks = self._build_tasks()
        self.escaping_target_pos = np.array(
            self.config.get("target_initial_pos", [0.0, 0.0, -10.0]), dtype=float
        )
        self.escaping_target_vel = np.array(
            self.config.get("target_velocity", [0.1, 0.05, 0.0]), dtype=float
        )

        # HoloOcean env（延迟初始化，方便在没有 HoloOcean 时也能 dry-run）
        self.env = None

    # ---------- 配置 ----------
    @staticmethod
    def _load_config(path: str) -> dict:
        try:
            with open(path, "r", encoding="utf-8") as f:
                return json.load(f)
        except FileNotFoundError:
            print(f"[WARN] 未找到 {path}，使用默认配置。")
            return {}

    def _build_tasks(self) -> List[Task]:
        task_cfgs = self.config.get("tasks", [])
        if not task_cfgs:
            # 默认三个包抄位置
            return [
                Task("Intercept_Left",   np.array([-5.0, -5.0, -10.0]), priority=2),
                Task("Intercept_Right",  np.array([5.0,  -5.0, -10.0]), priority=2),
                Task("Block_Back",       np.array([0.0,   5.0, -10.0]), priority=1),
            ]
        return [
            Task(t["name"], np.array(t["position"], dtype=float), t.get("priority", 1))
            for t in task_cfgs
        ]

    # ---------- HoloOcean ----------
    def init_holoocean(self) -> None:
        """在有 HoloOcean 依赖时调用。"""
        try:
            import holoocean  # type: ignore
        except ImportError:
            print("[INFO] HoloOcean 未安装，进入 dry-run 模式（用模拟数据演示流程）。")
            self.env = None
            return

        scenario_dict = {
            "name": self.config.get("scenario_name", "MultiAgentHunting"),
            "world": self.config.get("world", "OpenWater"),
            "agents": [
                {
                    "name": ag_cfg["name"],
                    "agent_type": ag_cfg.get("agent_type", "TorpedoAUV"),
                    "sensors": [{"sensor_type": "AcousticModem"}],
                }
                for ag_cfg in self.config.get("agents", [])
            ],
        }
        self.env = holoocean.make(scenario_dict)
        print(f"[INFO] HoloOcean 环境已启动: {scenario_dict['name']}")

    # ---------- 状态获取 ----------
    def _simulate_states(self) -> None:
        """没有 HoloOcean 时的模拟数据：AUV 从随机小偏移处出发。"""
        for auv in self.auvs.values():
            if not auv.state:
                # 第一次：给个随机起点
                offset = np.random.uniform(-5.0, 5.0, size=3)
                auv.state = {"Location": offset, "Energy": auv.energy}
            else:
                # 已有位置：保持不变，等控制步骤会朝目标移动
                auv.state["Energy"] = auv.energy

    def _dry_run_move(self, agent_id: str, target_pos: np.ndarray, step_size: float = 0.2) -> None:
        """dry-run 模式下让 AUV 朝目标靠近一点。"""
        auv = self.auvs[agent_id]
        current = np.array(auv.state.get("Location", np.zeros(3)), dtype=float)
        direction = target_pos - current
        dist = float(np.linalg.norm(direction))
        if dist > 0.01:
            move = direction / dist * min(step_size, dist)
            auv.state["Location"] = (current + move).tolist()

    def _update_target(self) -> None:
        """逃逸目标按固定速度移动。"""
        self.escaping_target_pos += self.escaping_target_vel * self.dt
        # 简单边界反射
        bounds = self.config.get("world_bounds", 50.0)
        for i in range(3):
            if abs(self.escaping_target_pos[i]) > bounds:
                self.escaping_target_vel[i] *= -1.0

    # ---------- 主循环 ----------
    def run(self) -> None:
        print("=" * 60)
        print(" 多 AUV 协同围捕场景启动")
        print(f" AUV 数量: {len(self.auvs)}   任务数量: {len(self.tasks)}")
        print(f" 拍卖周期: {self.auction_interval} 步   最大步数: {self.max_steps}")
        print("=" * 60)

        try:
            self.init_holoocean()
            if self.env is None:
                self._simulate_states()

            current_assignments: Dict[str, Task] = {}

            for step in range(self.max_steps):
                # 1) 更新目标位置
                self._update_target()

                # 2) 更新状态
                if self.env is None:
                    self._simulate_states()
                else:
                    tick_result = self.env.tick()
                    for name in self.auvs:
                        self.auvs[name].state = tick_result.get(name, {})

                # 3) 周期性重新拍卖
                if step % self.auction_interval == 0:
                    agent_info = {
                        name: {
                            "position": auv.position(),
                            "energy": auv.energy,
                        }
                        for name, auv in self.auvs.items()
                    }
                    current_assignments = self.allocator.allocate(agent_info, self.tasks)

                    # 通过水声信道广播分配结果
                    for agent_id, task in current_assignments.items():
                        self.modem.broadcast(
                            agent_id,
                            {"assigned_task": task.name, "target": task.position.tolist()},
                        )

                    print(f"\n[Step {step:5d}] 任务分配结果:")
                    for agent_id, task in current_assignments.items():
                        print(f"   {agent_id} -> {task.name}  @ {task.position.tolist()}")

                # 4) 控制各 AUV 执行
                for agent_id, auv in self.auvs.items():
                    target = current_assignments.get(agent_id)
                    if target is None:
                        continue

                    # 目标指向逃逸目标（包抄）
                    target_pos = self.escaping_target_pos
                    cmd = auv.controller.compute(target_pos, auv.position(), dt=self.dt)
                    auv.update_energy(self.dt)

                    if self.env is not None:
                        self.env.act(agent_id, cmd.tolist())
                    else:
                        # dry-run：让 AUV 朝目标实际靠近
                        self._dry_run_move(agent_id, target_pos)

                # 5) 通信轮询（消费水声消息）
                self.modem.poll()

                # 6) 每 100 步打印进度
                if step % 100 == 0 and step > 0:
                    print(f"[Step {step:5d}] 进行中...  目标位置: {self.escaping_target_pos.tolist()}")

            print("\n" + "=" * 60)
            print(" 场景结束 ✓")
            print(f" 最终目标位置: {self.escaping_target_pos.tolist()}")
            print("=" * 60)

        except KeyboardInterrupt:
            print("\n[INFO] 收到 Ctrl+C，优雅退出。")


# ---------------------------------------------------------------------------
# 入口
# ---------------------------------------------------------------------------
def main() -> None:
    import os
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(script_dir, "scenario.json")

    scenario = HuntingScenario(config_path=config_path)
    scenario.run()


if __name__ == "__main__":
    main()
