"""多 AUV 协同围捕的核心逻辑。

只依赖 numpy，不依赖 HoloOcean，因此可以在没有仿真器的机器上直接跑测试。

组成：
    - AcousticModem     水声信道：固定延迟 + 丢包，消息按模拟时间投递
    - AuctionAllocator  拍卖算法：距离 + 电量加权代价
    - AUVAgent          单艘 AUV：位置、电量、PID 控制
    - HuntingScenario   场景主循环：拍卖、通信、控制、捕获判定
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np


# ---------------------------------------------------------------------------
# 水声通信
# ---------------------------------------------------------------------------
@dataclass
class AcousticMessage:
    """一条待投递的水声消息。"""

    sender: str
    payload: dict
    deliver_at: float


class AcousticModem:
    """固定延迟 + 丢包的水声信道。

    丢包在发送瞬间决定；延迟按模拟时间判断，只有 ``deliver_at <= now`` 的
    消息才会被 ``poll`` 取走。因此接收方在延迟窗口内看不到新的分配结果，
    只能沿用上一次收到的任务——这正是「通信受限」对协同的影响。
    """

    def __init__(self, delay_ms: float = 200.0, packet_loss: float = 0.05,
                 rng: Optional[np.random.Generator] = None) -> None:
        if delay_ms < 0.0:
            raise ValueError('delay_ms 不能为负数，当前为 %r' % (delay_ms,))
        if not 0.0 <= packet_loss <= 1.0:
            raise ValueError(
                'packet_loss 必须在 0 到 1 之间，当前为 %r' % (packet_loss,))
        self.delay = float(delay_ms) / 1000.0
        self.packet_loss = float(packet_loss)
        self.rng = rng if rng is not None else np.random.default_rng(0)
        self._queue: List[AcousticMessage] = []
        self.sent = 0
        self.dropped = 0

    def broadcast(self, sender: str, payload: dict, now: float) -> bool:
        """广播一条消息，返回是否成功送达（False 表示被丢弃）。"""
        self.sent += 1
        if self.packet_loss > 0.0 and self.rng.random() < self.packet_loss:
            self.dropped += 1
            return False
        self._queue.append(
            AcousticMessage(sender, dict(payload), now + self.delay))
        return True

    def poll(self, now: float) -> List[AcousticMessage]:
        """取出此刻已经到达的消息，未到期的留在队列里。"""
        ready = [m for m in self._queue if m.deliver_at <= now]
        self._queue = [m for m in self._queue if m.deliver_at > now]
        ready.sort(key=lambda m: (m.deliver_at, m.sender))
        return ready

    def pending(self) -> int:
        """仍在延迟中的消息数量。"""
        return len(self._queue)


# ---------------------------------------------------------------------------
# 拍卖算法
# ---------------------------------------------------------------------------
@dataclass
class Task:
    """一个包抄位置。"""

    name: str
    position: np.ndarray
    priority: int = 1


class AuctionAllocator:
    """分布式拍卖：按「距离 − 电量奖励」的代价把任务分给代价最低的 AUV。

    电量奖励按满电比例归一化（``energy / 100``），乘以 ``energy_weight``，
    单位相当于「满电可以抵扣的距离（米）」。这样不同 AUV 的实际耗电差异
    才会影响分配结果，而不是被同一个常数平移掩盖。
    """

    def __init__(self, energy_weight: float = 5.0) -> None:
        self.energy_weight = float(energy_weight)

    def _cost(self, agent_pos: np.ndarray, agent_energy: float,
              task: Task) -> float:
        dist_cost = float(np.linalg.norm(agent_pos - task.position))
        energy_reward = self.energy_weight * (agent_energy / 100.0)
        return dist_cost - energy_reward

    def allocate(
        self,
        agents: Dict[str, Dict],
        tasks: List[Task],
    ) -> Dict[str, Task]:
        """返回 {agent_id: Task}，每个任务最多分给一艘 AUV。"""
        assignments: Dict[str, Task] = {}
        available_agents = list(agents.keys())
        for task in sorted(tasks, key=lambda t: t.priority, reverse=True):
            best_agent: Optional[str] = None
            best_cost = float('inf')
            for agent_id in available_agents:
                info = agents[agent_id]
                cost = self._cost(info['position'], info['energy'], task)
                if cost < best_cost:
                    best_cost = cost
                    best_agent = agent_id
            if best_agent is not None:
                assignments[best_agent] = task
                available_agents.remove(best_agent)
        return assignments


# ---------------------------------------------------------------------------
# 控制
# ---------------------------------------------------------------------------
class PIDController:
    """三轴 PID，输出 [surge, sway, heave, roll, pitch, yaw]。"""

    def __init__(self, kp: float = 0.5, ki: float = 0.01,
                 kd: float = 0.1, output_limit: float = 1.0) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self._integral = np.zeros(3)
        self._prev_error = np.zeros(3)

    def compute(self, target_pos: np.ndarray, current_pos: np.ndarray,
                dt: float = 0.1) -> np.ndarray:
        error = np.asarray(target_pos, dtype=float) - np.asarray(
            current_pos, dtype=float)
        self._integral += error * dt
        derivative = (error - self._prev_error) / max(dt, 1e-6)
        self._prev_error = error.copy()
        cmd = self.kp * error + self.ki * self._integral + self.kd * derivative
        cmd = np.clip(cmd, -self.output_limit, self.output_limit)
        return np.array([cmd[0], cmd[1], cmd[2], 0.0, 0.0,
                         math.atan2(cmd[1], cmd[0])])


def intercept_point(task_pos: np.ndarray, target_pos: np.ndarray,
                    closing_ratio: float) -> np.ndarray:
    """包抄点向逃逸目标收缩后的实际控制点。

    ``closing_ratio`` = 0 时停在包抄点，= 1 时直接追目标，中间值让多艘 AUV
    从不同方向同时收拢，形成合围而不是三艘挤在同一个点上。
    """
    ratio = min(max(float(closing_ratio), 0.0), 1.0)
    task_pos = np.asarray(task_pos, dtype=float)
    target_pos = np.asarray(target_pos, dtype=float)
    return task_pos + (target_pos - task_pos) * ratio


def closing_ratio_at(elapsed: float, base_ratio: float,
                     closing_time: float) -> float:
    """收缩比例随时间线性增大，最多到 1（直接追目标）。

    一开始停在包抄点外侧完成合围，随后逐步收紧；``closing_time`` 是收紧到
    贴身所需的时间（秒）。这样既能保持「从不同方向包抄」的形态，又能真正
    进入捕获半径。
    """
    if closing_time <= 0.0:
        return 1.0
    ratio = base_ratio + float(elapsed) / float(closing_time)
    return min(max(ratio, 0.0), 1.0)


def captured(positions: List[np.ndarray], target_pos: np.ndarray,
             radius: float) -> bool:
    """所有 AUV 都进入捕获半径时判定围捕成功。"""
    if not positions:
        return False
    target_pos = np.asarray(target_pos, dtype=float)
    return all(np.linalg.norm(np.asarray(p, dtype=float) - target_pos)
               <= radius for p in positions)


# ---------------------------------------------------------------------------
# AUV
# ---------------------------------------------------------------------------
@dataclass
class AUVAgent:
    """单艘 AUV。"""

    agent_id: str
    controller: PIDController = field(default_factory=PIDController)
    energy: float = 100.0
    state: Dict = field(default_factory=dict)

    def position(self) -> np.ndarray:
        """读取当前位置。

        dry-run 用 ``Location``，HoloOcean 的观测按传感器类型做键，位置在
        ``LocationSensor`` 里，两种都兼容。
        """
        for key in ('Location', 'LocationSensor'):
            value = self.state.get(key)
            if value is not None:
                return np.array(value, dtype=float)
        return np.zeros(3)

    def update_energy(self, dt: float, cmd: np.ndarray,
                      idle_drain_rate: float = 0.02,
                      move_drain_rate: float = 0.15) -> None:
        """按本船指令强度扣电：动得越多耗得越快。"""
        effort = float(np.linalg.norm(np.asarray(cmd, dtype=float)[:3]))
        drain = (idle_drain_rate + move_drain_rate * effort) * dt
        self.energy = max(0.0, self.energy - drain)


# ---------------------------------------------------------------------------
# 场景
# ---------------------------------------------------------------------------
class HuntingScenario:
    """多 AUV 协同围捕场景。"""

    def __init__(self, config: dict,
                 rng: Optional[np.random.Generator] = None) -> None:
        self.config = config
        self.seed = int(config.get('seed', 42))
        self.rng = rng if rng is not None else np.random.default_rng(self.seed)

        self.dt = float(config.get('dt', 0.1))
        self.max_steps = int(config.get('max_steps', 1000))
        self.auction_interval = int(config.get('auction_interval_steps', 50))
        self.closing_ratio = float(config.get('closing_ratio', 0.35))
        self.closing_time = float(config.get('closing_time', 20.0))
        self.capture_radius = float(config.get('capture_radius', 2.0))
        self.idle_drain_rate = float(config.get('idle_drain_rate', 0.02))
        self.move_drain_rate = float(config.get('move_drain_rate', 0.15))
        self.verbose = bool(config.get('verbose', True))

        self.modem = AcousticModem(
            delay_ms=config.get('comm_delay_ms', 200),
            packet_loss=config.get('packet_loss', 0.05),
            rng=self.rng,
        )
        self.allocator = AuctionAllocator(
            energy_weight=config.get('energy_weight', 5.0))

        self.auvs: Dict[str, AUVAgent] = {
            ag_cfg['name']: AUVAgent(
                agent_id=ag_cfg['name'],
                energy=float(ag_cfg.get('energy', 100.0)))
            for ag_cfg in config.get('agents', [])
        }
        self.tasks = self._build_tasks()
        self.target_pos = np.array(
            config.get('target_initial_pos', [0.0, 0.0, -10.0]), dtype=float)
        self.target_vel = np.array(
            config.get('target_velocity', [0.1, 0.05, 0.0]), dtype=float)
        self.world_bounds = float(config.get('world_bounds', 50.0))

        self.assignments: Dict[str, Task] = {}
        self.known_tasks: Dict[str, Task] = {}
        self.env = None

    # ---------- 配置 ----------
    @staticmethod
    def load_config(path: str) -> dict:
        try:
            with open(path, 'r', encoding='utf-8') as handle:
                return json.load(handle)
        except FileNotFoundError:
            print('[WARN] 未找到 %s，使用默认配置。' % path)
            return {}

    def _build_tasks(self) -> List[Task]:
        task_cfgs = self.config.get('tasks', [])
        if not task_cfgs:
            return [
                Task('Intercept_Left', np.array([-5.0, -5.0, -10.0]), 2),
                Task('Intercept_Right', np.array([5.0, -5.0, -10.0]), 2),
                Task('Block_Back', np.array([0.0, 5.0, -10.0]), 1),
            ]
        return [
            Task(item['name'], np.array(item['position'], dtype=float),
                 item.get('priority', 1))
            for item in task_cfgs
        ]

    # ---------- 状态 ----------
    def reset_start_positions(self) -> None:
        """按 seed 生成可复现的初始位置。"""
        radius = float(self.config.get('start_radius', 5.0))
        for auv in self.auvs.values():
            offset = self.rng.uniform(-radius, radius, size=3)
            auv.state = {'Location': offset, 'Energy': auv.energy}

    def _simulate_states(self) -> None:
        for auv in self.auvs.values():
            auv.state['Energy'] = auv.energy

    def _dry_run_move(self, agent_id: str, target_pos: np.ndarray,
                      step_size: float = 0.2) -> None:
        auv = self.auvs[agent_id]
        current = auv.position()
        direction = np.asarray(target_pos, dtype=float) - current
        dist = float(np.linalg.norm(direction))
        if dist > 1e-6:
            move = direction / dist * min(step_size, dist)
            auv.state['Location'] = (current + move).tolist()

    def _update_target(self) -> None:
        self.target_pos = self.target_pos + self.target_vel * self.dt
        for i in range(3):
            if abs(self.target_pos[i]) > self.world_bounds:
                self.target_vel[i] *= -1.0

    # ---------- HoloOcean ----------
    def init_holoocean(self) -> None:
        try:
            import holoocean  # type: ignore # noqa: F401
        except ImportError:
            print('[INFO] HoloOcean 未安装，进入 dry-run 模式。')
            self.env = None
            return
        scenario_dict = {
            'name': self.config.get('scenario_name', 'MultiAgentHunting'),
            'world': self.config.get('world', 'OpenWater'),
            'agents': [
                {
                    'name': ag_cfg['name'],
                    'agent_type': ag_cfg.get('agent_type', 'TorpedoAUV'),
                    # 必须带位置传感器，否则 tick() 里没有位置观测
                    'sensors': [
                        {'sensor_type': 'AcousticModem'},
                        {'sensor_type': 'LocationSensor'},
                    ],
                }
                for ag_cfg in self.config.get('agents', [])
            ],
        }
        self.env = holoocean.make(scenario_dict)
        print('[INFO] HoloOcean 环境已启动: %s' % scenario_dict['name'])

    # ---------- 主循环 ----------
    def _positions(self) -> List[np.ndarray]:
        return [auv.position() for auv in self.auvs.values()]

    def run(self, max_steps: Optional[int] = None) -> dict:
        steps = self.max_steps if max_steps is None else int(max_steps)
        if self.verbose:
            print('=' * 60)
            print(' 多 AUV 协同围捕场景启动')
            print(' AUV 数量: %d   任务数量: %d'
                  % (len(self.auvs), len(self.tasks)))
            print(' 通信延迟 %d ms   丢包率 %.0f%%   收缩比例 %.2f'
                  % (self.modem.delay * 1000.0,
                     self.modem.packet_loss * 100.0, self.closing_ratio))
            print('=' * 60)

        self.init_holoocean()
        if self.env is None and not self.auvs[next(iter(self.auvs))].state:
            self.reset_start_positions()

        is_captured = False
        done_steps = 0
        for step in range(steps):
            now = step * self.dt
            done_steps = step + 1
            self._update_target()

            if self.env is None:
                self._simulate_states()
            else:
                tick_result = self.env.tick()
                for name in self.auvs:
                    self.auvs[name].state = tick_result.get(name, {})

            # 1) 周期性拍卖，结果通过水声信道广播
            if step % self.auction_interval == 0:
                agent_info = {
                    name: {'position': auv.position(), 'energy': auv.energy}
                    for name, auv in self.auvs.items()
                }
                self.assignments = self.allocator.allocate(
                    agent_info, self.tasks)
                for agent_id, task in self.assignments.items():
                    self.modem.broadcast(
                        agent_id,
                        {'task': task.name,
                         'position': task.position.tolist()},
                        now)
                if self.verbose:
                    print('\n[Step %5d] 拍卖结果:' % step)
                    for agent_id, task in self.assignments.items():
                        print('   %s -> %s @ %s'
                              % (agent_id, task.name,
                                 task.position.tolist()))

            # 2) 只有收到广播的 AUV 才知道自己的任务
            for msg in self.modem.poll(now):
                payload = msg.payload
                self.known_tasks[msg.sender] = Task(
                    payload['task'],
                    np.array(payload['position'], dtype=float))

            # 3) 控制：朝「包抄点向目标收缩」后的位置移动
            ratio = closing_ratio_at(now, self.closing_ratio,
                                     self.closing_time)
            for agent_id, auv in self.auvs.items():
                task = self.known_tasks.get(agent_id)
                if task is None:
                    continue
                goal = intercept_point(task.position, self.target_pos, ratio)
                cmd = auv.controller.compute(goal, auv.position(), self.dt)
                auv.update_energy(self.dt, cmd,
                                  self.idle_drain_rate, self.move_drain_rate)
                if self.env is not None:
                    self.env.act(agent_id, cmd.tolist())
                else:
                    self._dry_run_move(agent_id, goal)

            # 4) 捕获判定
            if captured(self._positions(), self.target_pos,
                        self.capture_radius):
                is_captured = True
                if self.verbose:
                    print('\n[Step %5d] 捕获成功，所有 AUV 进入 %.1f m 范围'
                          % (step, self.capture_radius))
                break

            if self.verbose and step and step % 100 == 0:
                print('[Step %5d] 目标 %s  待投递消息 %d'
                      % (step, np.round(self.target_pos, 2).tolist(),
                         self.modem.pending()))

        if self.verbose:
            print('\n' + '=' * 60)
            print(' 场景结束: %s' % ('捕获成功 ✓' if is_captured else '超时结束'))
            print(' 最终目标位置: %s' % np.round(self.target_pos, 2).tolist())
            print(' 通信统计: 发送 %d 条，丢弃 %d 条'
                  % (self.modem.sent, self.modem.dropped))
            for agent_id, auv in self.auvs.items():
                print('   %s 位置 %s  电量 %.2f%%'
                      % (agent_id, np.round(auv.position(), 2).tolist(),
                         auv.energy))
            print('=' * 60)

        return {
            'steps': done_steps,
            'captured': is_captured,
            'assignments': {k: v.name for k, v in self.known_tasks.items()},
            'sent': self.modem.sent,
            'dropped': self.modem.dropped,
        }
