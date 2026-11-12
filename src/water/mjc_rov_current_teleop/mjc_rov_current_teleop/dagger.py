"""
dagger.py —— DAgger（Dataset Aggregation）数据聚合，缓解行为克隆的分布偏移。

为什么需要它：
  原始 BC 只见过"专家轨迹"上的状态。策略一旦漂到专家没去过的状态，
  输入分布就与训练集不一致，误差会累积——文档记录的 MLP 只到达 2/4 航点、
  专家 4/4，正是这个现象的体现。

DAgger 的做法：
  第 0 轮用专家本身跑闭环（等价于原始 BC 数据集）；
  之后每轮用**当前策略**跑闭环，把策略实际访问到的每个状态交给专家标注，
  再把新样本并入数据集重训。策略越多犯错，数据集就越覆盖这些区域。

本文件只依赖 numpy 在模块层导入，torch / mujoco 在函数内惰性导入，
因此 :class:`DaggerBuffer` 与其单元测试不需要 MuJoCo 或 PyTorch。
"""
import json

import numpy as np


class DaggerBuffer:
    """跨轮次累积 ``(特征, 专家动作)`` 样本。"""

    def __init__(self):
        self._features = []
        self._actions = []
        self.rounds = []

    def add(self, features, actions):
        """并入一轮样本，返回这一轮新增的条数。"""
        features = np.asarray(features, dtype=np.float32).reshape(-1, 6)
        actions = np.asarray(actions, dtype=np.float32).reshape(-1, 2)
        if features.shape[0] != actions.shape[0]:
            raise ValueError('features and actions must have the same length')
        if features.shape[0] == 0:
            raise ValueError('cannot add an empty round')
        self._features.append(features)
        self._actions.append(actions)
        self.rounds.append(int(features.shape[0]))
        return int(features.shape[0])

    def __len__(self):
        return int(sum(self.rounds))

    def arrays(self):
        """拼接后的 ``(X, Y)``；空缓冲区返回形状正确的零长度数组。"""
        if not self.rounds:
            return (np.zeros((0, 6), dtype=np.float32),
                    np.zeros((0, 2), dtype=np.float32))
        return (np.concatenate(self._features),
                np.concatenate(self._actions))

    def summary(self):
        return {'rounds': list(self.rounds), 'total': len(self)}


def rollout_and_label(params, net=None, horizon=6000):
    """用当前策略跑闭环，并在**访问到的每个状态**上记录专家动作。

    ``net is None`` 时用专家自身驱动，即第 0 轮（等价于原始 BC 采集）。
    返回 ``(features, expert_actions)``，两者形状为 ``(horizon, 6)`` / ``(horizon, 2)``。
    """
    import numpy as np  # noqa: F401  (保持与下方 torch 依赖一致的可读性)
    import torch

    from . import cruise_net

    model, data, dyn = cruise_net._make_sim(params)
    dt = model.opt.timestep
    waypoints = cruise_net.EXPERT_WAYPOINTS
    features, actions = [], []
    waypoint_index = 0

    for step in range(horizon):
        t = step * dt
        state = cruise_net._read_state(model, data)
        waypoint = waypoints[waypoint_index]
        if np.hypot(waypoint[0] - state[0], waypoint[1] - state[1]) < 0.4:
            waypoint_index = (waypoint_index + 1) % len(waypoints)
            waypoint = waypoints[waypoint_index]

        feature = cruise_net.make_features(state, waypoint)
        expert = cruise_net._expert_cmd(state, waypoint)
        features.append(feature)
        actions.append(expert)

        if net is None:
            command = expert
        else:
            with torch.no_grad():
                command = net(torch.tensor(feature).unsqueeze(0)).squeeze(0).numpy()

        twist = np.array([command[0], command[1], 0.0, 0.0, 0.0, 0.0])
        dyn.apply(data, t, model.body('rov').id, twist)
        cruise_net.mujoco_step(model, data)

    return np.stack(features), np.stack(actions)


def train_net(features, actions, epochs=200, lr=1e-3, seed=0):
    """在聚合数据集上从头训练；返回 ``(net, final_mse)``。"""
    import torch

    from . import cruise_net

    torch.manual_seed(seed)
    x = torch.tensor(np.asarray(features, dtype=np.float32))
    y = torch.tensor(np.asarray(actions, dtype=np.float32))
    net = cruise_net.CruiseMLP()
    optimiser = torch.optim.Adam(net.parameters(), lr=lr)
    loss_fn = torch.nn.MSELoss()
    loss_value = float('nan')
    for _ in range(epochs):
        optimiser.zero_grad()
        loss = loss_fn(net(x), y)
        loss.backward()
        optimiser.step()
        loss_value = float(loss.item())
    return net, loss_value


def run_dagger(params, iterations=4, horizon=6000, epochs=200, seed=0,
               eval_horizon=12000):
    """跑 ``iterations + 1`` 轮 DAgger，返回逐轮报告。

    第 0 轮的策略只见过专家轨迹；之后每轮都在策略自己访问到的状态上补标注，
    因此 ``visited_waypoints`` 与 ``mae_position_m`` 随轮次的变化就是收益证据。
    """
    import torch

    from . import common, cruise_net

    report = {'seed': seed, 'iterations': [], 'rounds': []}
    buffer = DaggerBuffer()
    net = None

    for iteration in range(iterations + 1):
        features, actions = rollout_and_label(params, net, horizon=horizon)
        buffer.add(features, actions)
        report['rounds'].append(buffer.summary())

        x, y = buffer.arrays()
        net, train_mse = train_net(x, y, epochs=epochs, seed=seed + iteration)

        metrics = cruise_net.evaluate(params, horizon=eval_horizon, use_net=True, net=net)
        entry = {
            'iteration': iteration,
            'samples': len(buffer),
            'train_mse': round(train_mse, 6),
            'mae_position_m': metrics['mae_position_m'],
            'max_err_m': metrics['max_err_m'],
            'waypoints_hit': len(metrics['visited_waypoints']),
            'all_waypoints_hit': metrics['all_waypoints_hit'],
        }
        report['iterations'].append(entry)
        print(f"[dagger] iter {iteration}: samples={entry['samples']} "
              f"MAE={entry['mae_position_m']}m "
              f"waypoints={entry['waypoints_hit']}/{len(cruise_net.EXPERT_WAYPOINTS)} "
              f"all_hit={entry['all_waypoints_hit']}")

    root = common.get_package_root()
    weights_dir = root / 'weights'
    weights_dir.mkdir(exist_ok=True)
    weights_path = weights_dir / 'cruise_mlp_dagger.pt'
    torch.save({'model': net.state_dict(), 'report': report}, weights_path)
    report_path = root / 'data' / 'dagger_report.json'
    report_path.parent.mkdir(exist_ok=True)
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2),
                           encoding='utf-8')
    print(f'[dagger] 权重: {weights_path}')
    print(f'[dagger] 报告: {report_path}')
    return report
