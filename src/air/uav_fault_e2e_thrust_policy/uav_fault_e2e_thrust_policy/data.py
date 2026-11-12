"""DAgger 数据采集。

第一轮（``round=0``）用专家飞，采集专家的状态分布；之后每一轮用"当前策略飞、
专家在策略飞到的地方打标签"的方式聚合数据。这就是 DAgger 的核心：
训练分布要跟着策略自己的错误走，否则策略一旦飞偏，专家的示范里根本没有那种状态。

每个样本存成 ``(深度图, 本体量, 专家指令)``，再带上 episode 编号与步号，
训练时按步号拼"最近三帧"，保证堆叠通道不会跨 episode 串味。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from . import obs as O
from . import policy as P
from .env import ThrustEnv
from .expert import TaskConfig
from .faults import FaultPlan

# 训练用的故障工况：健康 + 三种可恢复故障（单桨全失效物理上不可恢复，不放进训练集）
TRAIN_CASES = (
    ("healthy", dict(kind="healthy")),
    ("degradation_0.5@r2", dict(kind="degradation", rotor=2, onset=2.5, severity=0.5)),
    ("degradation_0.3@r1", dict(kind="degradation", rotor=1, onset=3.0, severity=0.3)),
    ("stuck@r0", dict(kind="stuck", rotor=0, onset=2.5, severity=0.8)),
    ("progressive@r3", dict(kind="progressive", rotor=3, onset=2.0, severity=0.15)),
)

EVAL_CASES = TRAIN_CASES + (
    ("degradation_0.6@r0", dict(kind="degradation", rotor=0, onset=2.0, severity=0.6)),
    ("loss@r1", dict(kind="loss", rotor=1, onset=2.5)),
)


@dataclass
class EpisodeStat:
    label: str
    seed: int
    success: bool
    crashed: bool
    t: float
    steps: int
    min_altitude: float


DEFAULT_ONSET = 2.5           # 故障注入时刻（s），训练/评估默认一致


def make_plan(spec: dict, default_onset: float = 2.5) -> FaultPlan:
    spec = dict(spec)
    spec.setdefault("onset", default_onset)
    return FaultPlan(**spec)


def stack_frames(dataset: dict, stack: int = P.DEPTH_STACK) -> np.ndarray:
    """按 episode 拼最近 ``stack`` 帧：末通道是当前帧，往前依次取历史帧。

    到 episode 开头就停下、用该 episode 的第一帧补齐，**绝不让上一段飞行的
    画面串进来**（那会让策略以为走廊在动）。
    """
    depth = np.asarray(dataset["depth"], dtype=np.float32)
    episode = np.asarray(dataset["episode"], dtype=int)
    n = depth.shape[0]
    starts = np.zeros(n, dtype=int)
    for i in range(n):
        starts[i] = i if i == 0 or episode[i] != episode[i - 1] else starts[i - 1]
    out = np.empty((n, stack) + depth.shape[2:], dtype=np.float32)
    for i in range(n):
        for k in range(stack):            # k = 0 是最旧的一帧
            j = max(starts[i], i - (stack - 1 - k))
            out[i, k] = depth[j, 0]
    return out


MIN_RECORD_ALTITUDE = 1.0     # 低于这个高度状态就不可学（见 collect 注释）


def collect(api, policy_module=None, rounds: int = 3, episodes_per_case: int = 1,
            task: TaskConfig | None = None, seed: int = 0,
            cases=TRAIN_CASES, beta_start: float = 0.8, beta_end: float = 0.2,
            beta: float | None = None,
            expert_noise: float = 0.02,
            verbose: bool = True) -> tuple[dict, list[EpisodeStat]]:
    """跑 DAgger，返回 ``(数据集, 每轮统计)``。

    ``beta`` 是**听专家的概率**（0~1）：1.0 = 纯专家演示（纯 BC），0.0 = 完全由当前
    策略自己飞（那会立刻撞墙，因为策略还没见过偏离状态）。``policy_module=None`` 时
    无论 ``beta`` 多少都走专家。
    """
    task = task or TaskConfig()
    env = ThrustEnv(api, task=task, plan=make_plan(cases[0][1]), seed=seed)
    rng = np.random.default_rng(seed)
    depths, proprios, labels, ep_ids, steps = [], [], [], [], []
    stats: list[EpisodeStat] = []
    env_ref = env
    try:
        for r in range(max(rounds, 1)):
            if beta is not None:
                beta_used = float(beta)
            elif rounds <= 1:
                beta_used = 1.0          # 纯 BC：全部听专家的
            else:
                frac = r / max(rounds - 1, 1)
                beta_used = beta_start + (beta_end - beta_start) * frac
            for ci, (label, spec) in enumerate(cases):
                for k in range(episodes_per_case):
                    ep_seed = seed * 1000 + ci * 10 + k + r * 100
                    plan = make_plan(spec)
                    ob = env_ref.reset(seed=ep_seed, plan=plan)
                    history = [ob["depth"][-1].copy()] * P.DEPTH_STACK
                    done = False
                    min_alt = 1e9
                    n_steps = 0
                    while not done:
                        expert_cmd = env_ref.expert_command()
                        use_policy = (policy_module is not None
                                      and rng.random() > beta_used)
                        if use_policy:
                            act = P.predict(policy_module, {
                                "depth": P.stack_depth(history, policy_module),
                                "proprio": ob["proprio"][None]})[0]
                            act = np.clip(act, 0.0, 1.0)
                        else:
                            act = expert_cmd
                            if expert_noise > 0.0:
                                # 专家演示加一点噪声：让第一轮数据里就出现"稍微飞歪"
                                # 的状态，否则策略只见过完美轨迹，一偏离就崩（协变量漂移）。
                                act = np.clip(
                                    expert_cmd + rng.normal(0.0, expert_noise, 4), 0.0, 1.0)
                        # 只记录"还飞得起来"的状态：一旦策略把机体带到 1 m 以下，
                        # 专家在这一步的标签对训练没有意义（那种状态本身就是策略的
                        # 失控产物，学它只会把网络往沟里带）。
                        altitude = float(env_ref.quad.state().pos[2])
                        if altitude > MIN_RECORD_ALTITUDE:
                            depths.append(ob["depth"].copy())
                            proprios.append(ob["proprio"].copy())
                            labels.append(np.clip(expert_cmd, 0.0, 1.0))
                            ep_ids.append(r * 10000 + ci * 100 + k)
                            steps.append(n_steps)
                        history.append(ob["depth"][-1].copy())
                        ob, info = env_ref.step(act)
                        min_alt = min(min_alt, float(info.position[2]))
                        n_steps += 1
                        done = info.done
                    stats.append(EpisodeStat(
                        label=label, seed=ep_seed, success=bool(info.success),
                        crashed=bool(info.crashed), t=float(info.t),
                        steps=n_steps, min_altitude=min_alt))
                if verbose:
                    ok = sum(1 for s in stats if s.success)
                    print(f"  round {r} beta={beta_used:.2f} [{label}] "
                          f"累计 episode {len(stats)}，成功 {ok}")
    finally:
        env_ref.close()
    dataset = {
        "depth": np.asarray(depths, dtype=np.float32),
        "proprio": np.asarray(proprios, dtype=np.float32),
        "label": np.asarray(labels, dtype=np.float32),
        "episode": np.asarray(ep_ids, dtype=np.int64),
        "step": np.asarray(steps, dtype=np.int64),
    }
    return dataset, stats


def save_dataset(path, dataset: dict, stats: list[EpisodeStat] | None = None) -> None:
    import json
    from pathlib import Path

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = dict(dataset)
    payload["stats_json"] = np.array(
        json.dumps([s.__dict__ for s in (stats or [])], ensure_ascii=False))
    np.savez_compressed(path, **payload)


def load_dataset(path) -> dict:
    import json

    with np.load(path, allow_pickle=False) as data:
        out = {k: data[k] for k in data.files if k != "stats_json"}
        stats_json = str(data["stats_json"]) if "stats_json" in data.files else "[]"
    out["stats"] = json.loads(stats_json)
    return out


def split_by_episode(dataset: dict, fractions=(0.8, 0.1, 0.1), seed: int = 0):
    """按 episode 切分（同一段飞行里的相邻样本几乎一样，按样本切会虚高）。

    三段互不重叠：episode 太少时优先保证 train 非空，val/test 可以空。
    """
    episode = np.asarray(dataset["episode"], dtype=int)
    uniq = np.unique(episode)
    rng = np.random.default_rng(seed)
    order = rng.permutation(uniq.size)
    n = uniq.size
    n_train = max(1, int(round(fractions[0] * n)))
    n_val = int(round(fractions[1] * n))
    n_test = n - n_train - n_val
    if n_test < 0:                       # episode 太少：先从 val 挪一个给 test
        n_val = max(0, n_val + n_test)
        n_test = n - n_train - n_val
    if n_test < 0:
        n_val, n_test = 0, 0
        n_train = n
    groups = {
        "train": uniq[order[:n_train]],
        "val": uniq[order[n_train:n_train + n_val]],
        "test": uniq[order[n_train + n_val:n_train + n_val + n_test]],
    }
    return {k: np.isin(episode, v) for k, v in groups.items()}
