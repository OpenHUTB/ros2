"""按完整轨迹划分监督数据。 / Split supervised samples by complete trajectories."""

import hashlib
import json
from pathlib import Path

import numpy as np

from .simulation import DT, MAX_RANGE, WarehouseSimulation, expert_target

FEATURE_NAMES = ([f"range_{i}_m" for i in range(9)] +
                 [f"closing_{i}_mps" for i in range(9)] + ["speed_mps", "cruise_mps"])


def features(ranges, previous, speed, cruise):
    """仅使用当前与上一帧，首帧闭合速度归零。 / Use current/past frames; reset first-frame rates."""
    closing = np.zeros(9) if previous is None else np.clip((previous - ranges) / DT, -10, 10)
    return np.concatenate([ranges, closing, [speed, cruise]]).astype(np.float32)


def collect(output, episodes=30, seed=42, duration=8.0):
    """轨迹独立采样，再按编号分组。 / Sample independent episodes and group by episode ID."""
    if episodes < 15 or episodes % 15 or not 0 < duration <= 120:
        raise ValueError("轨迹数须为15的正整数倍，时长须在(0,120]。 / Invalid episode count or duration.")
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    buckets = {name: [] for name in ("train", "validation", "test")}
    metadata = []
    for episode in range(episodes):
        rng = np.random.default_rng(np.random.SeedSequence([seed, episode]))
        scenario = ("static", "crossing", "narrow")[episode % 3]
        # 每组五条轨迹分为3/1/1，三个场景均进入每个分区。 / Stratify each scenario 3/1/1.
        slot = (episode // 3) % 5
        split = "train" if slot < 3 else ("validation" if slot == 3 else "test")
        cruise = float(rng.uniform(0.6, 2.4))
        obstacle_x = float(rng.uniform(3.0, 7.5))
        crossing_speed = float(rng.uniform(0.4, 1.2))
        sim = WarehouseSimulation(scenario, obstacle_x, cruise, crossing_speed)
        previous = None
        for step in range(round(duration / DT)):
            ranges = sim.ranges()
            speed = float(sim.data.qvel[0])
            target, risk = expert_target(ranges, speed, cruise)
            buckets[split].append((features(ranges, previous, speed, cruise), target, risk, episode))
            previous = ranges.copy()
            # 混合巡航与制动行为，包含接近障碍的高风险样本。 / Mix cruise and braking behavior.
            command = cruise if episode % 2 == 0 else target
            sim.step(command)
            if sim.collision:
                break
        metadata.append({"id": episode, "split": split, "scenario": scenario,
                         "cruise_mps": cruise, "obstacle_x_m": obstacle_x,
                         "crossing_speed_mps": crossing_speed, "collision": bool(sim.collision)})
    manifest = {"seed": seed, "episodes": metadata, "features": FEATURE_NAMES,
                "dt_s": DT, "range_limit_m": MAX_RANGE,
                "risk_classes": {"0": "clear", "1": "caution", "2": "brake"},
                "label_source": "stopping_distance_reference", "splits": {}}
    for split, rows in buckets.items():
        x = np.stack([r[0] for r in rows])
        target = np.asarray([r[1] for r in rows], dtype=np.float32)
        risk = np.asarray([r[2] for r in rows], dtype=np.int64)
        ids = np.asarray([r[3] for r in rows], dtype=np.int64)
        np.savez_compressed(output / f"{split}.npz", x=x, target_speed=target, risk=risk, episode=ids)
        digest = hashlib.sha256()
        for array in (x, target, risk, ids):
            digest.update(array.tobytes())
        manifest["splits"][split] = {"samples": len(x), "episode_ids": sorted(set(ids.tolist())),
            "risk_counts": np.bincount(risk, minlength=3).tolist(), "array_sha256": digest.hexdigest()}
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    audit(output)
    return manifest


def audit(output):
    """检查轨迹隔离、有限值和物理量范围。 / Check isolation, finite values and physical ranges."""
    seen = set()
    report = {}
    for split in ("train", "validation", "test"):
        with np.load(Path(output) / f"{split}.npz", allow_pickle=False) as data:
            x, risk, ids, target = (data[k] for k in ("x", "risk", "episode", "target_speed"))
            episode_ids = set(ids.tolist())
            if seen & episode_ids:
                raise ValueError("轨迹跨分区重复。 / Episode leakage detected.")
            seen |= episode_ids
            valid = (len(x) > 0 and x.shape == (len(risk), 20) and len(ids) == len(x)
                     and np.isfinite(x).all() and np.isfinite(target).all()
                     and np.all((x[:, :9] >= 0) & (x[:, :9] <= MAX_RANGE))
                     and np.all(np.abs(x[:, 9:18]) <= 10) and np.all(np.isin(risk, [0, 1, 2]))
                     and np.all((target >= 0) & (target <= x[:, 19] + 1e-6)))
            if not valid:
                raise ValueError("数据形状或取值无效。 / Invalid dataset values or shape.")
            report[split] = {"samples": len(x), "episodes": len(episode_ids)}
    return report
