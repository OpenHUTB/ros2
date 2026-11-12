"""生成可复现的间隙样本并训练基础策略。"""

import numpy as np

from carla_nn_roundabout_entry.policy import GapPolicy


def training_data(seed: int = 7, samples: int = 500) -> tuple[np.ndarray, np.ndarray]:
    rng = np.random.default_rng(seed)
    gap = rng.uniform(0.0, 20.0, samples)
    ttc = rng.uniform(0.0, 12.0, samples)
    speed = rng.uniform(0.0, 12.0, samples)
    lateral = rng.uniform(-4.0, 4.0, samples)
    features = np.column_stack((gap / 20.0, ttc / 12.0, speed / 12.0, lateral / 4.0))
    labels = ((gap >= 6.0 + speed * 0.35) & (ttc >= 3.0) & (np.abs(lateral) < 3.2)).astype(int)
    return features, labels


def main() -> None:
    features, labels = training_data()
    policy = GapPolicy(seed=7)
    accuracy = policy.train(features, labels)
    print(f"训练样本数: {len(labels)}; 准确率: {accuracy:.3f}")
    print(f"安全间隙样例: {policy.decide(np.array([0.7, 0.8, 0.2, 0.0]))}")


if __name__ == "__main__":
    main()
