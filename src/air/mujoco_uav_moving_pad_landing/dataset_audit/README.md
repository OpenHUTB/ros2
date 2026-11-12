# 着陆数据集完整性报告

该工具为已有的 MuJoCo rollout 特征、专家指令和 episode 编号生成结构与范围摘要。它检查样本行数、有限值、分组拆分交集，并将特征和标签的逐列范围写入 JSON，便于复核训练数据。

```python
from pathlib import Path

from mujoco_uav_moving_pad_landing.dataset import generate_grouped_samples
from mujoco_uav_moving_pad_landing.dataset_audit import audit_grouped_dataset, write_audit_report
from mujoco_uav_moving_pad_landing.landing import expert_command

features, labels, episode_ids = generate_grouped_samples(
    expert_command, samples=120, steps_per_episode=10, seed=17
)
report = audit_grouped_dataset(features, labels, episode_ids, test_size=0.2, seed=17)
write_audit_report(report, Path("output/dataset_audit.json"))
```

English: The report records sample and episode counts, per-column feature and label ranges, and confirms that the train/test split has no shared episode IDs.
