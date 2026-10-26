# 数据格式样本 / Data schema samples

每个 CSV 含相应集合均匀抽取的 24 行，仅用于查看字段，不用于训练。完整数据由模块根目录的 `dataset.py` 重建；`dataset_summary.json` 中的校验值对应完整 CSV，而非本目录抽样文件。

Each CSV contains 24 uniformly selected rows from its split for schema inspection, not training. Regenerate complete data with `dataset.py` from the module root. Hashes in `dataset_summary.json` refer to the complete CSV files rather than these samples.

```bash
python dataset.py --seed 2026 --episodes 30 --output datasets
```

输入 / Features: `sensor_range_m`、`closing_speed_m_s`、`height_error_m`。

标签 / Target: `accel_m_s2`。

`range_m`、`speed_m_s`、`height_m`、`collision` 是物理评价信息；水平控制和学习输入不得使用前两者代替传感器输入。竖直位置由理想 `framepos` 传感器读出；本数据不包含测距噪声或姿态变化。

`range_m`, `speed_m_s`, `height_m` and `collision` are physics evaluation fields. Horizontal control and learning features must not replace rangefinder observations with true clearance or speed. An ideal `framepos` sensor provides vertical position; these data do not include range noise or attitude changes.

`episode_id` 标识回合；同一回合所有帧只属于一个集合。首帧缺少前一距离，闭合速度初始化为 0，使用时可通过 `time_s` 明确识别。

`episode_id` identifies the episode; all frames from an episode stay in one split. Initial closing speed is zero because there is no previous range sample. The initial frame is identifiable through `time_s`.
