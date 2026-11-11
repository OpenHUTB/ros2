# 数据与扫描结果样例

`train_sample.csv`、`validation_sample.csv` 和 `test_sample.csv` 分别从对应数据集合均匀抽取 24 行，仅用于查看字段，不用于训练。完整数据可在模块根目录使用以下命令重建：

四个控制器评测指标的参考快照保存在 `sample_data/metrics/`。重新运行 `evaluate.py` 时，生成文件写入模块根目录的 `results/`，该目录由 Git 忽略。 / Reference snapshots for the four controller evaluation metrics are stored in `sample_data/metrics/`. Running `evaluate.py` writes fresh outputs to the module's root `results/` directory, which Git ignores.

```
python dataset.py --seed 2026 --episodes 30 --output datasets
```

`dataset_summary.json` 中的校验值对应完整 CSV，而非本目录中的抽样文件。输入特征为 `sensor_range_m`、`closing_speed_m_s`，标签为 `accel_m_s2`。

`range_m`、`speed_m_s`、`height_m`、`collision` 是物理评价信息；水平控制和学习输入不得使用真实间距或速度替代传感器输入。竖直位置由理想 `framepos` 传感器读出；样例不包含测距噪声或姿态变化。`episode_id` 标识回合，同一回合的帧只属于一个集合；首帧因缺少前一距离，闭合速度初始化为 0。

`scenario_sweep.csv` 是测距控制器在 4 组场景中的实际扫描结果，每行对应一组起点、障碍位置、初速度和飞行高度。可在模块根目录用以下命令重建 JSON 汇总和 CSV 表格：

```
python scenario_sweep.py --controller range --start-x=-8,-7 --wall-x 0 --speed 0,1.5 --height 1.2 --output results/range_sweep.json --csv-output sample_data/scenario_sweep.csv
```

`scenario_sensitivity.json` 从扫描 CSV 后处理生成，按起点、障碍位置、初速度和高度分别汇总平均/最差间距、样本碰撞占比，并按分组平均间距跨度排序。每个参数单独分组，其余参数合并；碰撞占比只描述当前扫描样本，不是实际碰撞概率。可用以下命令重建：

```
python scenario_analysis.py sample_data/scenario_sweep.csv --output sample_data/scenario_sensitivity.json
```
