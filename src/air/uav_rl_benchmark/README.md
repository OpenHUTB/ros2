# uav_rl_benchmark

无人机导航的**强化学习对比实验**包：在**完全相同的环境与接口**下对比

1. **同模型不同 RL 算法** —— PPO / SAC / TD3
2. **同算法不同网络结构** —— MLP 宽度（`[64,64]` vs `[256,256]`）与深度（`[64,64]` vs `[64,64,64]`）

环境直接复用 `uav_ppo_nav` 的 `NavEnv`（**不复制、不改动**），因此比出来的差异只反映
算法或结构的差别，而不是"环境不一样"。

## 为什么指标不只看成功率

只比成功率会漏掉两类实际不可用的策略：**能到但绕远路**、**能到但抖得厉害**。
所以每个配置都记录：

| 指标 | 含义 |
|---|---|
| `success_rate` / `crash_rate` / `timeout_rate` | 到达 / 碰撞或越界 / 超时（三者之和 = 1） |
| `mean_steps` | 成功与失败的回合平均步数 |
| `path_efficiency` | **起点到目标的直线距离 / 实际飞行路程**（0~1，越接近 1 越直；原地不动记 0） |
| `action_smoothness` | 相邻两步动作的 L2 距离均值（越小越平滑） |
| `train_time_s` / `train_fps` | 训练墙钟时间与吞吐（不同算法的**训练成本**差异很大） |

## 公平性约定

* 同一学习率 `3e-4`、同一折扣 `0.99`、同一批大小 `256`；
* **同一网络结构** —— SB3 的 SAC/TD3 默认是 `[256,256]`，若不统一，差异里会混进"网络更大"这个因素；
* **同一步数预算** —— 三种算法都训练 100k 步（对比的是**样本效率**，不是"谁训得久"）；
* **同一批评估任务** —— 固定 `seed` 的 100 个回合，全部用**确定性动作**（取策略均值，不采样）；
* 每个配置重复 **3 个随机种子**，报告里给均值 ± 标准差。

## 快速开始

```bash
pip install "stable-baselines3" "gymnasium" numpy pandas matplotlib

cd src/air/uav_rl_benchmark/scripts

# 完整对比矩阵（3 算法 × 3 结构 × 3 种子）
python3 main.py --sweep

# 只跑一组
python3 main.py --algo sac --arch 64,64 --seeds 0 --steps 100000

# 由已有结果重新生成表格与图表（不重新训练）
python3 main.py --report
```

也可以按 ROS 方式启动（后台跑对比并在 `/benchmark/status` 播报进度）：

```bash
roslaunch uav_rl_benchmark main.launch
roslaunch uav_rl_benchmark main.launch steps:=200000
```

> ⚠️ 这是**离线训练任务**，需要 `torch` / `stable-baselines3`。只装了 ROS 的机器上，
> 节点会在 `/benchmark/status` 报 `ERROR` 并退出（**不会静默失败**）。

## 产出

```
results/
├── results.csv                     每个配置一行（原始数据）
├── report.md                       汇总表格 + 结论要点（可直接粘进文档）
├── fig_success_by_algo.png         不同算法成功率（带标准差误差棒）
├── fig_success_by_arch_ppo.png     不同网络结构成功率
├── fig_learning_curves.png         成功率随训练步数的变化（样本效率）
├── fig_cost.png                    训练耗时对比
├── fig_tradeoff.png                路径效率 vs 动作平滑度
└── runs/<algo>_<arch>_seed<k>/
    ├── eval_log.json               周期评估曲线（用于画学习曲线）
    ├── metrics.json                该配置的最终指标
    └── best_model.zip              按成功率保存的最优模型（不入库）
```

`results/runs/` 下的模型文件**不入库**（可再训练得到），只提交 `results.csv`、`report.md` 与图表。

## 本地测试（无需 ROS / 仿真器 / torch）

```bash
python3 tests/test_rl_benchmark_local.py     # 45 项
```

其中「评估指标计算」一节用**桩策略**驱动真实 `NavEnv`：一个永远悬停（必然超时）、
一个最简单的比例控制器（无障碍时应能到达），用来验证成功率/超时率/路径效率/
动作平滑度**算得对不对** —— 不需要训练框架即可回归。

还有一节 `run_sweep 参数透传`，专门盯住一个曾经出过的坑：`main.py --sweep` 给
`run_sweep` 传了 `eval_freq`，但 `run_sweep` 签名里没有它 —— **命令直接 TypeError 崩溃**；
而且即便不崩，配置里的 `eval.freq` 也会被 `run_one` 的默认值顶掉，让配置文件形同虚设。
现在这条调用路径有回归测试（猴子补丁住 `run_one`，断言参数被原样透传）。

> **依赖说明**：45 项里有 4 项需要 `pandas`（报告汇总那节）。没装 pandas 时那 4 项会跳过，
> 输出「通过 41 项」并给出提示；其余只依赖 `numpy`。

