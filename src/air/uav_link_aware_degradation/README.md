# uav_link_aware_degradation

通信链路质量驱动的无人机分级降级与失联自主返航。

源码位置：`src/air/uav_link_aware_degradation/`，文档页：
<https://openhutb.github.io/ros2/air/uav_link_aware_degradation/>。

## 它解决什么

现有模块（遥测诊断、高度限幅、电池 RTH）都在“机体/任务”一层做安全；
本模块把“**遥控链路本身**”作为一等公民：

- 接收端只看得到的量：命令年龄（age）、滑窗投递率（pdr）、抖动；
- 状态机 `NORMAL → DEGRADED → RETURN → LAND` 带滞回，避免单次丢包打翻模式；
- 与两种常见做法对照：`hold_last`（沿用最后指令，很多地面站默认）和
  `hover`（超时置零悬停，`uav_mujoco_sim` 的 `cmd_timeout` 行为）。

## 目录

```
uav_link_aware_degradation/
├── core/
│   ├── link.py        # 链路损伤模型：延迟/抖动/Gilbert-Elliott 丢包/计划中断
│   ├── mission.py     # 航点任务与进度
│   ├── supervisor.py  # 四态降级状态机（纯逻辑，可单测）
│   ├── policies.py    # 遥控端、机载跟踪律、两种对照策略
│   ├── sim.py         # MuJoCo 闭环 + 几何净空统计
│   ├── experiment.py  # 实验矩阵：4 档链路 × 3 策略 × 多种子
│   └── render.py      # 离屏渲染对比动图与指标图
├── ros/               # ROS2 节点（link_emulator / supervisor）
├── launch/            # link_degradation.launch.py
├── config/            # link_profiles.yaml
└── test/              # pytest 纯逻辑测试
```

## 运行

```bash
# 纯逻辑自检（不需要 MuJoCo）
python3 main.py selftest

# 完整实验矩阵
python3 main.py run --out results --seeds 3

# 单场景 + 对比动图 + 指标图
python3 main.py demo --out results --profile outage \
    --gif results/link_degradation_demo.gif \
    --figure results/link_degradation_metrics.png
```

ROS2 联调（需要 `uav_mujoco_sim` 已 colcon build）：

```bash
ros2 launch uav_link_aware_degradation link_degradation.launch.py
```

## 测试口径

- `pytest test/` 6 项纯逻辑用例全部通过；
- 离线闭环 36 组（4 链路档 × 3 策略 × 3 种子）指标见
  `results/results.csv`；
- 判定阈值：净空 < 0.10 m 记为接触/撞墙。
