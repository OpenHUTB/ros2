# 多 AUV 协同围捕与动态任务分配

利用 HoloOcean 的多智能体支持和水声通信仿真 (Acoustic Modem)，
演示通信受限环境下的分布式拍卖算法 (Auction Algorithm) 与协同围捕策略。

## 模块结构

```
src/water/
├── main.py                    # 启动入口
├── multi_agent_hunting.py     # 核心实现
├── scenario.json              # 场景配置
└── README.md                  # 本文档
```

## 核心组件

| 组件 | 说明 |
|------|------|
| `PIDController`    | 三轴 PID 控制器，输出 6 维方向指令 |
| `AcousticModem`    | 水声通信模拟：固定延迟 + 丢包率 |
| `AuctionAllocator` | 拍卖算法：距离 + 剩余电量加权代价 |
| `AUVAgent`         | 单艘 AUV 封装：状态 + 能量管理 |
| `HuntingScenario`  | 场景管理：Holoocean 初始化 + 主循环 |

## 运行环境

- Python 3.10+
- HoloOcean 2.x（可选，未安装时自动进入 dry-run 模式）
- NumPy

安装依赖：

```bash
pip install numpy
# 可选：pip install holoocean
```

## 运行步骤

```bash
cd src/water
python main.py
```

## 可调节参数（scenario.json）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `max_steps`               | 1000   | 场景最大步数 |
| `auction_interval_steps`  | 50     | 拍卖触发周期（步） |
| `energy_weight`           | 0.3    | 电量在代价函数中的权重 |
| `comm_delay_ms`           | 200    | 水声通信延迟（毫秒） |
| `packet_loss`             | 0.05   | 丢包率（0 ~ 1） |
| `target_velocity`         | [0.15, 0.08, 0.02] | 逃逸目标速度 |

## 算法简述

拍卖算法每 `auction_interval_steps` 步触发一次：

1. 所有 AUV 向 `AuctionAllocator` 报告自身位置和剩余电量；
2. 任务按优先级排序，依次寻找代价最低的 AUV；
3. 代价函数 = 距离代价 − 电量奖励（权重 `energy_weight`）；
4. 分配结果通过 `AcousticModem` 广播，模拟水声信道延迟和丢包。

## Dry-run 模式

未安装 HoloOcean 时脚本会自动使用模拟数据，
可以直接演示拍卖算法和控制器的运行逻辑。
