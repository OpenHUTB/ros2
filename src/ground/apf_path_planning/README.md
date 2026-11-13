# 人工势场法（APF）路径规划

基于 Khatib (1986) 的反应式规划：目标产生引力、障碍物产生斥力，机器人沿合力方向步进。纯 pygame 2D 可视化，含势场热图、路径动画、合力箭头与演示 gif 录制。

## 算法原理

### 核心思想

将机器人视为在人工势场中运动的质点：目标位置产生引力场（势能随距离增大），障碍物产生斥力场（势能随接近增大）。机器人沿总势能的负梯度方向运动，自然趋向目标、避开障碍。

### 引力势函数（分段）

| 区域 | 条件 | 公式 | 梯度幅值 |
|---|---|---|---|
| 近场 | d ≤ d₀ | U = ½·k·d² | k·d（线性增长） |
| 远场 | d > d₀ | U = k·d₀·d - ½·k·d₀² | k·d₀（恒定） |

分段设计避免远距离引力过大导致步进发散，是 APF 的经典改进。

### 斥力势函数

障碍物（圆心 + 半径）影响范围 [0, d_thresh] 内：

- U = ½·k_rep·(1/d - 1/d_thresh)² ，d 为机器人到障碍物边界距离
- d > d_thresh 时 U = 0（无影响）
- d ≤ 0（穿入障碍物）时 U = +∞

### 合力与步进

F = -∇U_att - ∇U_rep，归一化为单位向量后乘步长 step_size，保证步长恒定。

### 局部最小值逃逸

APF 经典缺陷：障碍物正后方会形成势阱（引力与斥力抵消），机器人停在原地或来回抖动。本实现用**距目标的历史最优距离**做停滞判据：只要最优距离连续 `escape_patience` 步没有被刷新，就判定为陷入势阱，注入一次随机扰动（`escape_jitter`）并重新观察。

判据特意不用「这一步是否更近」：2-周期抖动里每隔一步就有一步更近，那种写法会把计数反复清零，永远攒不满阈值。逃逸是**有限次随机尝试**——单障碍正挡在起点—目标连线上这类势阱能稳定跳出（有单测覆盖），U 形深陷阱仍可能 `timeout`，这是 APF 加简单随机扰动的已知局限。

## 源码结构

```
apf_path_planning/
├── apf_path_planning/
│   ├── __init__.py
│   ├── apf.py           # 核心算法（势函数、梯度、合力、步进、规划、逃逸）
│   ├── config.py        # 画布、颜色、默认场景、APF 参数
│   └── main.py          # pygame 可视化与 gif 录制
├── launch/
│   └── main.launch.py   # ROS 2 launch
├── test/
│   └── test_apf.py      # 离线单测 41 项
├── config/
│   └── params.yaml
├── resource/
├── setup.py
├── package.xml
├── main.py / main.sh / main.bat   # 根目录三入口
└── README.md
```

## 运行方式

### 1. 直接运行

```bash
pip install pygame pillow pyyaml
python main.py                  # 交互模式
python main.py --record         # 录制演示 gif
```

### 2. ROS 2 运行

```bash
cd ~/ros2_ws
colcon build --packages-select apf_path_planning --symlink-install
source install/setup.bash

ros2 run apf_path_planning main
```

## 测试

```bash
cd apf_path_planning
python -m pytest test/ -v   # 或 python test/test_apf.py
```

41 项离线单测覆盖：参数校验、距离函数、障碍物距离、引力势函数（目标处为零/近场二次/远场锥形/阈值连续性）、引力梯度（方向/幅值/远场恒定）、斥力势函数（障碍外为零/影响内为正/穿入为 inf）、斥力梯度方向、合力（无障碍等于引力/指向目标）、步进（趋向目标/步长上界）、规划（无障碍到达/侧方障碍避让/多障碍全避让）、局部最小值逃逸（单障碍挡路到达/2-周期抖动触发扰动/patience 阈值/直线前进不误触发/U 形陷阱确实尝试逃逸）、总势能。

## 大模型使用声明

本项目在开发过程中使用了大语言模型辅助代码编写与文档生成，具体用于：

- 人工势场法势函数与梯度公式的 Python 实现
- pygame 可视化与势场热图绘制代码框架
- 单元测试用例设计
- 文档撰写

核心算法逻辑（分段引力势、斥力势、负梯度合力、局部最小值逃逸）均基于经典教材与公开文献（Khatib 1986）实现，AI 辅助部分经过人工审核与虚拟机实测验证。

## License

MIT