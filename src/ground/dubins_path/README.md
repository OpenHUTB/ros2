# Dubins 曲线轨迹生成与可视化

带运动学约束的最短可行路径生成，给定起末位姿与最小转弯半径，枚举 6 种基本组合（LSL/LSR/RSL/RSR/RLR/LRL）的最短 Dubins 路径，纯 pygame 2D 可视化。

## 算法原理

### 问题定义

给定起点 (x0, y0, yaw0) 和终点 (x1, y1, yaw1) 及最小转弯半径 R，求带曲率约束 |κ| ≤ 1/R 的最短可行路径。

### 6 种基本组合

| 类型 | 组合 | 几何构造 |
|---|---|---|
| CSC | LSL | 左左圆外公切线 |
| CSC | RSR | 右右圆外公切线 |
| CSC | LSR | 左右圆内公切线 |
| CSC | RSL | 右左圆内公切线 |
| CCC | LRL | 中间反向相切圆 |
| CCC | RLR | 中间反向相切圆 |

L = 左转圆弧（逆时针）、R = 右转圆弧（顺时针）、S = 直线段。

### 圆心计算

- 左转圆心 = (x - R·sin yaw, y + R·cos yaw)
- 右转圆心 = (x + R·sin yaw, y - R·cos yaw)

### 公切线

- 外公切线（LSL/RSR）：等径圆切线方向 = 圆心连线方向
- 内公切线（LSR/RSL）：`sin(dir-base) = s·2R/dist`，s 随 turn1 翻转
  （L,R 取 +1；R,L 取 -1），要求 dist ≥ 2R
- CCC（LRL/RLR）：中间圆有两个候选（中垂线 ±），两者都合法，取总长较短者

### 弧长

单段弧长 = (角度差 mod 2π) · R，限定 [0, 2πR)

## 源码结构

```
dubins_path/
├── dubins_path/
│   ├── __init__.py
│   ├── config.py         # 参数配置
│   ├── dubins.py         # 核心算法（圆心、公切线、6 组合、最短路径、采样）
│   └── main.py           # pygame 可视化与录制
├── launch/
│   └── main.launch.py    # ROS 2 launch
├── test/
│   └── test_dubins.py    # 离线单测 36 项（同目录 conftest.py 让 pytest 直接可用）
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
pip install pygame numpy pillow
python main.py                  # 交互模式
python main.py --record         # 录制演示 gif
```

### 2. ROS 2 运行

```bash
cd ~/ros2_ws
colcon build --packages-select dubins_path --symlink-install
source install/setup.bash

ros2 run dubins_path main

# 无显示设备（VM / CI）用 launch 切到离屏录制
ros2 launch dubins_path main.launch.py mode:=record output:=/tmp/dubins_demo.gif
```

## 参数配置

`config/params.yaml` 会被真正读取（`config.load_params()`），查找顺序为
`DUBINS_PARAMS` 环境变量 → ROS 2 安装空间 `share/dubins_path/config/` → 源码树 `config/`。
可配置画布尺寸/FPS、起末位姿、最小转弯半径与采样步数。

## 测试

```bash
cd dubins_path
python -m pytest test/ -v          # 或 python test/test_dubins.py
```

36 项离线单测覆盖：mod2pi 边界、6 种组合非负、最短路径类型与长度、最短 ≤ 候选、缩放相似性、起末位姿对齐、双方向端点对齐、采样点数量、_advance 各段运动学，以及打包自检与 params.yaml 生效性。

## 大模型使用声明

本项目在开发过程中使用了大语言模型辅助代码编写与文档生成，具体用于：

- Dubins 曲线几何构造算法的 Python 实现
- pygame 可视化代码框架
- 单元测试用例设计
- 文档撰写

核心算法逻辑（圆心构造、公切线方向、弧长积分）均基于经典教材与公开文献实现，AI 辅助部分经过人工审核与虚拟机实测验证。

## License

MIT