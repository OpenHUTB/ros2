# Bezier 曲线轨迹生成与可视化

基于 de Casteljau 算法与 Bernstein 多项式的 Bezier 曲线生成，附 B 样条对比、曲率分析与弧长计算，纯 pygame 2D 可视化。

## 算法原理

### Bezier 曲线

给定 n+1 个控制点 P₀, P₁, ..., Pₙ，n 次 Bezier 曲线定义为：

```
B(t) = Σ C(n,i) (1-t)^(n-i) t^i P_i,  t ∈ [0, 1]
```

本项目实现两种等价求值方式：

- **de Casteljau 递推算法**：通过逐级线性插值得到曲线点，数值稳定性好
- **Bernstein 多项式形式**：直接用基函数加权求和

### 关键性质

| 性质 | 说明 |
|---|---|
| 端点插值 | B(0)=P₀, B(1)=Pₙ |
| 凸包性 | 曲线完全在控制点的凸包内 |
| 导数 | B'(t) 的控制点 = 原控制点差分 × n |
| 曲率 | κ = \|x'y'' - y'x''\| / (x'²+y'²)^(3/2) |

### B 样条对比

3 次 B 样条使用 Cox-de Boor 递归基函数，与 Bezier 的区别：

- B 样条不通过中间控制点（局部性）
- B 样条有局部支撑性（改一个控制点只影响附近曲线段）

> 端点注意：基函数支撑为半开区间 `[u_i, u_{i+1})`，参数取到 `u_max` 会让所有基函数为 0、
> 返回值退化成 `(0, 0)`。`bspline_point()` 内部把参数夹到 `[knots[degree], knots[-degree-1])`，
> clamped 样条因此在末点收敛到最后一个控制点（有单测钉住）。

## 参数配置

`config/params.yaml` 会被真正读取（`config.load_params()`），查找顺序为
`BEZIER_PARAMS` 环境变量 → ROS 2 安装空间 `share/bezier_trajectory/config/` → 源码树 `config/`。
可配置 `control_points`、`bspline_control_points`、`curve_samples`、`curvature_samples`、`anim_steps`。

## 源码结构

```
bezier_trajectory/
├── bezier_trajectory/
│   ├── __init__.py
│   ├── config.py         # 参数配置
│   ├── bezier.py         # 核心算法（de Casteljau、Bernstein、导数、曲率、弧长、B 样条）
│   └── main.py           # pygame 可视化与录制
├── launch/
│   └── main.launch.py    # ROS 2 launch
├── test/
│   └── test_bezier.py    # 离线单测 24 项（同目录 conftest.py 让 pytest 直接可用）
├── config/
│   └── params.yaml
├── resource/
├── setup.py
├── package.xml
├── main.py / main.sh / main.bat   # 根目录三入口
└── README.md
```

## 运行方式

### 1. 直接运行（最简单）

```bash
pip install pygame numpy pillow
python main.py                  # 交互模式
python main.py --record         # 录制演示 gif
```

### 2. ROS 2 运行

```bash
# 编译
cd ~/ros2_ws
colcon build --packages-select bezier_trajectory --symlink-install
source install/setup.bash

# 运行
ros2 run bezier_trajectory main

# 无显示设备（VM / CI）用 launch 切到离屏录制
ros2 launch bezier_trajectory main.launch.py mode:=record output:=/tmp/bezier_demo.gif
```

## 测试

```bash
cd bezier_trajectory
python -m pytest test/ -v          # 或 python test/test_bezier.py
```

24 项离线单测覆盖：组合数、端点插值、de Casteljau 与 Bernstein 一致性、导数端点性质、曲率（直线为零/曲线非零）、弧长（直线/曲线下界）、采样计数、B 样条基函数非负性与凸包性、clamped B 样条端点行为（末点=末控制点）、params.yaml 键生效。

## 大模型使用声明

本项目在开发过程中使用了大语言模型辅助代码编写与文档生成，具体用于：

- Bezier 曲线算法的 Python 实现与数值计算
- pygame 可视化代码框架
- 单元测试用例设计
- 文档撰写

核心算法逻辑（de Casteljau 递推、Bernstein 多项式、Cox-de Boor 递归）均基于经典教材与公开文献实现，AI 辅助部分经过人工审核与虚拟机实测验证。

## License

MIT
