# train_consist_coupler

多编组列车纵向动力学与车钩力仿真。把 `train_ato_stop` 的单质点模型拆成 N 节车 +
N-1 个车钩，逐节积分，回答一个单质点模型回答不了的问题：**车与车之间的力有多大，
从哪来。**

## 结论摘要

同一条线路（2400 m 站间距、160 km/h），只改编组建模与指令传播：

| 工况 | 停车误差 | 最大压钩 | 最大拉钩 |
|---|---|---|---|
| 单质点（对照） | −0.020 m | 0 | 0 |
| 4 节，理想同步 | −0.020 m | 0 | 0 |
| 4 节，车头→车尾 60 ms | −0.020 m | +27.5 kN | −58.2 kN |
| 4 节，车尾→车头 60 ms | −0.002 m | +64.6 kN | −15.0 kN |
| 8 节，车尾→车头 100 ms | +0.132 m | +124.5 kN | −124.7 kN |

- **理想同步时车钩力严格为 0**：模型不会凭空造力，这条有单元测试守着；
- **头尾两种传播方向给出镜像的一对**，压钩与拉钩互换；
- **长编组超线性**：8 节 + 200 ms 到 166 kN，4 节 + 200 ms 只有 79 kN；
- **均匀坡度不进车钩**：坡度只通过"列车跨在变坡点两侧"才起作用，量级只有十几 kN。

峰值量级可以用 $\sqrt{k m/2}\cdot a\tau$ 手算核对：4 节 60 ms 估出 58.1 kN，仿真
−58.2 kN。

## 运行

不需要 ROS，也不需要 numpy / matplotlib（出图只用标准库 zlib）：

```bash
cd src/ground/train_consist_coupler
python3 -m train_consist_coupler.main --offline --output results
python3 -m unittest discover -s tests -v      # 29 项
```

`--quick` 只跑五个主场景，约 5 s；全量含扫描与变坡点实验，约 30 s。

ROS 2 回放：

```bash
cd ~/ros2_ws && colcon build --packages-select train_consist_coupler
source install/setup.bash
ros2 launch train_consist_coupler main.launch.py
ros2 launch train_consist_coupler main.launch.py scenario:=consist_long_tail
```

发布 `/train_consist/markers`、`/train_consist/coupler_force`、`/train_consist/status`。

## 结构

```
train_consist_coupler/
├── params.py      # 车辆、车钩、执行机构、控制、线路参数
├── consist.py     # 多质点状态、车钩力、加速度、积分
├── profile.py     # 参考速度曲线
├── sim.py         # ATO 指令、力分配、逐节延迟、闭环
├── scenarios.py   # 场景与实验定义
├── render.py      # 极简画布 + PNG 编码
├── font5x7.py     # 点阵字模
├── plots.py       # 五张图
├── ros_node.py    # ROS 2 回放节点
└── main.py        # 命令行入口
```

## 测试环境

Windows 11 / Python 3.13.14，未安装 rclpy、numpy、matplotlib。Ubuntu 22.04 +
ROS 2 Humble 下的 launch 需要本机 colcon 构建后验证。

## 已知限制

线性车钩（无缓冲器非线性吸能）、仅纵向、延迟取常数、牵引制动力按编组均分、
无再生制动与能耗。详见 `docs/ground/train_consist_coupler.md`。

## 声明

代码、图表与文档在 AI 辅助下完成，作者已逐行审查并跑通全部离线仿真与单元测试，
对提交内容负责。
