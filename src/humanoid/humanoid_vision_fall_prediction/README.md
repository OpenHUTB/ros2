# 人形机器人视觉跌倒预测与分级平衡控制

> 课程任务②：实现传感器感知及运动控制 —— 前视相机图像 → MLP 跌倒方向分类 → 踝/髋分级修正

## 功能概述

基于 MuJoCo 物理仿真（实测 4.0.0），在人形机器人头部挂载前视相机，采集随机外力扰动下的灰度图像，训练一个纯 NumPy 实现的 3 层 MLP（256→128→64→5），实时预测机器人的跌倒方向（正常/前倒/后倒/左倒/右倒），并输出相应的踝/髋关节修正目标角。

### 方法原理

```
前视相机 64×64 灰度图
      │
      ▼ 16×16 平均池化 → flatten (256 维)
┌─────────────────┐
│   3 层 MLP       │
│ 256→128→64→5    │
│  ReLU + Softmax  │
└────────┬────────┘
         │ 5 类概率
         ▼
  normal / forward / backward / left / right
         │
         ▼
  BalanceController：按类别叠加踝/髋修正力矩
```

- **感知**：前视相机（`front_camera`，fovy=60）渲染 64×64 灰度图
- **神经网络**：纯 NumPy 实现的 MLP（无 PyTorch 依赖），交叉熵损失 + SGD 动量
- **控制**：基于预测类别，叠加 `_CORRECTION` 关节角偏移到站姿 PD 目标

## 目录结构

```
humanoid_vision_fall_prediction/
├── main.py                              # standalone 入口（--collect/--train/--demo/--check）
├── main.sh                              # Linux 启动脚本
├── package.xml / setup.py / setup.cfg    # ROS2 ament_python 包配置
├── requirements.txt                     # Python 依赖
├── config/params.yaml                    # 仿真/站姿/采集/网络参数
├── models/humanoid_camera.xml           # MuJoCo MJCF 人形模型（含前视相机）
├── launch/main.launch.py                # ROS2 launch（同时启动仿真+推理节点）
├── humanoid_vision_fall_prediction/
│   ├── sim.py                           # MuJoCo 仿真封装（站姿 PD + 推力）
│   ├── dataset.py                       # 随机推力数据采集
│   ├── nn.py                            # 纯 NumPy MLP
│   ├── train.py                         # 训练脚本
│   ├── predictor.py                     # 推理封装
│   ├── controller.py                    # 分级平衡控制器
│   ├── vision_sim_node.py                # ROS2 仿真节点
│   └── fall_predictor_node.py           # ROS2 推理节点
├── data/fall_dataset.npz                # 数据集（1500 样本，每类 300）
├── models_trained/fall_predictor.npz     # 训练权重（保存验证集最优）
└── test/test_smoke.py                   # 冒烟测试
```

## 环境要求

- Python 3.8+（实测 3.12.11）
- MuJoCo ≥ 3.0（实测 4.0.0）
- NumPy ≥ 1.21（实测 1.26.4）
- PyYAML ≥ 5.4
- （ROS2 节点需要 rclpy，实测 Ubuntu 22.04 + ROS 2 Humble）

安装：
```bash
pip install -r requirements.txt
```

## 快速开始

### 1. 自检（验证模型/渲染/训练流程）
```bash
python3 main.py --check
```

### 2. 采集数据集
```bash
python3 main.py --collect --samples 1500
# 输出：data/fall_dataset.npz（1500 样本，5 类各 300）
```

### 3. 训练神经网络
```bash
python3 main.py --train
# 输出：models_trained/fall_predictor.npz（保存验证集最优权重）
```

### 4. 演示（推扰 + 视觉预测 + 平衡控制）
```bash
# 前向推扰 120 N
python3 main.py --demo --force 120 --direction 0

# 侧向推扰
python3 main.py --demo --force 120 --direction 90
```

### 5. ROS2 launch（Ubuntu 22.04 + Humble，实测）
```bash
# 编译
colcon build --packages-select humanoid_vision_fall_prediction
source install/setup.bash
# 启动
ros2 launch humanoid_vision_fall_prediction main.launch.py
```

## 性能评价（Ubuntu 22.04 + MuJoCo 4.0.0 实测）

| 指标 | 数值 | 复现方式 |
|---|---|---|
| 数据集规模 | 1500 样本（5 类各 300，完全均衡） | `--collect --samples 1500` |
| 采集耗时 | 约 75 s | — |
| 训练轮数 | 300 epochs（lr 0.005，每 50 轮 ×0.8） | `--train` |
| 训练耗时 | 约 1.2 s（纯 NumPy，CPU） | — |
| 验证准确率 | **1.000**（300 个验证样本全部正确） | `--train` 日志 |
| 单帧推理耗时 | < 1 ms（CPU） | — |
| 无扰动站立 3 s | 躯干倾角 < 5°，不跌倒 | `--check` |
| 60 N 前推存活步数 | 648 步（原站姿 161 步，约 4 倍） | 见「站姿标定」 |
| 预测类别 | normal / forward / backward / left / right | — |

验证准确率之所以能到 1.000，是因为采集帧取自“即将跌倒”的临界区（倾角 ≥ 7°），
图像本身已经清楚编码了倾斜方向：用最近类心分类器在同样数据上也是 1.000，
Fisher 判别比 9.16。也就是说感知不是瓶颈，瓶颈在控制环节（见下）。

## 站姿标定（为什么改默认站姿）

原 `DEFAULT_STAND_TARGET` 用 hip=0.08 / knee=-0.16 / ankle=-0.08，三组角度没有配平
（踝角应与髋角+膝角反号），实测开局躯干即前倾 **60.8°**，1 s 内根高从 1.088 m
沉到 0.944 m，任何推力都会立刻跌倒，且调 PD 增益（kp 60→800、kd 6→50）完全无效。

按“踝角 ≈ -(髋角 + 膝角)”做网格搜索（30 组）后仅两组能站住 3 s，取其中更稳的
hip=0.22 / knee=-0.05 / ankle=+0.14（3 s 倾角 1.49°）。同一 PD 下的抗推对比：

| 推力/方向 | 原站姿（跌倒步数） | 新站姿（跌倒步数） |
|---|---|---|
| 60 N / 前 | 161 | 648 |
| 100 N / 前 | 154 | 563 |
| 140 N / 前 | 147 | 503 |
| 60 N / 左 | 172 | 928 |
| 100 N / 左 | 172 | 898 |
| 60 N / 后 | 185 | 842 |

另需注意：`reset()` 不能把目标站姿直接写进 `qpos`（脚会离地，553 步即倒），
应先置“脚掌贴地”的初始姿态再让 PD 收敛，`sim.py` 的 `INIT_QPOS` 就是为此保留。

## 已知局限（实测记录）

1. **分级平衡修正不足以阻止跌倒。** 在 60–140 N、持续 0.08 s 的推力下，开/
   关视觉修正的跌倒率都是 100%，平均存活步数只差 1–3 步。量级估算：修正量
   |Δ| ≈ 0.10 rad × gain 0.6 × conf ≈ 0.05 rad，对应关节力矩约 3 N·m，而抵抗
   60 N 推力需要约 32 N·m（力臂约 0.35 m），差一个数量级。要真正抗扰，需要
   换成带根姿态反馈的 WBC/MPC，或把增益提高一个量级，属于后续工作。
2. **纯关节 PD 属于不稳定平衡。** 模型没有根姿态反馈，站姿只能“缓慢漂移”。
   3 s 内倾角 < 2°，但时间更长会逐渐前倾直到跌倒，所以所有指标都在有限时间窗
   内测量，不能声称“无限期站立”。
3. **图像区分的是“当前倾斜方向”。** 采集帧位于倾角 ≥ 7° 的临界区（距跌倒约
   0.5–2 s），此时“当前倒向”≈“即将倒向”；不是从完全直立的静态姿态预测未来。

## 与已有模块的区别

- 与 `humanoid_push_recovery`（规则式 ZMP/捕获点平衡）不同：本模块**从前视图像学习**跌倒方向，不依赖解析的 ZMP/质心计算。
- 与 PR2（端到端图像→关节力矩）形成对比：本模块是"感知分类 + 规则控制"的模块化方法。

## 参考文献

1. Macenski S, et al. *Robot Operating System 2: Design, architecture, and uses in the wild.* Science Robotics, 2022.
2. MuJoCo Documentation: https://mujoco.readthedocs.io/
