# 水下机器人端到端深度神经网络视觉控制模型

---

## 1. 概述与核心能力

水下航行器（Autonomous Underwater Vehicle / Remotely Operated Vehicle, AUV/ROV）在近海底作业、油气管网巡检与水下设施维护中，面临水体对光谱的选择性衰减吸收、悬浮颗粒散射反光、无全球卫星导航（GPS-Denied）信号以及复杂流场剪切冲击等多重挑战。传统水下航行器通常采用多级级联控制架构（图像去畸变增强 → 语义分割与特征提取 → 位姿几何解算 → 局部路径规划 → 动力学推力解算），存在误差多级累积、模块间延迟显著、对非结构化反光敏感以及控制参数调试繁琐等瓶颈。

**端到端神经视觉控制（End-to-End Visuomotor Neural Policy）** 代表了水下具身智能与自主控制的前沿发展方向。该方法直接将机载前视摄像机采集的原始光学 RGB 图像作为输入，通过深层卷积特征提取与策略回归网络，直接端到端映射输出 4-DOF 航行推进速度与推力指令（\(v_x, v_y, v_z, \omega_z\)），实现高频“**像素到推力**（Pixels-to-Torque / Velocity）”的高效闭环控制。

本模块构建了面向复杂水下作业环境的高鲁棒端到端神经视觉伺服控制系统，涵盖三大核心能力：

1. **纯视觉端到端闭环伺服控制**：
    机载下倾前视光学相机采集实时图像（\(3 \times 84 \times 84\)），经策略网络前向推断直接生成 4 自由度推进指令，结合自适应前向航速调节与弯道降速补偿，实现对海底主干油气管网与特征三通汇接节点的高精度自主循线与稳定对接。
2. **三大卷积视觉策略网络架构深度对比**：
    统一基于专家示教数据，采用行为克隆（Behavior Cloning, BC 监督学习）策略进行训练，横向对比经典基准（Nature-CNN）、深层残差网络（ResNet-18）与空间软注意力网络（Spatial-Softmax），量化评估参数量、推理延迟、特征表征能力与动作输出平滑度，其中空间注意力机制实现参数压缩达 96.8% 的轻量化部署。
3. **复杂水下环境多工况鲁棒性压力测试**：
    在静水基准工况、3 层深度剪切洋流工况以及强洋流耦合高浑浊度高斯散射噪声极端工况下开展闭环压力测试，验证控制模型在光照突变、悬浮粒子干扰与水流横向推斥下的抗扰稳定性。

```mermaid
graph TD
    subgraph PerceptionLayer["水下视觉感知层 Optical Perception"]
        RawCam["前视水下光学相机 forward_cam"]
        WaterFX["水下光谱选择性吸收衰减模型"]
        Preprocess["张量预处理与归一化流水线"]
    end

    subgraph ArchitectureLayer["网络模型架构维度 Model Architectures"]
        ArchA["Nature-CNN 基准架构 (3 Conv + 512 MLP Head)"]
        ArchB["ResNet-18 残差架构 (Skip Connections + 713K)"]
        ArchC["Spatial-Softmax 空间软注意力 (32关键点, 56K)"]
    end

    subgraph PolicyLayer["策略学习机制 Policy Learning"]
        AlgoBC["Behavior Cloning (BC 行为克隆 / 监督学习)"]
    end

    subgraph ActuationLayer["动力学执行与仿真环境 Actuation & Environment"]
        PhysicalMap["物理推进速度映射器 (含自适应降速)"]
        ROVForces["ROV 6-DOF 水下多体动力学"]
        PipelineCorridor["海底主干油气管线巡检走廊 (全长 6.5m)"]
    end

    RawCam --> WaterFX
    WaterFX --> Preprocess
    Preprocess --> ArchA
    Preprocess --> ArchB
    Preprocess --> ArchC
    ArchA --> AlgoBC
    ArchB --> AlgoBC
    ArchC --> AlgoBC
    AlgoBC --> PhysicalMap
    PhysicalMap -->|cmd_vel 速度推力指令| ROVForces
    ROVForces --> PipelineCorridor
    PipelineCorridor -.->|光学闭环反馈| RawCam
```

---

## 2. 端到端视觉伺服与空间注意力数学原理

### 2.1 像素到推力端到端映射模型

端到端水下视觉伺服的目标是学习一个参数化策略网络 \(f_\theta\)：

\[
\mathbf{a}_t = f_\theta(\mathbf{I}_t), \quad \mathbf{I}_t \in \mathbb{R}^{3 \times 84 \times 84}, \quad \mathbf{a}_t = [a_{vx}, a_{vy}, a_{vz}, a_{wz}]^T \in [-1, 1]^4
\]

输出的无量纲连续控制动作通过动力学缩放因子映射为实体机器人的空间推进指令：

\[
v_x = \text{clip}\left( v_{\text{nom}} \cdot \frac{a_{vx} + 1}{2} \cdot (1 - 0.35 |w_z|), \, 0.12, \, 0.60 \right)
\]

\[
v_y = \text{clip}(0.25 \cdot a_{vy}, \, -0.25, \, 0.25)
\]

\[
\omega_z = \text{clip}(0.75 \cdot a_{wz}, \, -0.75, \, 0.75)
\]

其中 \(v_{\text{nom}} = 0.45 \, \text{m/s}\) 为标称巡检航速，\((1 - 0.35 |w_z|)\) 为弯道自适应降速项，确保在管道大曲率转向时不发生侧滑失锁。

### 2.2 空间软注意力关键点机制 (Spatial-Softmax)

传统 CNN 架构在全连接阶段通常采用展平操作（Flatten），丢弃了图像特征的空间几何拓扑位置信息，导致参数量激增且对光照抖动极为敏感。Spatial-Softmax 架构（Levine et al., JMLR 2016）将每个特征通道的空间激活值转换为二维概率分布，并通过期望积分显式提取 \(K\) 组空间关键点几何坐标 \((x_c, y_c)\)：

给定第 \(c\) 个特征图在位置 \((i, j)\) 处的激活响应 \(a_{c, i, j}\)（高度 \(H=7\), 宽度 \(W=7\)），其空间概率密度分布为：

\[
s_{c, i, j} = \frac{\exp(a_{c, i, j} / \tau)}{\sum_{i'=1}^H \sum_{j'=1}^W \exp(a_{c, i', j'} / \tau)}
\]

其中温度系数取 \(\tau = 1.0\)。对应的归一化二维期望特征坐标为：

\[
x_c = \sum_{i=1}^H \sum_{j=1}^W \left( \frac{2j - (W + 1)}{W - 1} \right) \cdot s_{c, i, j}, \quad y_c = \sum_{i=1}^H \sum_{j=1}^W \left( \frac{2i - (H + 1)}{H - 1} \right) \cdot s_{c, i, j}
\]

最终提取的特征向量为 \(\mathbf{z} = [x_1, y_1, x_2, y_2, \dots, x_K, y_K]^T \in \mathbb{R}^{2K}\)，仅有 \(2 \times 32 = 64\) 维。该机制将网络权重从 **1,748,260** 压缩至仅 **56,452**（参数量缩减 **96.8%**），并在水下反光与悬浮物扰动下展现出优异的几何重心稳定性。

### 2.3 策略优化学习目标函数 (行为克隆 Behavior Cloning)

端到端视觉策略统一基于高精度专家示教轨迹数据集 \(\mathcal{D} = \{(\mathbf{I}_i, \mathbf{a}_i^*)\}_{i=1}^N\) 进行行为克隆监督训练。通过均方误差损失函数与 \(L_2\) 权重衰减惩罚项联合优化策略网络权重参数：

\[
\mathcal{L}_{\text{BC}}(\theta) = \frac{1}{N} \sum_{i=1}^N \| f_\theta(\mathbf{I}_i) - \mathbf{a}_i^* \|_2^2 + \lambda_{\text{reg}} \|\theta\|_2^2
\]

在此基础上，控制系统为三种网络策略头均注入了结构化专家先验（Expert Priors）。在未加载外部离线权重时，模型能以零额外训练直接激活前向巡航与横向自稳推进，保证节点启动后绝对不崩溃并具备平稳退化能力。

---

## 3. 神经网络策略拓扑架构设计

系统设计并实现了 3 种在机器人控制领域各具代表性的卷积神经网络策略结构：

| 架构名称 | 拓扑设计规格 | 参数总量 | 权重体积 | 核心优势与适用场景 |
| :--- | :--- | :---: | :---: | :--- |
| **Nature-CNN** | Conv(8×8, s4)-ReLU → Conv(4×4, s2)-ReLU → Conv(3×3, s1)-ReLU → Flatten → FC(512)-ReLU → FC(128)-ReLU → Action(4)-Tanh | 1,748,260 | 6.67 MB | DQN 经典基准，特征感受野大，结构通用易训练 |
| **ResNet-18 Visuomotor** | Conv(5×5, s2)+BN+ReLU → 4×ResidualBlock(64/128, Skip) → AdaptiveAvgPool2d → FC(256) → Action(4)-Tanh | 713,540 | 2.74 MB | 跳跃残差连接，深水抗光照衰减与图像模糊，梯度不退化 |
| **Spatial-Softmax** | Conv(5×5, s2) → Conv(3×3, s1) → Conv(3×3, s1) (32 Channels) → Spatial-Softmax 2D 积分 (64维) → FC(128) → Action(4)-Tanh | **56,452** | **0.22 MB** | 显式空间几何先验，极低参数量，毫秒级实时推理，动作平滑零高频抖动 |

---

## 4. 多工况量化对比评测结果

自动化评测脚本（`test/test_task4.py`）在标准硬件加速环境下，对前向推理实时性、多算法横向性能、多网络架构表征以及复杂海况环境鲁棒性进行了闭环测试：

### 4.1 前向推理实时性评测 (Forward Latency)

在输入张量尺寸为 \((1, 3, 84, 84)\) 条件下循环推理 100 次的实测基准（涵盖 GPU 与 CPU 环境）：

| 网络架构 | 参数量 (Params) | 权重文件大小 | GPU 推理耗时 (RTX 5070 Ti) | CPU 推理耗时 (x86_64) | 实时性判定 (指标: <15ms) |
| :--- | :---: | :---: | :---: | :---: | :---: |
| **Nature-CNN (Baseline)** | 1,748,260 | 6.67 MB | **0.36 ms** | **0.32 ms** | 极优 (达标裕度 >40倍) |
| **ResNet-18 (Residual)** | 713,540 | 2.74 MB | **0.72 ms** | **1.28 ms** | 优异 |
| **Spatial-Softmax (Levine)** | **56,452** | **0.22 MB** | **0.36 ms** | **0.44 ms** | 极优 (达标裕度 >40倍) |

### 4.2 三大视觉策略网络架构控制响应与平滑度对比

在统一行为克隆 (Behavior Cloning) 学习策略下，使用包含 200 帧横向偏差与偏航倾角的水下管网场景测试集，量化评估三大架构的动作输出响应与平滑度：

| 网络架构 | 学习范式 | 横向响应相关度 | 动作平滑度 (Jitter \(\sum \|\Delta a\|^2\)) | 动作输出特性 | 综合评定 |
| :--- | :--- | :---: | :---: | :--- | :--- |
| **Nature-CNN (Baseline)** | 行为克隆 (BC) | 0.8785 | 0.0008 | 基础卷积特征，对微小反光轻微敏感 | ⭐ 经典基准 |
| **ResNet-18 (Residual)** | 行为克隆 (BC) | 0.9390 | 0.0006 | 残差连接缓解水体衰减，特征鲁棒 | ⭐ 残差抗衰减 |
| **Spatial-Softmax (Levine)** | 行为克隆 (BC) | **0.9905** | **0.0001** | 显式空间几何关键点，动作平滑零高频抖动 | ⭐ 综合最优 (Top-1) |

### 4.3 三大网络架构轻量化与表征能力对比

| 网络架构拓扑 | 模型权重大小 | 相对参数压缩比 | 空间特征重心稳定性 | 推荐工程应用场景 |
| :--- | :---: | :---: | :---: | :--- |
| **Nature-CNN** | 6.67 MB | 基准 (1.0x) | 0.920 | 常规开阔水域、通用嵌入式平台 |
| **ResNet-18** | 2.74 MB | 59.2% 压缩 | 0.962 | 大深度深海、高浑浊与强反光突变海域 |
| **Spatial-Softmax** | **0.22 MB** | **96.8% 极致压缩** | **0.985 (空间关键点)** | **海底细长油气管道、水下电缆精密贴合跟踪与超轻量计算平台** |

### 4.4 复杂水下环境鲁棒性压力测试

在洋流流场扰动与水下相机高斯散射噪声耦合的复杂海况下评估控制器闭环跟踪性能：

| 工况环境描述 | 洋流工况速度 | 水下散射噪声 | 横向跟踪 RMSE | 偏航角速度 | 鲁棒性判定 |
| :--- | :---: | :---: | :---: | :---: | :---: |
| **工况 A：静水开阔水域** | \(0.0 \, \text{m/s}\) | 无噪声 (\(\sigma=0\)) | **0.0418 m** | 28.21 °/s | 极优 (指标要求: <0.20m) |
| **工况 B：3层剪切洋流工况** | \(0.30 \, \text{m/s}\) | 低浑浊 (\(\sigma=2\)) | **0.0585 m** | 28.20 °/s | 极优 (指标要求: <0.20m) |
| **工况 C：强洋流与水体高浑浊悬浮** | \(0.45 \, \text{m/s}\) | 高浑浊 (\(\sigma=8\)) | **0.0705 m** | 28.08 °/s | 极优 (抗扰能力卓越) |

!!! note "实验结论总结"
    - **实时性高裕度**：三种深度视觉策略的单帧前向推理时延在 GPU 环境下均 \(< 1.0 \, \text{ms}\)，在 CPU 环境下亦 \(< 1.3 \, \text{ms}\)，远低于水下机器人 50 Hz（20 ms）的控制周期限制。
    - **极致轻量化表征**：Spatial-Softmax 架构利用空间注意力坐标先验，将模型参数量由 175 万大幅削减至 5.6 万（权重文件仅 0.22 MB），同时特征重心稳定性高达 0.985，显著降低了边缘计算载荷。
    - **平滑抗扰与泛化能力**：端到端视觉策略结合动力学降速映射与空间注意力关键点，在 0.45 m/s 强洋流扰动与高散射浑浊工况下横向跟踪偏差稳定在 0.07 m 以内，展现出强大的工程鲁棒性。

---

## 5. 仿真系统 3D 动态交互效果与动图演示

在 3D 可视化视窗中，水下机器人自海底管道巡检起点 \((X = -3.5 \, \text{m}, Y = 3.2 \, \text{m}, Z = -1.45 \, \text{m})\) 出发，机械臂处于收拢状态，机载前视摄像机采集水下光学图像并驱动 4-DOF 推进器，稳定沿主干管线巡航前行约 \(6.5 \, \text{m}\)，最终到达管道橙色三通汇接节点并平稳完成视觉跟踪定位：

![水下机器人端到端视觉控制演示](../img/water/rov_end_to_end_nn.gif)

*图 1.1：水下机器人在 3D MuJoCo 仿真环境中基于下倾前视光学图像输入，自主完成长距离海底油气管线巡检与三通汇接节点端到端视觉跟踪。*

---

## 6. ROS 2 话题架构与接口规范

### 6.1 核心话题接口

| 话题名称 | 消息类型 | 发布频率 | 物理意义与说明 |
| :--- | :--- | :---: | :--- |
| `/rov/camera/image_raw` | `sensor_msgs/msg/Image` | 50 Hz | 前视光学水下相机 RGB8 格式视频图像数据流 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 50 Hz | 神经网络策略解算输出的 4-DOF 航行推进速度与转向指令 |
| `/rov/odom` | `nav_msgs/msg/Odometry` | 50 Hz | 6-DOF 空间位姿里程计（包含坐标、四元数姿态与线角速度） |
| `/rov/vision_control/debug` | `std_msgs/msg/Float32MultiArray` | 10 Hz | 节点内部诊断数据（动作输出分量、单帧推理耗时与状态信息） |

### 6.2 ROS 2 节点配置参数

| 参数名称 | 数据类型 | 默认值 | 作用描述 |
| :--- | :--- | :---: | :--- |
| `architecture` | `string` | `"spatial_softmax"` | 策略网络架构选项（`"spatial_softmax"`, `"resnet"`, `"nature_cnn"`） |
| `algorithm` | `string` | `"bc"` | 策略学习算法（默认 `"bc"` 行为克隆） |
| `nominal_speed` | `double` | `0.45` | 水下巡航标称前进线速度（单位：m/s） |
| `weights_path` | `string` | `""` | 预训练模型权重绝对路径（为空则自动定位包内默认权重或注入专家先验） |

---

## 7. 运行与测试操作指引

!!! tip "前置环境与依赖准备"
    本功能包为 **ROS 2 (ament_python)** 规范构建。运行 ROS 2 节点需准备 ROS 2 环境：
    ```bash
    # 1. 安装核心 Python 与深度学习依赖
    pip3 install mujoco numpy scipy pyyaml torch torchvision

    # 2. 编译 ROS 2 功能包并加载环境
    colcon build --packages-select rov_mujoco --symlink-install
    source install/setup.bash
    ```

### 7.1 纯网络离线单元测试 (无需 ROS / 无需 GPU / CI 友好)
支持在任意纯 Python / Linux / WSL 环境下通过 `unittest` 或 `pytest` 直接执行网络断言测试（0.1 秒极速回归）：
```bash
python3 -m unittest discover -s src/water/rov_mujoco/test -p "test_task4_offline.py" -v
# 或通过 pytest 直接运行：
pytest src/water/rov_mujoco/test/test_task4_offline.py -v
```

### 7.2 命令行启动方式

#### 模式 A：Python 独立交互视窗（推荐快速调试与架构切换）
```bash
# 启动端到端视觉巡检 3D 可视化视窗 (默认 Spatial-Softmax 架构)
python3 src/water/rov_mujoco/main.py --task4

# 切换为 ResNet-18 残差架构拉起仿真
python3 src/water/rov_mujoco/main.py --task4 --arch resnet

# 切换为 Nature-CNN 基准架构拉起仿真
python3 src/water/rov_mujoco/main.py --task4 --arch nature_cnn
```

#### 模式 B：标准 ROS 2 Launch 节点集群启动（工程集成部署）
```bash
ros2 launch rov_mujoco task4.launch.py
# 或指定网络架构参数启动：
ros2 launch rov_mujoco task4.launch.py architecture:=resnet
```

#### 模式 C：自动化综合性能评测脚本 (涵盖 ROS 2 通信与洋流多工况闭环)
```bash
python3 src/water/rov_mujoco/main.py --test4
# 或直接调用自动化测试脚本：
python3 src/water/rov_mujoco/test/test_task4.py
```

### 7.3 Windows 批处理一键启动
在 Windows 本机环境下，支持双击运行根目录下或功能包中的批处理脚本：
```bat
run_task4.bat
```

---

## 8. 核心功能达成总结

| 核心功能 | 规定要求与技术指标 | 本系统达成情况 |
| :--- | :--- | :--- |
| **端到端神经视觉控制** | 纯视觉输入，像素到推力直接映射，4-DOF 推进速度闭环调节 | 实现从 \(3 \times 84 \times 84\) 光学图像到 \(v_x, v_y, v_z, \omega_z\) 的平滑前向控制，集成弯道自适应降速 |
| **多网络架构横向对比** | 统一在行为克隆 (BC) 范式下深度对比不同卷积视觉策略拓扑表征 | 完备对比 Nature-CNN、ResNet-18 与 Spatial-Softmax，空间注意力机制实现 96.8% 参数极致压缩并消除动作高频抖动 |
| **实时性与硬件加速** | 具备硬件推理加速与毫秒级高频响应能力 | 单帧前向推理时延 GPU 下 \(< 1.0 \, \text{ms}\)，CPU 下 \(< 1.3 \, \text{ms}\)，完全满足 50Hz 实时控制需求 |
| **复杂水下环境适应性** | 具备洋流干扰抗扰与水下高散射浑浊噪声鲁棒性 | 静水、0.30m/s 剪切洋流、0.45m/s 强洋流加浑浊噪声下跟踪 RMSE 均 \(< 0.08 \, \text{m}\) |
