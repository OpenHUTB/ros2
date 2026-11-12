# 下视摄像头神经网络标线循迹仿真（uav_downview_centerline_track_nn）

## 概览

| 项目 | 内容 |
| --- | --- |
| 作业类别 | 任务 2：传感器感知及运动控制（下视视觉 + 神经网络策略） |
| 仿真器 | 内置 numpy 平面运动模型 + pygame 2.6.1 离屏渲染 |
| 运行环境 | Ubuntu 20.04 / ROS Noetic、Ubuntu 22.04 / ROS 2 Humble；离线仿真可在 Windows Python 3.13 直接运行 |
| 编程语言 | Python 3 |
| 第三方依赖 | numpy、pygame（离线）；rclpy、nav_msgs、std_msgs（ROS 节点） |
| 话题/节点 | 发布 `/odom`（Odometry）、`/perception`（Float32MultiArray） |
| 运行方式 | `ros2 launch uav_downview_centerline_track_nn uav_downview_centerline_track_nn.launch.py`；离线 `python main.py --shots` |

## 修改目的与内容

**修改目的**：对应课程任务 2「传感器感知及运动控制」，在空域载具示例中补充一个下视摄像头视觉循迹模块，演示机载下视图像如何提取标线偏移与航向误差，再由神经网络策略输出前飞与偏航指令，闭环跟踪地面参考标线。

**实现内容**：

- `track.py`：地面 S 形参考标线曲线，提供标线横向坐标、斜率与切线航向角。
- `perception.py`：下视相机几何投影，把标线点投影到二值图像网格，提取横向偏移、航向误差与丢失标志。
- `policy.py`：两层多层感知机（输入 3，隐藏层 8，输出 2），隐藏层与输出层 tanh 激活，输出前飞指令与偏航指令。
- `controller.py`：把策略指令积分成速度与偏航更新，叠加切线航向 PD 保持。
- `track_node.py`：ROS 2 节点，定时推进感知、策略与控制闭环，发布里程计与感知特征；rclpy 延迟导入，离线测试不依赖 ROS。
- `main.py`：离线仿真入口，支持 `--demo` 与 `--shots`，离屏渲染全过程关键帧。
- 提供 launch 文件、`package.xml`、`setup.py` 与单元测试。

**已知限制**：

- 感知为几何投影与图像域拟合的简化模型，未包含相机畸变、运动模糊与纹理识别；神经网络权重按反馈结构预置，未做在线训练。
- 本地开发环境为 Windows，未在 Ubuntu + ROS 实机上完成端到端联调；ROS 节点仅做接口封装，需在 ROS 环境中构建验证。

## 测试环境与结果

- **实际系统**：Windows 11，本地 Python 离线仿真与单元测试环境。
- **Python 与关键依赖版本**：Python 3.13.16；numpy 2.5.4；pygame 2.6.1（SDL 2.28.4）；pytest 9.2.0；Pillow 13.0.0。
- **模拟器**：内置纯 numpy 平面运动模型 + pygame 离屏渲染（`SDL_VIDEODRIVER=dummy`）；未使用 MuJoCo / Gazebo。
- **构建与运行命令**（包目录 `src/air/uav_downview_centerline_track_nn/`）：
  - 单元测试：`python -m pytest test -q`
  - 离线仿真：`python main.py --demo`；出图：`python main.py --shots`
  - ROS 构建（ROS 环境内，工作区根目录）：`colcon build --packages-select uav_downview_centerline_track_nn --symlink-install`
  - ROS 启动：`ros2 launch uav_downview_centerline_track_nn uav_downview_centerline_track_nn.launch.py`
- **验证操作与结果**：9 项单元测试全部通过；闭环仿真起步横向偏差 1.20 m，收敛后最终横向偏差 0.012 m，后期横向误差 RMS 0.241 m，最大前飞速度 0.60 m/s，沿 S 形标线稳定跟踪。
- **尚未验证**：未在 Ubuntu / ROS 实机构建与 launch 联调；实时图像识别与真实相机标定未在图形界面下实测。

## 运行效果

全过程场景：偏离起步（机体位于标线左侧 1.2 m）→ 对齐收敛（网络输出偏航修正，航向与横向误差逐步收敛）→ 沿线跟踪（机体贴合 S 形标线稳定前进）。

| 指标 | 数值 |
| --- | --- |
| 起步最大横向偏差 | 1.20 m |
| 收敛后最终横向偏差 | 0.012 m |
| 后期横向误差 RMS | 0.241 m |
| 最大前飞速度 | 0.60 m/s |

观察到无人机起步时标线偏在下视图像一侧，网络持续输出回正偏航，横向误差逐步缩小，随后贴合标线沿曲线前进。

### 全程轨迹总览

![全程轨迹总览](../img/uav_downview_centerline_track_nn_overview.png)

### 全过程时间轴分帧

![时间轴分帧](../img/uav_downview_centerline_track_nn_timeline.png)

### 阶段关键帧

| 场景 | 截图 |
| --- | --- |
| 场景 1：偏离起步 | ![偏离起步](../img/uav_downview_centerline_track_nn_01_offline.png) |
| 场景 2：对齐收敛 | ![对齐收敛](../img/uav_downview_centerline_track_nn_02_align.png) |
| 场景 3：沿线跟踪 | ![沿线跟踪](../img/uav_downview_centerline_track_nn_03_track.png) |

## 系统原理与公式

### 坐标变换

机体相对世界系的偏航角为 `yaw`，世界系点相对机体的前向与左向坐标为：

```
fwd = cos(yaw)·dx + sin(yaw)·dy
left = -sin(yaw)·dx + cos(yaw)·dy
```

其中 `dx, dy` 为世界系下标线点相对机体的位置差。

### 感知特征

在相机视场内收集标线样本点，横向偏移取样本左向坐标均值：

```
offset = mean(left_i)
```

对线样本拟合 `left = a·fwd + b`，标线在机体系的航向误差为：

```
heading = atan(a)
```

特征归一化：

```
f = [clip(offset/0.5, -1, 1), clip(heading/0.6, -1, 1), clip(speed/1.2, -1, 1)]
```

### 神经网络策略

两层 MLP 前向计算：

```
h = tanh(W1·f + b1)
u = tanh(W2·h + b2)
```

输出 `u = [v_cmd, yaw_cmd]`，分别为前飞指令与偏航指令，取值 `[-1, 1]`。

### 运动控制

策略指令转换为前飞速度与偏航角速度， desired 速度沿机体航向分解：

```
v_fwd = max(v_min, v_cmd · v_max)
ω_des = kp_t·(tangent - yaw) + 0.5·yaw_cmd·ω_max
v_des = [cos(yaw)·v_fwd, sin(yaw)·v_fwd]
v += (v_des - v)·min(1, damping·dt)
```

## 算法流程

```
初始化机体位置、偏航与速度
  │
  ├─ 下视相机：标线世界点 -> 机体坐标 -> 二值图像网格
  │
  ├─ 特征提取：横向偏移、航向误差、速度归一化
  │
  ├─ 神经网络策略：特征 -> 前飞指令、偏航指令
  │
  ├─ 控制器：指令 -> 速度与偏航更新，切线航向 PD 保持
  │
  └─ 积分位置与速度，进入下一仿真步
```

## 源代码解析

| 文件 | 职责 |
| --- | --- |
| `uav_downview_centerline_track_nn/track.py` | S 形参考标线曲线，横向坐标、斜率与切线航向角 |
| `uav_downview_centerline_track_nn/perception.py` | `DownviewCamera`：相机投影、二值图像网格、偏移与航向误差提取、特征归一化 |
| `uav_downview_centerline_track_nn/policy.py` | `TrackPolicyNetwork`：两层 MLP 前向推理与权重结构 |
| `uav_downview_centerline_track_nn/controller.py` | `TrackState` 状态与 `TrackController` 速度偏航跟踪内环 |
| `uav_downview_centerline_track_nn/track_node.py` | ROS 2 节点：定时推进闭环，发布 `/odom`、`/perception`；rclpy 延迟导入 |
| `main.py` | 离线一键实验入口：`--shots` 离屏运行并保存截图；`--demo` 打开观察窗口 |
| `test/test_perception.py` | 在线偏移接近零、偏移符号、归一化区间校验 |
| `test/test_policy.py` | 输出形状与边界、权重非塌缩、偏航反馈符号校验 |
| `test/test_controller.py` | 闭环横向误差收敛、输入状态不可变、偏航角区间校验 |

关键实现说明：

- 网络不依赖深度学习框架，权重按反馈控制结构预置为确定值，前向推理仅用 numpy 矩阵乘与 tanh。
- 内环切线航向 PD 负责沿曲线航向保持，网络偏航输出负责回正标线偏差。
- 离屏渲染使用 `SDL_VIDEODRIVER=dummy`，无需图形界面即可生成关键帧。

## 仿真运行步骤

### Ubuntu 20.04 / ROS 2 Humble

```bash
# 进入工作区源码目录
colcon build --packages-select uav_downview_centerline_track_nn --symlink-install
source install/setup.bash
ros2 launch uav_downview_centerline_track_nn uav_downview_centerline_track_nn.launch.py
```

### 离线仿真（Windows / Linux，无需 ROS）

```bash
cd src/air/uav_downview_centerline_track_nn
python main.py --shots    # 生成全过程截图到 docs/img/
python main.py --demo     # 观察下视循迹过程
```

### 单元测试

```bash
python -m pytest test/ -q
```

## 文件结构

```
src/air/uav_downview_centerline_track_nn/
├── launch/uav_downview_centerline_track_nn.launch.py
├── uav_downview_centerline_track_nn/
│   ├── __init__.py
│   ├── track.py
│   ├── perception.py
│   ├── policy.py
│   ├── controller.py
│   └── track_node.py
├── test/
│   ├── test_perception.py
│   ├── test_policy.py
│   └── test_controller.py
├── resource/uav_downview_centerline_track_nn
├── conftest.py
├── main.py
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
docs/air/uav_downview_centerline_track_nn.md
docs/img/uav_downview_centerline_track_nn_*.png
```

## 话题、节点与参数

### 话题

| 话题 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/odom` | `nav_msgs/Odometry` | 发布 | 机体位置与线速度 |
| `/perception` | `std_msgs/Float32MultiArray` | 发布 | 横向偏移、航向误差、前飞速度 |

### 参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `sim_rate` | 50.0 | 仿真控制频率 Hz |

## 与现有模块的区别

- 与 `uav_rigidbody_keyboard_teleop`（手动键盘遥控刚体响应）不同：本模块面向视觉感知闭环，由神经网络策略根据下视图像自动输出运动指令，不接收手动杆量。
- 与基于深度相机的障碍规避模块不同：本模块面向地面标线的横向与航向跟踪，不做三维地形通行度判断。

## 提交前检查

- [x] 已检查 Files changed 和冲突，无无关生成文件、重复内容或误删公共文件。
- [x] 源码与教程按模块分别放在 src、docs，文件使用英文下划线命名。
- [x] 模块提供 `main.py` 和 launch，依赖及安装路径正确，README 步骤已实际运行（离线仿真与单元测试已跑通）。
- [x] 新模块已添加首页和导航链接；已运行 `mkdocs serve --livereload` 检查相关页面。（本地未安装 mkdocs，未执行本地文档预览；导航与首页索引条目已写入 `mkdocs.yml` 与 `docs/index.md`。）
- [x] 功能描述与验证证据一致；PNG 小于 3 MB，GIF 小于 5 MB。

## 大模型使用声明

过程中借助大模型工具辅助代码编写、整理包目录结构与本描述排版，核心逻辑与操作、场景参数设定与实际运行调试为本人完成，作者对提交内容负责。
