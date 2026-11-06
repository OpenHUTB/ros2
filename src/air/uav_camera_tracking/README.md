# uav_camera_tracking

无人机机载相机**视觉目标跟踪**：用合成图像训练一个**神经网络感知器**（图像 → 目标像素热图），
再分别比较三条控制链路的闭环性能。全部在自带的轻量仿真中完成（纯 NumPy，无需 AirSim/Gazebo），
并提供 ROS 2 节点与 launch 启动。

三条被比较的链路：

| 链路 | 感知 | 控制 | 说明 |
|---|---|---|---|
| `expert` | 几何投影真值 | IBVS 比例+积分伺服 | 传统方法基线，同时是示教数据来源 |
| `two_stage` | **神经网络**（热图检测头） | **神经网络**（MLP 策略） | 感知与控制解耦，行为克隆 + DAgger |
| `e2e` | — | **神经网络**（图像直出指令） | 端到端策略，作为负结果如实报告 |

对应作业要求：**选择的感知/控制/端到端算法均为神经网络**、**多场景测试并含性能评价**、
**所有程序支持 launch 启动**。

![跟踪动图](../../../docs/img/air/uav_camera_tracking/tracking.gif)

## 目录结构

```
uav_camera_tracking/
├── uav_camera_tracking/
│   ├── params.py          全部仿真与算法参数（文档引用的数值都在这里）
│   ├── camera.py          针孔相机：投影、光线投射渲染、真值标注
│   ├── quadrotor.py       四旋翼简化动力学（速度环 + 一阶滞后 + 阻尼 + 阵风）
│   ├── scenes.py          4 个测试场景：静止/圆周/立柱遮挡/阵风
│   ├── nn.py              NumPy 手写 MLP（He 初始化 + Adam + 掩码回归）
│   ├── detector.py        感知网络：16x16 热图 + 可见性头
│   ├── controllers.py     三种控制器 + 视觉伺服特征定义
│   ├── sim.py             闭环推演、示教采集、DAgger、指标汇总
│   ├── plotting.py        训练曲线、场景对比图、三面板跟踪动图
│   ├── main.py            一键训练 + 评估 + 出图（离线入口）
│   ├── sim_node.py        ROS 2：仿真节点（出图/动力学/真值评价）
│   ├── detector_node.py   ROS 2：感知节点
│   ├── controller_node.py ROS 2：控制节点
│   └── model_io.py        权重目录定位（源码运行 / install 后都能找到）
├── launch/main.launch.py  一键启动三节点闭环
├── config/params.yaml     节点默认参数
├── models/                预训练权重（setup.py 安装到 share）
└── results/               CSV、曲线图、跟踪动图
```

## 快速开始

### 方式 A：离线一键复现（不需要 ROS）

```bash
cd ~/ros2/src/air/uav_camera_tracking
python3 -m pip install numpy matplotlib
python3 -m uav_camera_tracking.main          # 约 6 分钟（CPU）
python3 -m uav_camera_tracking.main --quick  # 约 1 分钟，仅验证流程
```

输出：`models/{detector,control,e2e}.npz`、`results/compare_summary.csv`，以及
`results/` 下的 `training_curves.png`、`compare.png`、`tracking.gif`、`occlusion.gif`。
其中只有 `compare_summary.csv` 与 `models/*.npz` 入库；图片是会随训练变化的生成物，
仓库里引用的是 `docs/img/air/uav_camera_tracking/` 下那一份（同一次运行拷贝过去的），
避免同一张图存两份。

### 方式 B：ROS 2 launch 启动闭环

```bash
cd ~/ros2
colcon build --packages-select uav_camera_tracking
source install/setup.bash

ros2 launch uav_camera_tracking main.launch.py                        # 圆周目标 + 两阶段
ros2 launch uav_camera_tracking main.launch.py scene:=occlusion       # 换场景
ros2 launch uav_camera_tracking main.launch.py mode:=expert scene:=static
ros2 launch uav_camera_tracking main.launch.py csv:=/tmp/uav_metrics.csv
```

`mode` 取 `expert` / `two_stage` / `e2e`，`scene` 取 `static` / `moving` / `occlusion` / `gust`。
仿真节点跑满时长后把本回合的可见占比、误差 RMS、平均速度与判定追加写入 `csv` 指定的文件。

## 话题接口

| 话题 | 类型 | 方向 | 内容 |
|---|---|---|---|
| `/uav/image` | `sensor_msgs/Image` | sim → detector, controller | 机载相机 RGB 图（160×120，`rgb8`） |
| `/uav/odom` | `nav_msgs/Odometry` | sim → controller | 位姿与速度（控制器做速度记忆用） |
| `/uav/detection` | `geometry_msgs/PointStamped` | detector → controller | `x=u`、`y=v`、`z=可见概率`（像素） |
| `/uav/cmd_vel` | `geometry_msgs/Twist` | controller → sim | `linear.x/y` 机体系前向/右向，`linear.z` 世界系垂直，`angular.z` 偏航角速度 |
| `/uav/target` | `geometry_msgs/PointStamped` | sim → 评价 | 目标真值像素（策略**不**读取，仅用于打分） |
| `/uav/error` | `geometry_msgs/PointStamped` | sim → 评价 | 归一化像素误差 `(ex, ey, 可见标志)` |

## 方法要点

### 1) 感知：空间热图而不是坐标回归

网络输入是图像的颜色对立通道（R−G、R−B、灰度）均值池化到 16×16 的 768 维向量，
主干为单隐层 MLP(768→96)，输出 16×16+1 维：前 256 维是目标位置热图，最后 1 维是可见概率。

监督用中心位于真值格子的二维高斯软标签做交叉熵（σ = 0.8 格），推理时对 softmax 概率做
**软质心**得到亚像素坐标。相比直接回归 `(u, v)`，定位误差从 21 px 降到 3 px 量级
——网络只需回答"哪个格子更像目标"，不必学习位置的查表映射，样本效率高得多。

### 2) 控制：图像纵坐标就是距离

对地面目标，图像纵坐标 `ey` 唯一决定距离（越远越靠上），因此距离环不必再回归视在尺寸：

```
ex = (u - (W-1)/2) / (W/2)          # 横向归一化误差
ey = (v - (H-1)/2) / (H/2)          # 纵向归一化误差（距离的单调函数）
vx = clip(KP_EY * (-ey) + KI_EY * ie, ±V_MAX)     # 前飞速度
vy = clip(KP_LAT * ex, ±V_MAX)                    # 横向纠偏
```

偏航由**机头指向律**负责（目标在画面右侧就右转，使光轴对准目标），三套控制器共用该律，
否则横向环与距离环会互相拉扯，对比也失去公平。目标丢失时先记忆保持 2 s（被遮挡的目标常从
原方位重现），仍未捕获再 1.2 rad/s 扫描。

控制网络的输入是 7 维伺服特征（`ex, ey, v_prev, 高度误差, 可见标志, 误差积分`），
输出归一化速度指令，用几何专家飞出的示教数据做行为克隆，再用 **DAgger** 让学习器自己飞、
专家对"它真正访问到的状态"重新标注。行为克隆只在专家轨迹附近有效，闭环一旦偏离就会外推发散
（实测未做 DAgger 时四场景全部发散，误差 0.68）。

### 3) 端到端：一个诚实的负结果

24×24 图像特征直接映射到速度指令，训练样本与两阶段相同。结果见下表：四个场景均未通过，
误差在 0.37–0.85 之间且遮挡/阵风场景大量丢失目标。原因可控地归为两点：

1. 端到端网络必须同时学"目标在哪"和"该怎么飞"，而示教数据只有约 1 千帧，
   样本量远不足以支撑两个耦合子任务；
2. 输出端没有显式的几何先验（如"纵坐标决定距离"），网络把无关的背景纹理也拟合进去，
   在没见过的位姿上直接外推到饱和输出。

这说明**解耦的感知 + 控制**在低数据量下更可靠，也是本模块保留两阶段为主方案的理由。

## 结果

下面是 4 个场景 × 3 个控制器、每格 3 个随机种子的闭环统计（成功率要求可见占比 ≥ 0.85
且可见时误差 RMS ≤ 0.18；误差为归一化像素误差，0.18 约等于 14 px）。

| 场景 | 控制器 | 成功率 | 误差 RMS | 丢失占比 | 平均速度 (m/s) |
|---|---|---|---|---|---|
| 静止目标 | 几何视觉伺服(专家) | 1.00 | 0.0781 | 0.0000 | 0.087 |
| 静止目标 | 两阶段(感知网+控制网) | 1.00 | 0.0848 | 0.0000 | 0.154 |
| 静止目标 | 端到端(图像->指令) | 0.00 | 0.3618 | 0.0917 | 2.885 |
| 圆周匀速目标 | 几何视觉伺服(专家) | 1.00 | 0.1152 | 0.0000 | 0.650 |
| 圆周匀速目标 | 两阶段(感知网+控制网) | 1.00 | 0.1355 | 0.0000 | 0.637 |
| 圆周匀速目标 | 端到端(图像->指令) | 0.00 | 0.5879 | 0.0000 | 4.046 |
| 立柱遮挡 | 几何视觉伺服(专家) | 1.00 | 0.1338 | 0.1200 | 0.284 |
| 立柱遮挡 | 两阶段(感知网+控制网) | 1.00 | 0.1587 | 0.1200 | 0.283 |
| 立柱遮挡 | 端到端(图像->指令) | 0.00 | 0.6799 | 0.8356 | 4.719 |
| 阵风扰动 | 几何视觉伺服(专家) | 1.00 | 0.1555 | 0.0000 | 0.488 |
| 阵风扰动 | 两阶段(感知网+控制网) | 1.00 | 0.1616 | 0.0000 | 0.488 |
| 阵风扰动 | 端到端(图像->指令) | 0.00 | 0.5270 | 0.1720 | 3.844 |

![场景对比](../../../docs/img/air/uav_camera_tracking/compare.png)

![训练曲线](../../../docs/img/air/uav_camera_tracking/training_curves.png)

立柱遮挡场景的重捕获过程（目标被三根立柱遮挡三次，每次约 0.6 s）：

![遮挡重捕获](../../../docs/img/air/uav_camera_tracking/occlusion.gif)

### ROS 2 侧实测（Ubuntu 22.04 + Humble，GitHub Actions）

同一随机种子、三个节点经 `ros2 launch` 闭环跑出的结果，由 `ros:humble` 容器产出
（`.github/workflows/ros2_uav_camera_tracking.yml`）：

| 场景 / 控制器 | 可见占比 | 误差 RMS | 平均速度 (m/s) | 判定 |
|---|---|---|---|---|
| `static` / expert | 1.0000 | 0.0835 | 0.252 | 通过 |
| `moving` / two_stage | 1.0000 | 0.1343 | 0.674 | 通过 |
| `occlusion` / two_stage | 0.8800 | 0.1682 | 0.279 | 通过 |
| `gust` / two_stage | 1.0000 | 0.1746 | 0.542 | 通过 |

比上面的离线表略高，来源只有一个：ROS 里第 k 帧图像对应的指令从 k+1 才生效（50 ms），
而离线评估是同进程零延迟。控制节点改成"检测到达即下发"后，定时器的相位抖动被消掉，
ROS 与本地接口替身的结果一致（详见文档 §6.1；同容器多次运行在 ±0.001 内波动）。

## 场景设计

| 场景 | 目标运动 | 干扰 | 考察点 |
|---|---|---|---|
| `static` | 静止于 10 m 外 | — | 收敛速度与稳态误差 |
| `moving` | 半径 5 m、0.22 rad/s 圆周 | 1 个蓝色干扰方块 | 持续转向跟随、匀速稳态差 |
| `occlusion` | 14 m 处横向匀速穿越立柱 | 3 根 0.56×0.44×2.4 m 立柱 | 丢失—记忆保持—重捕获 |
| `gust` | 0.8 m/s 往复机动 | 阵风（幅值约 0.5 m/s²，双频叠加） | 抗扰动能力 |

## 参数（`params.py`）

| 参数 | 值 | 含义 |
|---|---|---|
| `IMG_W` / `IMG_H` | 160 / 120 | 图像分辨率 |
| `FOV_DEG` / `PITCH_DEG` | 70 / 40 | 水平视场角、相机下俯角 |
| `DT` | 0.05 | 控制周期（20 Hz） |
| `H_DES` | 9.0 m | 期望飞行高度 |
| `V_MAX` / `VZ_MAX` | 3.5 / 1.5 m/s | 速度限幅 |
| `DS` / `E2E_DS` | 16 / 24 | 感知网络与端到端网络的输入边长 |
| `ERR_OK` / `LOST_FRAC_OK` | 0.18 / 0.85 | 成功判定阈值 |

## 单元测试

```bash
cd ~/ros2/src/air/uav_camera_tracking
python3 -m pytest test/ -q     # 有 ROS：7 过 6 跳过；无 ROS：13 过（替身接管）
```

`test/test_offline.py`（7 个）覆盖相机投影方向、遮挡判定、热图标签与解码的格子约定
一致性、专家指令符号、四旋翼速度环收敛与 MLP 拟合——都是"错了会静默失效"的地方。

`test/test_nodes_ros.py`（6 个）用 `test/fake_ros.py`（`rclpy` 与消息、ament、launch 的
替身，只在没有真 rclpy 时启用）把三个节点按真实话题跑起来，验证：参数文件里的键都被
节点声明、`setup.py` 的 `data_files` 与实际文件一致、入口点指向的模块和函数存在、
inspect 后的路径能被 `model_io` 定位、`launch` 把参数正确传给三个节点、以及三节点闭环
在四个场景上收敛。**这 6 个用例在 Ubuntu + Humble 上会自动跳过**，交给真 ROS 去跑
（见 `.github/workflows/ros2_uav_camera_tracking.yml`）。

## 复现环境

- 训练与评估：Ubuntu 20.04 + Python 3.8 + NumPy + Matplotlib（纯 CPU，无 GPU 依赖）
- ROS 2：Humble（Ubuntu 22.04）与 Foxy（Ubuntu 20.04）均可，节点只依赖
  `rclpy`、`sensor_msgs`、`geometry_msgs`、`nav_msgs`
- 构建：`colcon build --packages-select uav_camera_tracking`，无编译步骤

需要说明的边界：本模块的仿真器是自带的解析式渲染器（光线投射），**没有**接 AirSim/CarlaAir。
这样做的目的是让"感知网络必须真的从像素里找目标"这一点在无显卡、无仿真器许可的机器上也能复现；
代价是图像纹理比真实相机简单得多，迁移到 AirSim 图像上需要重新采集训练集（`build_dataset`
的采样接口可直接替换为真实图像 + 标注）。

## 已知限制

- 端到端策略未收敛（见上），保留在代码与结果中而不是删掉，便于对比；
- 目标为单色方块，遮挡为几何遮挡，无烟雾/逆光/尺度剧变；
- 动力学是质点 + 一阶姿态滞后，不含姿态角限制与电机饱和；
- DAgger 只做 2 轮，样本量继续增大可能还有收益。

## 大模型使用声明

本模块的代码骨架、注释与文档在编写过程中使用了大语言模型辅助；所有参数、指标与图表均由
`main.py` 实际运行生成并写入 `results/`，可复现。算法选择（热图检测头、IBVS 纵向误差环、
DAgger）与其中的失败结论（端到端不收敛）来自实际实验对比，而非模型生成。
