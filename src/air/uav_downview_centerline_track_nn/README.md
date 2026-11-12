# uav_downview_centerline_track_nn

下视摄像头标线循迹仿真：下视相机把地面 S 形参考标线投影成 32×24 二值图像，提取标线的横向偏移与航向误差，两层 numpy MLP 策略据此输出前飞与偏航指令，内环完成速度与偏航跟踪。

策略权重是按反馈结构预置的确定值，**未经过训练**，隐藏层只有 3 个独立通道；感知特征是解析投影得到的几何量，图像网格目前只用于可视化。两点都在「已知限制」里写明。

## 概览

| 项目 | 内容 |
| --- | --- |
| 话题类别 | 任务 2：传感器感知及运动控制（下视视觉 + 结构化 MLP 策略） |
| 平台 | Ubuntu 22.04 / ROS 2 Humble（仓库只有 ROS 2 包，无 ROS 1 版本）；离线仿真可在 Windows Python 3.13 运行 |
| 语言 | Python 3 |
| 依赖 | rclpy、nav_msgs、std_msgs（ROS 节点）；离线仅依赖 numpy、pygame |
| 运行方式 | `ros2 launch uav_downview_centerline_track_nn uav_downview_centerline_track_nn.launch.py`；`ros2 run uav_downview_centerline_track_nn track_node`；离线 `python main.py --shots` |
| 仿真器 | 内置 numpy 刚体平面运动模型 + pygame 离屏/窗口渲染 |

## 原理

- 下视相机按几何投影把地面标线采样点投到 32×24 二值图像网格，并在机体系 `fwd/left` 坐标上提取横向偏移（采样窗口均值）与航向误差（`atan` 拟合斜率）。二值网格与特征同源，但控制回路使用的是投影后的几何特征，不是从像素里读出来的。
- 归一化特征 `[offset, heading, speed]` 送入两层 MLP（输入 3、隐藏层 8、输出 2），隐藏层与输出层均 tanh 激活，输出前飞指令与偏航指令。
- 权重为结构化预置值：隐藏层保留 3 个独立通道（偏移、航向、速度各一），输出层连接 3 个隐藏单元，等价于「线性映射 + tanh」。
- 内环把策略指令换算为速度与偏航角速度，叠加切线航向 **P 保持**（`kp_tangent` 单项，无微分项），速度走一阶阻尼。

## 快速运行

### ROS 2 Humble（工作区根目录）

```bash
colcon build --packages-select uav_downview_centerline_track_nn --symlink-install
source install/setup.bash
ros2 launch uav_downview_centerline_track_nn uav_downview_centerline_track_nn.launch.py
# 等价地直接起节点
ros2 run uav_downview_centerline_track_nn track_node
# 改控制频率（launch 参数已透传到节点）
ros2 launch uav_downview_centerline_track_nn uav_downview_centerline_track_nn.launch.py sim_rate:=20.0
```

### 离线仿真（无需 ROS）

```bash
python main.py --shots    # 离屏运行并保存全过程截图到 docs/img/
python main.py --demo     # 打开观察窗口，查看下视循迹过程
```

## 测试

```bash
python -m pytest test/ -q     # 9 项：感知 3、策略 3、控制器 3
```

## 话题与参数

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/odom` | `nav_msgs/Odometry` | 机体位置与线速度；当前只填 `pose.position` 与 `twist`，`child_frame_id` 未赋值、`orientation` 保持单位四元数，未发布 TF |
| `/perception` | `std_msgs/Float32MultiArray` | `[横向偏移, 航向误差, 前飞速度]` |

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `sim_rate` | 50.0 | 控制频率 Hz，launch 通过 `sim_rate:=` 覆盖 |

## 已知限制

1. **感知不是像素级视觉**：`perception.observe()` 由解析真值标线 `track.reference_y` 采样后投影，`image` 只是这些样本点的渲染结果；把 `image` 换成随机噪声，控制输出不变。像素级提取（逐列质心 → 拟合直线 → 偏移/航向）属后续工作。
2. **偏移取窗口均值**：`offset = mean(left)`，窗口 `fwd ∈ [-0.15, 0.85]`；S 形远端曲率会把均值拉向 0，近端已偏出中心时读数偏小，更合适的是取 `fwd≈0` 处的插值或近端加权。
3. **策略未训练**：权重结构化预置，隐藏层只有 3 个独立通道；`test_weights_not_collapsed` 只校验方差非零，测不出「有效单元只有 3 个」。
4. **丢失标线未处理**：`lost=True` 时特征置 0，节点未检查 `lost`，仍按直飞下发，未做悬停或搜索。
5. **里程计接口不完整**：`child_frame_id` 为空、`orientation` 未写 yaw，也没有 `odom→base_link` 的 TF，RViz 里看不到机体系。
6. **重复感知**：`track_node._publish()` 内又调用一次 `camera.observe()`，每周期算两遍，只增加开销、不影响结果。
7. **指标口径**：14 s、50 Hz 闭环，RMS 取 t ≥ 3 s 的 550 个样本；`max_abs_lateral` 含起步瞬时偏差。
8. **实测边界**：只在 Windows 上跑通离线仿真与单元测试；Ubuntu + ROS 2 的构建与 launch 联调、真实相机标定与实时识别未实测。

## 文件结构

```
uav_downview_centerline_track_nn/
├── launch/uav_downview_centerline_track_nn.launch.py
├── uav_downview_centerline_track_nn/
│   ├── track.py        # 地面参考标线曲线
│   ├── perception.py   # 下视相机投影与标线特征提取
│   ├── policy.py       # 两层 MLP 策略（结构化预置权重）
│   ├── controller.py   # 速度与偏航跟踪控制器
│   └── track_node.py   # ROS 2 封装节点
├── test/               # 感知、策略、控制器单元测试
├── main.py             # 离线一键实验入口
├── package.xml
├── setup.py
└── setup.cfg
```

## 许可

MIT
