# uav_vision_nn — AirSim 无人机视觉端到端控制（行为克隆）

用单目相机图像直接回归无人机机体系速度指令 `(vx, vy)`，整条链路由 ROS2 Humble
节点组成：AirSim 桥 → 数据采集 → 离线训练 → 在线推理。这是
[OpenHUTB/ros2](https://github.com/OpenHUTB/ros2) 课程任务 (4)（端到端模型）的实现。

- 设计文档：[docs/air/uav_vision_nn.md](../../docs/air/uav_vision_nn.md)（含公式、算法流程、源码解析）
- 与仓库已有 `#113`（YOLO 检测 + 规则控制）的区别：那条链路是「检测框 → 手工规则 → 指令」，
  本模块把感知与控制合成一个网络，直接从像素回归控制量，不需要人工设计规则。

## 目录结构

```
uav_vision_nn/
├── package.xml
├── setup.py / setup.cfg
├── resource/uav_vision_nn
├── config/params.yaml
├── launch/
│   ├── main.launch.py           # 桥 + 推理
│   └── collect.launch.py   # 桥 + 采集
└── uav_vision_nn/
    ├── model.py          # 唯一的网络定义与图像预处理
    ├── airsim_source.py  # AirSim ↔ ROS2 桥（图像/位姿上行，cmd_vel 下行）
    ├── collect.py        # 专家示教数据采集
    ├── train.py          # 离线训练与指标输出
    └── main.py           # 总入口：推理节点
```

## 运行环境

| 项目 | 版本 |
| --- | --- |
| 操作系统 | Ubuntu 22.04 |
| ROS | ROS2 Humble |
| 仿真器 | AirSim 1.8.1（Windows 宿主机跑 Blocks，或 Linux 本机） |
| Python | 3.10 |
| 深度学习 | PyTorch 2.x（CPU 即可，模型只有几十万参数） |

安装依赖：

```bash
sudo apt install python3-numpy python3-opencv python3-matplotlib ros-humble-desktop
pip3 install torch --index-url https://download.pytorch.org/whl/cpu
pip3 install airsim            # 没有 rosdep key，只能用 pip
```

> 虚拟机通过 NAT 访问宿主机时，AirSim 地址是 `10.0.2.2`；Linux 本机跑仿真则用
> `airsim_ip:=127.0.0.1`。

## 构建

```bash
cd ~/ros2
colcon build --packages-select uav_vision_nn
source install/setup.bash
```

## 运行步骤

### 1. 启动仿真

Windows 宿主机双击 `Blocks.exe`（AirSim），确认 41451 端口在监听。

### 2. 起飞

```bash
ros2 launch uav_vision_nn collect.launch.py
# 另开终端
ros2 service call /uav/takeoff std_srvs/srv/Trigger
```

爬升到 1 m 以上采集节点才开始记录，终端会打印 `检测到已起飞`。

### 3. 采集专家数据

采集节点默认是 `waypoint` 模式：自己按 `config/params.yaml` 里的航点飞一圈，
把「当前图像 + 当前下发速度」成对存到 `~/.ros/uav_vision_nn/data`。

```bash
ros2 launch uav_vision_nn collect.launch.py \
  mode:=waypoint data_dir:=$HOME/.ros/uav_vision_nn/data
```

换个地图 / 换个航点再采一轮，数据越多越好（建议 ≥ 2000 帧）。航点走完后节点打印
`航点全部到达`，按 `Ctrl-C` 结束。

也可以遥控采样：`mode:=follow` 时节点不发布指令，只旁听 `/uav/cmd_vel`
（例如接 `uav_airsim_keyboard` 的键盘遥控），把人工操纵当作专家。

### 4. 训练

```bash
ros2 run uav_vision_nn train -- --epochs 50
```

输出到 `~/.ros/uav_vision_nn/`：

| 文件 | 说明 |
| --- | --- |
| `model.pth` | 验证损失最低的权重 |
| `loss_curve.png` | 训练/验证 MSE 曲线（对数纵轴） |
| `metrics.json` | 测试集 MSE、MAE、分维 MAE、样本量、耗时 |

常用参数：`--epochs 50 --batch-size 32 --lr 1e-3 --image-size 64 --seed 42 --data-dir ...`。
8:1:1 划分，固定随机种子，同一份数据重跑结果一致。

### 5. 推理（闭环）

```bash
ros2 launch uav_vision_nn main.launch.py
ros2 service call /uav/takeoff std_srvs/srv/Trigger
```

节点订阅 `/uav/image`，以 10 Hz 发布 `/uav/cmd_vel`，桥把它变成 AirSim 的
`moveByVelocityBodyFrameAsync`。没有新指令超过 `cmd_timeout` 秒会自动悬停。

单独跑推理节点（不经过 launch）时需要手动 remap：

```bash
ros2 run uav_vision_nn main --ros-args \
  -r ~/image:=/uav/image -r ~/cmd_vel:=/uav/cmd_vel \
  -p model_path:=$HOME/.ros/uav_vision_nn/model/model.pth
```

## 接口

### 话题

| 话题 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/uav/image` | `sensor_msgs/Image` | 桥 → 推理/采集 | FPV 相机，`bgr8`，10 Hz |
| `/uav/depth` | `sensor_msgs/Image` | 桥 → 其他 | 深度图，`32FC1`，5 Hz，米 |
| `/uav/pose` | `geometry_msgs/PoseStamped` | 桥 → 采集 | 位置，`world_ned`，20 Hz |
| `/uav/cmd_vel` | `geometry_msgs/Twist` | 推理/采集 → 桥 | `linear.x` 前向、`linear.y` 右向、`linear.z` 向下，m/s |
| `/uav/infer_ms` | `std_msgs/Float32` | 推理 → 监控 | 单帧推理耗时，毫秒 |

### 服务

| 服务 | 类型 | 说明 |
| --- | --- | --- |
| `/uav/takeoff` | `std_srvs/srv/Trigger` | 起飞，爬升到 1 m 后转为速度控制 |
| `/uav/land` | `std_srvs/srv/Trigger` | 降落，落地后自动解除解锁 |

### 参数

见 `config/params.yaml`，全部可用 `-p 名字:=值` 覆盖，节点里没有硬编码路径。

| 节点 | 参数 | 默认值 |
| --- | --- | --- |
| `airsim_source` | `airsim_ip` / `airsim_port` | `10.0.2.2` / `41451` |
| `airsim_source` | `frame_rate` / `pose_rate` / `depth_rate` / `control_rate` | 10 / 20 / 5 / 10 Hz |
| `airsim_source` | `cmd_timeout` | 0.5 s |
| `uav_vision_node` | `model_path` | `~/.ros/uav_vision_nn/model/model.pth` |
| `uav_vision_node` | `image_size` / `max_speed` / `publish_rate` / `device` | 64 / 2.0 / 10.0 / cpu |
| `uav_vision_collect` | `data_dir` / `sample_rate` / `mode` | `~/.ros/uav_vision_nn/data` / 2.0 / waypoint |
| `uav_vision_collect` | `waypoints` / `altitude` / `gain` / `reach_radius` | 见 yaml |

## 性能评价方法

不写死结论，按下面的方法自己量，数据直接填进 `metrics.json` 和文档：

1. **开环精度**：`metrics.json` 里的 `test.mse`（m²/s²）与 `test.mae`（m/s），
   以及分维 `mae_vx` / `mae_vy`，衡量「同一帧图像上网络输出和专家动作差多少」。
2. **闭环跟踪误差**：让无人机按固定航点飞，同时记录 `/uav/pose`，
   计算实际轨迹到航点折线的平均偏差，单位米。
3. **推理耗时**：`ros2 topic echo /uav/infer_ms` 取 200 帧的均值与 95 分位，单位毫秒。

## 常见问题

- **连不上 AirSim**：用 `airsim_ip:=` 显式指定；Windows 防火墙放开 41451；确认虚拟机是 NAT 模式。
- **`colcon build` 报找不到 launch**：`setup.py` 里 `glob('launch/*.launch.py')` 需要文件真实存在，检查是否漏检。
- **训练报样本太少**：至少 10 帧才能划分，建议先采 200 帧跑通流程再扩大。
- **飞机不动**：确认已调用 `/uav/takeoff`，且 `/uav/cmd_vel` 有数据（`ros2 topic hz`）。
