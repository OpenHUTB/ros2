# ROS 2 传感器数据均值滤波 (Mean Filter) 节点

## 项目简介

本项目基于 ROS 2 (Humble) 和 Python (rclpy) 实现一个**滑动窗口均值滤波节点**。
节点订阅原始的传感器数据话题，使用固定长度的滑动窗口计算均值，
从而抑制数据中的突变噪声，并将平滑后的结果发布出去。

## 功能与原理

节点使用 `collections.deque(maxlen=window_size)` 维护滑动窗口：
每收到一条数据就加入窗口，并发布当前窗口内所有样本的平均值。

* 窗口未填满时按已有样本求均值，不会把未采集的数据当作 0；
* 窗口填满后自动丢弃最早的样本；
* 窗口长度为 `N` 时，阶跃输入需要 `N` 个新样本才能完全跟随，
  平滑效果与响应延迟随窗口增大而增大。

## 项目目录结构

本模块位于 `src/chap2/mean_filter_pkg`，结构如下：

```text
mean_filter_pkg/
├── main.py                       # 模块入口（launch 方式启动）
├── launch/
│   └── mean_filter.launch.py     # ros2 launch 启动文件
├── package.xml                   # 依赖声明
├── setup.py                      # 节点入口与安装配置
├── setup.cfg
├── README.md                     # 包内简要说明
├── resource/
│   └── mean_filter_pkg
├── test/                         # 算法单元测试与节点集成测试
└── mean_filter_pkg/
    ├── __init__.py
    └── mean_filter_node.py       # 滤波器与节点实现
```

## 核心参数

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `input_topic` | string | `/sensor/raw` | 原始数据订阅话题 |
| `output_topic` | string | `/sensor/filtered` | 滤波结果发布话题 |
| `window_size` | int | `5` | 窗口长度，必须为正整数；传入 0 或负数时节点抛出 `ValueError` 并退出 |

## 依赖环境

- WSL2 Ubuntu 22.04
- ROS 2 Humble
- Python 3.10（系统默认，Humble 官方二进制）
- `rclpy`、`std_msgs`、`launch`、`launch_ros`（ROS 2 自带）

## 编译与运行步骤

1. **编译工作空间**（在工作空间根目录执行）

```bash
colcon build --packages-select mean_filter_pkg
source install/setup.bash
```

2. **启动滤波节点**（推荐使用 launch）

```bash
ros2 launch mean_filter_pkg mean_filter.launch.py
```

也可以通过 launch 参数覆盖话题与窗口大小：

```bash
ros2 launch mean_filter_pkg mean_filter.launch.py \
  window_size:=3 input_topic:=/demo/raw output_topic:=/demo/filtered
```

等价的启动方式：

```bash
ros2 run mean_filter_pkg mean_filter
python3 src/chap2/mean_filter_pkg/main.py
```

3. **发送测试数据**（在另一个终端中运行）

```bash
# 先发布 2 条 10.0，再发布 5 条 20.0，形成一次阶跃
ros2 topic pub -r 2 --times 2 /sensor/raw std_msgs/msg/Float64 "{data: 10.0}"
ros2 topic pub -r 2 --times 5 /sensor/raw std_msgs/msg/Float64 "{data: 20.0}"
```

4. **查看滤波结果**（在第三个终端中运行）

```bash
ros2 topic echo /sensor/filtered
```

当窗口为 5 时，`ros2 topic echo /sensor/filtered` 实际打印如下
（`std_msgs/Float64` 按 Python float 原样输出，不作小数位对齐）：

```text
data: 10.0
---
data: 10.0
---
data: 13.333333333333334
---
data: 15.0
---
data: 16.0
---
data: 18.0
---
data: 20.0
---
```

即窗口未填满前按已有样本平均，窗口填满后表现为线性爬升，
最终稳定在阶跃后的 20.0。

## 自动化测试

本包为 `ament_python` 构建类型，测试用例使用 pytest 风格。colcon 默认用
`python3 -m unittest` 收集用例，必须显式指定 `--python-testing pytest` 才会执行，
因此在已 source 工作空间的环境中执行：

```bash
colcon test --packages-select mean_filter_pkg --python-testing pytest
colcon test-result --verbose --test-result-base build/mean_filter_pkg
```

不加 `--python-testing pytest` 时收集到的是 0 个用例。

也可以直接在包目录里运行 pytest：

```bash
cd src/chap2/mean_filter_pkg
python3 -m pytest test -v
```

测试覆盖：

* 非法 `window_size`（0、负数、非整数）的参数校验；
* 滑动窗口的预热、阶跃响应与旧样本淘汰行为；
* ROS 节点通过话题收发数据时的滤波结果。

## 运行效果展示

### 1. 系统编译与基础通信

![系统编译](../img/01_Project_Compilation_and_Basic_Communication.png)

### 2. 均值滤波数据平滑效果验证

![滤波效果](../img/02_Mean_Filter_Data_Smoothing_Verification.png)

## 设计说明与已知限制

* 固定窗口均值对窗口内样本等权，窗口越大越平滑，但对快速变化的跟随越慢；
* 节点只处理 `std_msgs/msg/Float64` 单变量数据；多路传感器可分别部署节点，
  或改用 `Float64MultiArray` 扩展；
* 均值滤波无法抑制与信号同频段的噪声，需要更强抑噪时可改用中值滤波或低通滤波；
* 若话题没有数据，请先确认发布端与订阅端使用同一 ROS_DOMAIN_ID，
  且终端只 source 了当前 ROS 2 版本的环境。
