# mean_filter_pkg

ROS 2 滑动窗口均值滤波节点。订阅单路浮点传感器数据，按固定长度窗口计算
均值并发布滤波结果，用于抑制阶跃突变与随机噪声。

详细教程（实验环境、阶跃实验步骤与运行截图）见
[docs/chap2/mean_filter.md](../../../docs/chap2/mean_filter.md)。

## 参数

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `input_topic` | string | `/sensor/raw` | 原始数据订阅话题 |
| `output_topic` | string | `/sensor/filtered` | 滤波结果发布话题 |
| `window_size` | int | `5` | 窗口长度，必须为正整数；传入 0 或负数时节点抛出 `ValueError` |

窗口未填满时按已有样本求均值，因此启动阶段不会把未采集的数据当作 0。

## 目录结构

```text
mean_filter_pkg/
├── main.py                    # 模块入口（launch 方式启动）
├── launch/
│   └── mean_filter.launch.py  # ros2 launch 启动文件
├── mean_filter_pkg/
│   └── mean_filter_node.py    # 滤波器与节点实现
├── test/
│   ├── test_mean_filter.py    # 算法单元测试 + ROS 节点集成测试
│   ├── test_flake8.py
│   └── test_pep257.py
├── package.xml
├── setup.py
└── setup.cfg
```

## 运行环境

- WSL2 Ubuntu 22.04
- ROS 2 Humble
- Python 3.10（Humble 官方二进制自带）

## 编译与运行

在工作空间根目录执行：

```bash
colcon build --packages-select mean_filter_pkg
source install/setup.bash
```

推荐使用 launch 启动，并通过参数覆盖话题与窗口大小：

```bash
ros2 launch mean_filter_pkg mean_filter.launch.py
ros2 launch mean_filter_pkg mean_filter.launch.py \
  window_size:=3 input_topic:=/demo/raw output_topic:=/demo/filtered
```

等价启动方式：

```bash
ros2 run mean_filter_pkg mean_filter
python3 src/chap2/mean_filter_pkg/main.py
```

发送测试数据并查看结果：

```bash
ros2 topic pub -r 2 --times 2 /sensor/raw std_msgs/msg/Float64 "{data: 10.0}"
ros2 topic pub -r 2 --times 5 /sensor/raw std_msgs/msg/Float64 "{data: 20.0}"
ros2 topic echo /sensor/filtered
```

窗口为 5 时，上述阶跃的输出依次为
`10.0, 10.0, 13.333333333333334, 15.0, 16.0, 18.0, 20.0`。

## 测试

本包为 `ament_python` 构建类型，`package.xml` 中声明了
`<test_depend>python3-pytest</test_depend>`。
colcon 默认用 `python3 -m unittest` 收集用例，pytest 风格的文件必须显式指定
`--python-testing pytest` 才会执行：

```bash
colcon test --packages-select mean_filter_pkg --python-testing pytest
colcon test-result --verbose --test-result-base build/mean_filter_pkg
```

`setup.cfg` 中的 `[tool:pytest]` 只对直接运行 pytest 生效，不会改变 colcon
的测试运行器，所以不加 `--python-testing pytest` 时收集到的是 0 个用例。
也可以直接在包目录里运行 pytest：

```bash
cd src/chap2/mean_filter_pkg
python3 -m pytest test -v
```

测试覆盖参数校验、滑动窗口行为以及 ROS 话题收发的滤波结果。

## 设计说明

* 固定窗口均值对窗口内样本等权：窗口越大越平滑，但对阶跃的跟随越慢，
  窗口长度为 `N` 时需要 `N` 个新样本才能完全跟随新值。
* 当前节点只处理 `std_msgs/msg/Float64` 单变量；多路传感器可分别部署节点，
  或改用 `Float64MultiArray` 扩展。
