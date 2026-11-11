# tf_health_monitor

TF 链路健康监视节点：把 `/tf` 与 `/tf_static` 的每条 `(parent, child)` 边聚合成
新鲜度与发布频率，识别断链、时间戳回退、`TF_OLD_DATA`、多 parent 与频率不足。

完整说明见 [docs/ground/tf_health_monitor.md](../../../docs/ground/tf_health_monitor.md)。

## 包结构

```
src/ground/tf_health_monitor/
├── resource/tf_health_monitor
├── tf_health_monitor/
│   ├── __init__.py
│   ├── core.py                      # 判据，不依赖 ROS，可离线回归
│   ├── tf_health_monitor_node.py    # 监视节点
│   └── tf_demo_publisher.py         # 演示数据源，制造四类 TF 异常
├── launch/
│   └── main.launch.py
├── test/
│   └── test_tf_health_monitor.py
├── package.xml
├── setup.cfg
├── setup.py
└── README.md
```

## 编译

```bash
cd ~/ros2_ws
colcon build --packages-select tf_health_monitor
source install/setup.bash
```

## 启动

```bash
ros2 launch tf_health_monitor main.launch.py
```

带参数启动，例如只看两条指定边、把频率门限放宽到 0.5 Hz：

```bash
ros2 launch tf_health_monitor main.launch.py \
  min_rate:=0.5 watched:='map->odom,odom->base_footprint'
```

## 演示

监视节点不需要底盘或仿真器。另开一个终端跑演示数据源，它会在 8 s、18 s、26 s
分别制造断链、`TF_OLD_DATA` 和多 parent：

```bash
ros2 run tf_health_monitor tf_demo_publisher
```

## 测试

判据全部集中在 `core.py`，不依赖 rclpy，因此没有 ROS 环境的机器也能直接跑：

```bash
cd src/ground/tf_health_monitor
python3 -m pytest test -v
```

在 ROS 工作区里则用：

```bash
colcon test --packages-select tf_health_monitor
colcon test-result --verbose
```
