# odom_health_monitor

里程计健康监视节点：比较 `/cmd_vel` 的期望速度与 `/odom` 的实测速度，识别打滑、堵转与反向运动。
完整说明见 [docs/ground/odom_health_monitor.md](../../../docs/ground/odom_health_monitor.md)。

## 包结构

```
src/ground/odom_health_monitor/
├── resource/odom_health_monitor
├── odom_health_monitor/
│   ├── __init__.py
│   └── odom_health_monitor_node.py    # 监视节点
├── launch/
│   └── main.launch.py
├── test/
│   └── test_odom_health_monitor.py
├── package.xml
├── setup.cfg
├── setup.py
└── README.md
```

## 编译

```bash
cd ~/ros2_ws
colcon build --packages-select odom_health_monitor
source install/setup.bash
```

## 启动

```bash
ros2 launch odom_health_monitor main.launch.py
```

带参数启动，例如把打滑阈值放宽到 2.0：

```bash
ros2 launch odom_health_monitor main.launch.py slip_ratio:=2.0
```

## 测试

```bash
colcon test --packages-select odom_health_monitor
colcon test-result --verbose
```
