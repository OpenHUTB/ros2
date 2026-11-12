# heartbeat_watchdog

上游心跳看门狗节点：监视指定节点的 `/heartbeat`，超时即判定失联并输出急停。
完整说明见 [docs/ground/heartbeat_watchdog.md](../../../docs/ground/heartbeat_watchdog.md)。

## 包结构

```
src/ground/heartbeat_watchdog/
├── resource/heartbeat_watchdog
├── heartbeat_watchdog/
│   ├── __init__.py
│   └── heartbeat_watchdog_node.py    # 看门狗节点
├── launch/
│   └── main.launch.py
├── test/
│   └── test_heartbeat_watchdog.py
├── package.xml
├── setup.cfg
├── setup.py
└── README.md
```

## 编译

```bash
cd ~/ros2_ws
colcon build --packages-select heartbeat_watchdog
source install/setup.bash
```

## 启动

```bash
ros2 launch heartbeat_watchdog main.launch.py
```

带参数启动，例如同时监视两个节点、超时改成 0.5 秒：

```bash
ros2 launch heartbeat_watchdog main.launch.py \
    watched_nodes:=navigator,controller heartbeat_timeout:=0.5
```

## 测试

```bash
colcon test --packages-select heartbeat_watchdog
colcon test-result --verbose
```
