# estop_latch

急停闩锁节点：把 `/emergency_stop` 上的安全事件锁住，故障消失也不会自动放行，必须调用
`/estop_latch/reset` 服务人工确认。默认上电即闩锁。
完整说明见 [docs/ground/estop_latch.md](../../../docs/ground/estop_latch.md)。

## 包结构

```
src/ground/estop_latch/
├── resource/estop_latch
├── estop_latch/
│   ├── __init__.py
│   └── estop_latch_node.py    # 闩锁节点
├── launch/
│   └── main.launch.py
├── test/
│   └── test_estop_latch.py
├── package.xml
├── setup.cfg
├── setup.py
└── README.md
```

## 编译

```bash
cd ~/ros2_ws
colcon build --packages-select estop_latch
source install/setup.bash
```

## 启动

```bash
ros2 launch estop_latch main.launch.py
```

人工复位：

```bash
ros2 service call /estop_latch/reset std_srvs/srv/Trigger
```

调整参数，例如要求急停消失 3 秒后才允许复位：

```bash
ros2 launch estop_latch main.launch.py release_hold_time:=3.0
```

上游是周期发布（如 `heartbeat_watchdog`）时可开启链路监视：

```bash
ros2 launch estop_latch main.launch.py input_timeout:=1.0
```

## 测试

```bash
colcon test --packages-select estop_latch
colcon test-result --verbose
```
