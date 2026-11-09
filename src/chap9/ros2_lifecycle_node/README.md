# ros2_lifecycle_node — 托管节点：先配置，再干活

普通节点一构造就发布，出问题时你分不清「数据不对」还是「依赖还没起来」。
生命周期节点把这件事拆开：构造后停在 `unconfigured`，必须依次收到
`configure`、`activate` 才开始工作，启停可以反复执行。

## 状态机

```
unconfigured --configure--> inactive --activate--> active
     ^                          |                     |
     |______cleanup_____________|____deactivate_______|
                    shutdown -> finalized
```

非法转移会抛异常（`InvalidTransition`），不会「悄悄什么都不做」。

## 目录

```
ros2_lifecycle_node/
├── scripts/
│   ├── lifecycle.py    # 四状态机 + 回调 + 发布门控（纯 Python，可离线测）
│   ├── simulate.py     # 时间轴脚本：configure/activate/deactivate/cleanup
│   ├── main.py         # 打印时间轴 + 生成状态图与累计曲线
│   └── ros_node.py     # 真正的 rclpy LifecycleNode
├── tests/test_lifecycle.py  # 26 项单元测试
└── launch/ros2_lifecycle_node.launch.py
```

## 快速开始

```bash
python3 scripts/main.py         # 打印时间轴 + 3 张 PNG
python3 tests/test_lifecycle.py # 26 项单元测试
```

## 结果

时间轴 14 s、每 0.1 s 一个 tick，每个 tick 尝试发一条心跳：

| 指标 | 数值 |
|---|---|
| tick 总数 | 141 |
| 发布被接受 | 70 |
| 发布被丢弃（未激活） | 71 |
| 激活时间占比 | 49.6% |
| 进入 active 次数 | 2 |
| 合法 / 非法转移 | 7 / 2 |
| 终态 | finalized |

被拒绝的两条：`inactive` 下再 `configure`、`unconfigured` 下 `activate`——都被
状态机挡住并给出合法动作列表。

## 在 ROS 2 中运行

```bash
colcon build --packages-select ros2_lifecycle_node
source install/setup.bash
ros2 launch ros2_lifecycle_node ros2_lifecycle_node.launch.py

# 另开终端
ros2 lifecycle get /lifecycle_heartbeat
ros2 lifecycle set /lifecycle_heartbeat configure
ros2 lifecycle set /lifecycle_heartbeat activate
ros2 topic echo /heartbeat          # 现在才有数据
ros2 lifecycle set /lifecycle_heartbeat deactivate
ros2 lifecycle set /lifecycle_heartbeat cleanup
```

想看 configure 失败会怎样：`-p fail_on_configure:=true`，节点会停在
`unconfigured`，任何 `activate` 都被拒绝。

## 关于验证范围

状态机、发布门控与统计数值均由离线单测覆盖（26 项全过）。`ros2 lifecycle set`
等 CLI 命令未在装了 ROS 2 发行版的机器上实测，节点按 `rclpy.lifecycle` 标准
接口编写。
