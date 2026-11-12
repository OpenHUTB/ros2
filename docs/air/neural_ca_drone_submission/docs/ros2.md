# ROS 2（Humble）部署

## 1. 包结构

```
ros2_ws/src/neural_ca_drone_ros2/
├── package.xml
├── setup.py / setup.cfg
├── resource/neural_ca_drone_ros2
├── launch/
│   ├── main.launch.py
│   ├── full.launch.py
│   ├── perception.launch.py
│   ├── planning.launch.py
│   ├── control.launch.py
│   └── e2e.launch.py
├── neural_ca_drone_ros2/
│   ├── common.py            # 路径、深度解码、四元数
│   ├── main_node.py         # 推理节点
│   └── sim_bridge_node.py   # CA-Sim → ROS 2 桥
└── scripts/
    ├── main.py              # main. 入口
    ├── sim_bridge.py
    └── probe.py             # 无 CLI 依赖的话题探针
```

## 2. 构建

```bash
cd neural_ca_drone && pip3 install -e . && cd ..
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select neural_ca_drone_ros2
source install/setup.bash
```

实测：

```
Starting >>> neural_ca_drone_ros2
Finished <<< neural_ca_drone_ros2 [3.52s]
Summary: 1 package finished [3.65s]
```

## 3. 运行

```bash
ros2 launch neural_ca_drone_ros2 main.launch.py module:=full scene:=sparse
ros2 launch neural_ca_drone_ros2 e2e.launch.py scene:=dense
```

参数：`module`、`scene`、`rate`、`goal_x`、`goal_y`、`replan_period`。

## 4. 验证

本机的自制 Humble 镜像里 `ros2 topic echo` 依赖的 daemon 不工作：

```
xmlrpc.client.Fault: <Fault 1: "<class 'RuntimeError'>:!rclpy.ok()">
```

这是镜像问题，不是包的问题。用 rclpy 直接订阅的 `probe.py` 验证：

```bash
python3 install/neural_ca_drone_ros2/lib/neural_ca_drone_ros2/probe.py
```

实测输出：

```
CMD 1.193 0.133 1.045
STATUS module=full cmd=(1.19,0.13) yaw_rate=1.05 speed=1.20
RESULT ok
```

深度话题：

```
$ ros2 topic hz /camera/depth/image_raw
average rate: 10.033
```

## 5. 与 ROS 1 的差异

| 项 | ROS 1 | ROS 2 |
| --- | --- | --- |
| 构建 | catkin_make | colcon build |
| 参数 | `<param>` | `DeclareLaunchArgument` |
| 消息 | `ros/...` | `rclpy` 同名类型 |
| 时间 | `rospy.Time` | `node.get_clock()` |
| 桥接 | `roscpp` 可选 | 纯 rclpy |

算法代码（`ca_drone/`）完全复用，两个包只是适配层，避免重复实现。

## 6. 常见问题

**Q: `colcon build` 报 `ModuleNotFoundError: ca_drone`**
A: 先 `pip3 install -e .`，再 `colcon build`。

**Q: launch 起来了但 `/cmd_vel` 没有消息**
A: 检查 `sim_bridge` 是否在跑；`ros2 node list` 应有两个节点。

**Q: `probe.py` 超时**
A: `module` 参数写错，或深度话题没有发布（`sim_bridge` 挂了）。
