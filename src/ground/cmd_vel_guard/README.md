# cmd_vel_guard

地面移动底盘的 `/cmd_vel` 安全监督节点。订阅原始速度指令 `/cmd_vel_raw`，经过限幅、急停、超时、非法值拦截四道保护后发布到 `/cmd_vel`。不需要 Gazebo 或任何仿真环境。

完整的话题与参数说明、演示步骤与验收标准见 [docs/ground/cmd_vel_guard.md](../../docs/ground/cmd_vel_guard.md)。

## 包结构

```
src/ground/cmd_vel_guard/
├── resource/cmd_vel_guard
├── cmd_vel_guard/
│   ├── __init__.py
│   └── cmd_vel_guard_node.py    # 监督节点
├── launch/
│   └── main.launch.py
├── test/
│   └── test_cmd_vel_guard.py    # 单元测试
├── package.xml
├── setup.cfg
├── setup.py
└── README.md
```

## 编译

```bash
cd ~/ros2_ws
colcon build --packages-select cmd_vel_guard
source install/setup.bash
```

## 启动

```bash
ros2 launch cmd_vel_guard main.launch.py
```

launch 暴露 `max_linear_x`、`max_linear_y`、`max_angular_z`、`cmd_timeout`、`publish_rate` 五个参数，例如：

```bash
ros2 launch cmd_vel_guard main.launch.py \
  max_linear_x:=0.2 cmd_timeout:=1.0 publish_rate:=10.0
```

## 测试

```bash
colcon test --packages-select cmd_vel_guard
colcon test-result --verbose
```

用例覆盖 `_clamp` 的正常值、边界值与 `NaN`/`Inf`，急停优先于超时、超时输出全零、非法指令输出全零、正常指令原样透传，急停清除缓存后「解除但不再发送新指令」保持零速、急停期间收到的指令被丢弃，以及非法参数被拒绝。
