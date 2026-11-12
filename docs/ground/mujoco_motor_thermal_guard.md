# ROS 2 电机温度保护节点

节点使用标准 ROS 2 消息类型接收温度测量和速度指令，按可配置预测时域外推温升；预测温度达到上限后限制线速度和角速度，低于解除阈值后恢复。
首次收到有效温度消息前以及温度消息超过 `temperature_timeout_s`（默认 0.5 秒）未更新时，每次收到速度输入均输出六分量全零的指令。有效测温期间，线速度和角速度的全部分量按同一比例缩放。乱序和非有限温度样本被丢弃，不刷新测温有效期。

## 运行

从 ROS 工作区根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select mujoco_motor_thermal_guard
source install/setup.bash
ros2 launch mujoco_motor_thermal_guard thermal_guard.launch.py limit_c:=65 release_c:=60 horizon_s:=5
```

launch 将 `limit_c`、`release_c`、`horizon_s`、`temperature_timeout_s` 显式解析为浮点数，整数形式的启动参数同样可用。默认预测时域为 5 秒。

## 验证

保护逻辑及 ROS 适配共有 5 项测试，覆盖温升外推、迟滞、阈值合法性、时间戳顺序、六分量缩放与失去测温后的停止输出。在已加载 ROS 环境并完成构建后执行：

```bash
colcon test --packages-select mujoco_motor_thermal_guard --python-testing pytest --return-code-on-test-failure
colcon test-result --verbose
```

若构建使用 `--merge-install`，测试时也加入该选项。普通 Python 环境可运行 3 项核心测试，ROS 适配测试需加载 ROS 2 Humble 环境。

Ubuntu 22.04 / ROS 2 Humble 容器中，colcon 构建成功，5 项测试全部通过。使用 `limit_c:=65 release_c:=60 horizon_s:=5` 启动节点，通过标准消息话题检查以下结果：

| 输入与状态 | 输出 |
|---|---|
| 尚未收到有效温度 | 六个速度分量均为 0 |
| 70 °C 有效温度 | 速度比例 0.70，线速度 1.0 m/s → 0.70 m/s，角速度 0.5 rad/s → 0.35 rad/s；其他分量同比例缩放 |
| 温度超过 0.5 秒未更新 | 六个速度分量均为 0 |
| 温度降至解除阈值 60 °C | 原始速度恢复 |
| 随后收到乱序高温样本 | 记录警告并丢弃，保持此前有效状态 |

终止 launch 时，节点正常释放资源并退出。
