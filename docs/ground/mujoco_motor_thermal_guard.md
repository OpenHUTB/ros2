# ROS 2 电机温度保护节点

节点使用标准 ROS 2 消息类型接收温度测量和速度指令，按可配置预测时域外推温升；预测温度达到上限后限制线速度和角速度，低于解除阈值后恢复。
首次收到温度消息前以及温度消息超过 0.5 秒未更新时，节点将速度指令置零。

## 验证

与 ROS 无关的保护逻辑由测试覆盖，包括温升外推、迟滞、阈值合法性和时间戳顺序：

```bash
PYTHONPATH=src/ground/mujoco_motor_thermal_guard python -m pytest -q src/ground/mujoco_motor_thermal_guard/test
```

启动节点和验证话题通信需在已加载 ROS 2 Humble 环境中运行。
