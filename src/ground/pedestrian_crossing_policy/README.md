# pedestrian_crossing_policy

基于 TTC（time-to-collision）的行人过街决策模块。它把“附近有没有车”升级为
“车辆是否会在可接受时间内到达冲突点”，并提供独立的 ROS 2 适配节点。

## 环境

- Python 3.8+
- 可选：ROS 2 Humble、`rclpy`、`std_msgs`

纯策略和测试不依赖 ROS 2。

## 直接运行

```bash
cd src/ground/pedestrian_crossing_policy
python main.py --distance 10 --closing-speed 10
```

输出示例：

```text
wait=True reason=ttc_below_threshold ttc=1.000s
```

不带参数运行会执行一个内置自检。

## ROS 2

先在一个终端订阅输出，再在另一个终端启动节点并发布一次观测。`Bool` 使用
默认 volatile QoS，反过来先 pub 再 echo 会漏掉消息。

```bash
# 终端 A
ros2 topic echo /pedestrian_crossing_policy/wait

# 终端 B
ros2 launch pedestrian_crossing_policy main.launch.py
ros2 topic pub --once /pedestrian_crossing_policy/observation \
  std_msgs/msg/Float32MultiArray "{data: [10.0, 10.0]}"
```

输入 `[distance_m, closing_speed_mps]`，输出 `Bool`。`closing_speed <= 0.1`
视为车辆没有接近。

## 测试

```bash
python -m unittest discover -s test -p "test_*.py" -v
```

## 限制

本模块只实现单行人的间隙接受判据，不包含 CARLA 场景、轨迹预测或群体行为。
接入 HUTB/CARLA 时，由 `carla_walker_agent` 或自定义桥接节点发布观测并消费
`wait`。
