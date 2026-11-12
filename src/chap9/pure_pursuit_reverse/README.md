# pure_pursuit_reverse

差速小车**倒车**纯追踪路径跟踪，含转向角速率限制与到达停车。

## 要点

- 路径弧长参数化，前视点按 `s + L_d` 插值取得，路径顶点顺序即行进方向
- 倒车时把车头方向取反得到行进方向，曲率 `kappa = 2 sin(alpha) / L`
- 转向角速率限制 `max_steer_rate`，末端 0.15 m 内停车并输出横向误差

## 环境（实测）

- Ubuntu 20.04 + ROS 2 Foxy，Python 3.8.10
- 无 Gazebo 依赖：仿真在包内节点完成

## 构建与运行

```bash
cd ~/ros2_ws
colcon build --packages-select pure_pursuit_reverse
source install/setup.bash
ros2 launch pure_pursuit_reverse main.launch.py
```

## 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `speed` | 0.25 | 倒车线速度绝对值 (m/s) |
| `lookahead` | 0.9 | 前视距离 (m) |
| `max_steer_rate` | 1.2 | 角速度变化率上限 (rad/s²) |

## 没有 ROS 也能复现

```bash
cd src/chap9/pure_pursuit_reverse
python3 main.py --no-ros
# offline reverse pure pursuit
#   t=30.10 s  s=8.25/8.40 m  pos=(7.85, -0.01)
#   max lateral error = 0.1084 m
```

`--no-ros` 用与节点相同的控制律（`core.py` 的纯函数）加 `path_sim` 的积分方式离线跑一遍，
不依赖 rclpy，方便没有 ROS 的环境复现 0.108 m 这个结果。

## 验证范围

- ✅ 10 项单元测试通过（纯 Python，无 ROS）
- ✅ Foxy 下 `colcon build` + `ros2 launch` 实跑，最大横向误差 0.108 m
- ⚠️ 仿真是运动学积分，无侧滑/延迟；未在 Humble 复测

## 大模型使用声明

本模块的代码骨架、注释与文档由 **DeepSeek** 辅助整理；作者已在 Ubuntu 20.04 + ROS 2 Foxy
上实际构建、运行并截图，对提交内容负责。
