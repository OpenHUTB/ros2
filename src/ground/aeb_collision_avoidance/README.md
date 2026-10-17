# AEB Collision Avoidance

基于**所需制动距离**与**碰撞时间（TTC）**的无人车前向碰撞预警与自动紧急制动功能包。
本模块对应无人车方向新增的主动安全能力：接近前方障碍物时分级预警，风险超过阈值后自动施加最大制动力并锁存，直到车辆停车或风险解除。

## 与已有模块的区别

| 已有模块 | 已有能力 | 本模块新增内容 |
| --- | --- | --- |
| `carla_keyboard_control` | CARLA 物理仿真与键盘运动控制 | 基于障碍物距离和自车速度的自动风险判断与制动决策 |
| `vehicle_pygame_teleop` | 键盘遥操作与二维运动模拟 | 不依赖键盘，自动计算所需制动距离与 TTC，输出预警状态和制动命令 |
| 停车模拟/路径规划类模块 | 特定场景的规划或停车流程 | 与规划任务解耦的前向安全层，可作为规划器与底盘控制器之前的安全保护 |

## 决策要点

- 主判据为所需制动距离 `d_req = v^2 / (2 * a_max) + v * t_response + d_stop`
- 补充判据为碰撞时间 `TTC = d / v`
- 紧急制动触发后**锁存**，避免减速导致 TTC 回升而出现制动命令抖动
- 进入停车余量后保持最大制动，防止车辆继续向障碍物蠕动

## 功能

- `SAFE`、`WARNING`、`BRAKING`、`STOPPED` 四级状态机
- 预警阈值、制动阈值、最大减速度、响应时间、停车余量均可配置
- 自带独立二维纵向仿真，不要求先安装 CARLA 或 ROS
- 生成 `aeb_simulation.png` 与 `aeb_simulation.csv`，便于提交运行效果与复现实验
- 可选 ROS 2 节点：订阅 `/aeb/obstacle_distance`、`/aeb/ego_speed`，发布 `/aeb/brake_command` 和 `/aeb/state`

## 环境

- Python **3.8+**（已在 Ubuntu 20.04 + Python 3.8.10、Windows + Python 3.11 验证）
- 独立演示：`matplotlib`（Ubuntu 20.04 自带 `python3-matplotlib` 即可）
- ROS 2 模式：ROS 2 Humble（本仓库的课程虚拟机已内置）、`rclpy`、`std_msgs`
- CARLA/UE 仿真对接：把障碍物距离与自车速度转换后发布到上述 ROS 话题即可；本模块不重复 CARLA 通用网络配置，请参考仓库已有的 CARLA 连接文档

## 独立运行

```bash
cd ros2/src/ground/aeb_collision_avoidance
python3 main.py --output-dir results
```

Windows：

```bat
main.bat --output-dir results
```

Linux：

```bash
bash main.sh --output-dir results
```

运行结束后终端输出是否停车、是否碰撞、最终距离与最终速度，以及结果文件路径。默认场景从 32 m、8 m/s 开始，AEB 会在障碍物前停车。

## ROS 2 运行

已编译工作空间后，二选一：

```bash
# 方式一：课程约定的 main 入口
python3 src/ground/aeb_collision_avoidance/main.py --ros

# 方式二：ROS 2 launch
ros2 launch aeb_collision_avoidance main.launch.py

# 方式三：直接运行节点
ros2 run aeb_collision_avoidance aeb_node
```

输入话题：

| 话题 | 类型 | 含义 |
| --- | --- | --- |
| `/aeb/obstacle_distance` | `std_msgs/Float32` | 自车到前方障碍物的距离（m） |
| `/aeb/ego_speed` | `std_msgs/Float32` | 自车速度（m/s） |

输出话题：

| 话题 | 类型 | 含义 |
| --- | --- | --- |
| `/aeb/brake_command` | `std_msgs/Float32MultiArray` | `[brake, target_speed]` |
| `/aeb/state` | `std_msgs/String` | 当前安全状态 |

用 `ros2 topic pub` 灌入数据即可观察状态变化，例如距离 8 m、车速 10 m/s 时应进入 `braking`。

## 测试

测试不依赖 ROS 或 CARLA：

```bash
python3 -m pytest -q test
```

## 目录

```text
aeb_collision_avoidance/
├── main.py                         # 课程约定主入口（standalone / --ros）
├── main.sh / main.bat              # Linux / Windows 入口
├── aeb_collision_avoidance/
│   ├── aeb_core.py                 # 所需制动距离 + TTC 决策与锁存
│   ├── aeb_node.py                 # ROS 2 节点（console_script: aeb_node）
│   └── simulator.py                # 独立纵向仿真与结果输出
├── launch/main.launch.py           # ROS 2 launch
├── config/aeb_params.yaml          # 阈值配置
├── test/test_aeb.py                # 离线单元测试
├── setup.py / setup.cfg / package.xml
└── resource/aeb_collision_avoidance
```

## 大模型声明

本模块开发过程中使用了 DeepSeek 进行代码和文档辅助生成；提交者已检查、测试并对提交内容负全部责任。
