# uav_depth_terrain_passability_nn

深度相机地形可通行性评估仿真：三扇区采样地形高度、台阶与坡度，两层 numpy MLP 评估各扇区通行度并选择可通行走廊，闭环沿中央通道前进。

## 概览

| 项目 | 内容 |
| --- | --- |
| 话题类别 | 任务 2：传感器感知及运动控制（深度地形 + 神经网络通行度评估） |
| 平台 | Ubuntu 20.04 ROS Noetic / Ubuntu 22.04 ROS 2 Humble；离线仿真可在 Windows Python 3.13 运行 |
| 语言 | Python 3 |
| 依赖 | rclpy、nav_msgs、std_msgs（ROS 节点）；离线仅依赖 numpy、pygame |
| 运行方式 | `ros2 launch uav_depth_terrain_passability_nn uav_depth_terrain_passability_nn.launch.py`；离线 `python main.py --shots` |
| 仿真器 | 内置 numpy 平面运动模型 + pygame 离屏/窗口渲染 |

## 原理

- 深度传感器在左中右三个前向扇区采样地面点，统计各扇区平均高度、平均台阶与平均坡度。
- 归一化特征送入两层 MLP（输入 9，隐藏层 8），输出三扇区通行度 softmax 与一路 tanh 速度指令。
- 控制器按选中扇区设定横向目标，航向 PD 转向，前飞速度由速度指令与所选扇区通行度共同标定，扇区切换带滞回。

## 快速运行

### ROS 2 Humble

```bash
colcon build --packages-select uav_depth_terrain_passability_nn --symlink-install
source install/setup.bash
ros2 launch uav_depth_terrain_passability_nn uav_depth_terrain_passability_nn.launch.py
```

### 离线仿真（无需 ROS）

```bash
python main.py --shots    # 离屏运行并保存全过程截图到 docs/img/
python main.py --demo     # 打开观察窗口，查看通行决策过程
```

## 测试

```bash
python -m pytest test/ -q
```

## 文件结构

```
uav_depth_terrain_passability_nn/
├── launch/uav_depth_terrain_passability_nn.launch.py
├── uav_depth_terrain_passability_nn/
│   ├── terrain.py      # 地形高度场、台阶与坡度基准
│   ├── depth_sensor.py # 三扇区地形深度采样与特征归一化
│   ├── policy.py       # 两层 MLP 通行度评估与速度指令网络
│   ├── controller.py   # 扇区选择滞回与速度偏航跟踪控制器
│   └── sim_node.py     # ROS 2 封装节点
├── test/                # 感知、策略、控制器单元测试
├── main.py              # 离线一键实验入口
├── package.xml
└── setup.py
```

## 许可

MIT
