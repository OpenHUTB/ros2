# uav_downview_centerline_track_nn

下视摄像头神经网络标线循迹仿真：机载下视图像提取横向偏移与航向误差，两层 numpy MLP 策略输出前飞与偏航指令，闭环跟踪地面 S 形参考标线。

## 概览

| 项目 | 内容 |
| --- | --- |
| 话题类别 | 任务 2：传感器感知及运动控制（下视视觉 + 神经网络策略） |
| 平台 | Ubuntu 20.04 ROS Noetic / Ubuntu 22.04 ROS 2 Humble；离线仿真可在 Windows Python 3.13 运行 |
| 语言 | Python 3 |
| 依赖 | rclpy、nav_msgs、std_msgs（ROS 节点）；离线仅依赖 numpy、pygame |
| 运行方式 | `ros2 launch uav_downview_centerline_track_nn uav_downview_centerline_track_nn.launch.py`；离线 `python main.py --shots` |
| 仿真器 | 内置 numpy 刚体平面运动模型 + pygame 离屏/窗口渲染 |

## 原理

- 下视相机把地面标线投影到二值图像网格，拟合标线在机体系的横向偏移与航向误差。
- 归一化特征送入两层 MLP（输入 3，隐藏层 8，输出 2），tanh 激活，输出前飞指令与偏航指令。
- 内环把策略指令积分成速度与偏航更新，并叠加切线航向 PD 保持。

## 快速运行

### ROS 2 Humble

```bash
colcon build --packages-select uav_downview_centerline_track_nn --symlink-install
source install/setup.bash
ros2 launch uav_downview_centerline_track_nn uav_downview_centerline_track_nn.launch.py
```

### 离线仿真（无需 ROS）

```bash
python main.py --shots    # 离屏运行并保存全过程截图到 docs/img/
python main.py --demo     # 打开观察窗口，查看下视循迹过程
```

## 测试

```bash
python -m pytest test/ -q
```

## 文件结构

```
uav_downview_centerline_track_nn/
├── launch/uav_downview_centerline_track_nn.launch.py
├── uav_downview_centerline_track_nn/
│   ├── track.py        # 地面参考标线曲线
│   ├── perception.py   # 下视相机投影与标线特征提取
│   ├── policy.py       # 两层 MLP 循迹策略网络
│   ├── controller.py    # 速度与偏航跟踪控制器
│   └── track_node.py   # ROS 2 封装节点
├── test/                # 感知、策略、控制器单元测试
├── main.py              # 离线一键实验入口
├── package.xml
└── setup.py
```

## 许可

MIT
