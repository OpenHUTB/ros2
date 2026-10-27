# AirSim Blocks 四旋翼无人机键盘遥控（uav_airsim_keyboard）

基于 ROS2 Humble 与 AirSim（Blocks 环境）的无人机键盘遥控模块：
在 AirSim 仿真中通过键盘手动控制四旋翼飞行，并实时发布位姿话题 `/uav/pose`。

## 环境要求

- Ubuntu 22.04 + ROS2 Humble
- Windows 宿主机运行 AirSim Blocks 环境（API 端口 8080，虚拟机通过 `10.0.2.2:8080` 连接）
- Python 依赖：

```bash
pip3 install airsim pynput -i https://pypi.tuna.tsinghua.edu.cn/simple
```

## 目录结构

```
src/uav_airsim_keyboard/
├── package.xml                 # ROS2 包描述（正规 ament_python 包）
├── setup.py / setup.cfg        # 安装配置
├── resource/                   # ament 索引资源
├── uav_airsim_keyboard/
│   └── main.py                 # ROS2 节点：AirSim 连接+键盘控制+位姿发布
├── launch/
│   └── uav_airsim_keyboard.launch.py   # launch 启动文件
└── README.md
```

## 构建

```bash
cd ~/ros2
colcon build --packages-select uav_airsim_keyboard
source install/setup.bash
```

## 运行步骤

1. Windows 上启动 AirSim Blocks：
   `D:\airsim\Blocks\WindowsNoEditor\Blocks.exe`
2. 虚拟机中启动节点：

```bash
ros2 launch uav_airsim_keyboard uav_airsim_keyboard.launch.py
```

3. 按 **T** 起飞，之后：

| 按键 | 动作 |
| --- | --- |
| W / S | 前 / 后 |
| A / D | 左 / 右 |
| R / F | 上升 / 下降 |
| Q / E | 左转 / 右转 |
| L | 降落 |
| Ctrl+C | 退出（自动降落） |

## 话题

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/uav/pose` | `geometry_msgs/PoseStamped` | 无人机位姿（坐标为 AirSim NED 系，x 北 / y 东 / z 下） |

查看位姿：

```bash
ros2 topic echo /uav/pose
```

## 运行效果

![AirSim 键盘遥控运行效果](../../img/uav_airsim_keyboard_demo.gif)
