# mujoco_car_follow

MuJoCo 双车跟车仿真：蓝色引导车沿 x 方向匀速前进、y 方向正弦摆动走出 S 形轨迹，
红色跟随车通过 PD 控制自动调整位置，与引导车保持 **0.5 m 车心距**跟随。

## 运行环境

- Ubuntu 20.04 及以上
- Python 3.8 及以上
- MuJoCo 2.3.7
- ROS 2 Humble（仅 ROS 2 启动方式需要）

## 安装依赖

```bash
pip install mujoco==2.3.7 numpy
```

ROS 2 方式另外需要 `ament_index_python`（Humble 桌面版默认已带）：

```bash
sudo apt install ros-humble-ament-index-python
```

## 方式一：直接运行（不依赖 ROS）

```bash
cd src/ground/mujoco_car_follow
pip install mujoco==2.3.7
python3 scripts/main.py
```

脚本会自己找到同包目录下的 `scene.xml`，无需设置环境变量。

## 方式二：ROS 2 编译运行

```bash
# 1. 把包放进 ROS 2 工作空间
mkdir -p ~/ros2_ws/src
cp -r src/ground/mujoco_car_follow ~/ros2_ws/src/

# 2. 编译
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select mujoco_car_follow

# 3. 加载环境并启动
source install/setup.bash
ros2 launch mujoco_car_follow main.launch.py
```

`scene.xml` 会安装到 `share/mujoco_car_follow/`，脚本优先通过
`ament_index_python.get_package_share_directory()` 定位它。

## 控制参数

| 参数 | 含义 | 默认值 |
| --- | --- | --- |
| `TARGET` | 目标纵向车心距 | 0.5 m |
| `KP_X` / `KD_X` | 纵向 PD 增益 | 1.2 / 0.6 |
| `KP_Y` / `KD_Y` | 横向 PD 增益 | 1.5 / 0.6 |
| `FMAX` | 单轴输出力限幅 | 1.0 N |
| `DEADBAND` | 误差死区 | 0.02 m |

距离口径：0.5 m 指**两车中心（车心）之间的纵向距离**，车身长 0.30 m，
因此前保险杠间隙约 0.20 m；横向偏差目标为 0。

## 运行输出

终端每帧打印一行，可直接作为测试验证日志：

```
车心距 dx=0.512 dy=0.003 | 引导(1.234,0.111) 跟随(0.722,-0.023)
```

## AI 辅助声明

本模块的代码整理、文档撰写与调试排错在 **豆包（Doubao）** 辅助下完成；
控制方案设计、仿真模型结构由作者完成，作者对全部内容负责。
