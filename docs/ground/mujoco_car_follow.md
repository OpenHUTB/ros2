# mujoco_car_follow

MuJoCo小车跟车仿真项目：引导车自动走 S 形曲线，跟随车通过 PD 控制自动调整位置，和引导车保持 0.5 米车心距跟随。

## 运行环境
- Ubuntu 20.04
- MuJoCo 2.3.7
- Python 3.8

## 运行步骤

### 方式一：直接运行（不依赖 ROS）

```bash
# 1. 安装依赖
pip install mujoco==2.3.7 numpy

# 2. 进入包源码目录
cd src/ground/mujoco_car_follow

# 3. 运行（脚本会自动在包目录下查找 scene.xml）
python3 scripts/main.py
```

### 方式二：ROS 2 编译运行

```bash
# 1. 把包拷进工作空间
mkdir -p ~/ros2_ws/src
cp -r src/ground/mujoco_car_follow ~/ros2_ws/src/

# 2. 加载 ROS 2 环境（Humble 为例，其他发行版换成对应路径）
cd ~/ros2_ws
source /opt/ros/humble/setup.bash

# 3. 编译
colcon build --packages-select mujoco_car_follow

# 4. 加载工作空间环境
source install/setup.bash

# 5. 启动
ros2 launch mujoco_car_follow main.launch.py
```

`scene.xml` 安装到 `share/mujoco_car_follow/`，脚本通过
`ament_index_python.get_package_share_directory()` 定位，不需要手动设路径。

## 实现思路
1. 场景搭建：地面平面 + 两个带双滑动关节的小车，蓝色为引导车，红色为跟随车
2. 引导车控制：x 方向匀速前进，y 方向做正弦摆动走出 S 形轨迹
3. 跟随车控制：实时计算与引导车的位置差，采用 **PD 比例控制 + 速度反馈**：距离远了加速靠近，距离近了自动减速，速度反馈抑制超调和抖振，稳定保持 0.5 米车心距
4. 物理仿真：使用 MuJoCo 内置物理引擎计算两车运动和相对位置变化

## 距离口径说明
文中 0.5 米指**两车中心（车心）之间的纵向距离**。小车车身长 0.30 m，因此对应的前保险杠间隙约为 **0.20 m**（0.5 − 0.30）。横向偏差目标为 0（即两车横向对齐）。

## 运行效果
![运行截图](../img/ground/mujoco_car_follow_run.png)

![演示动图](../img/ground/mujoco_car_follow_demo.gif)

## AI 辅助声明

本模块的代码整理、文档撰写与调试排错在 **豆包（Doubao）** 辅助下完成；控制方案设计、
仿真模型结构由作者完成，作者对全部内容负责。
