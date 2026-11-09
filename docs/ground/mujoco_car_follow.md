# mujoco_car_follow

MuJoCo小车跟车仿真项目：引导车自动走 S 形曲线，跟随车通过 PD 控制自动调整位置，和引导车保持 0.5 米车心距跟随。

## 运行环境
- Ubuntu 20.04
- MuJoCo 2.3.7
- Python 3.8

## 运行步骤
```bash
pip install mujoco==2.3.7
python3 main.py
```

ROS 2 环境：
```bash
colcon build --packages-select mujoco_car_follow
source install/setup.bash
ros2 launch mujoco_car_follow main.launch.py
```

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
