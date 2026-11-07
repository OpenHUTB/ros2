# mujoco_car_follow

MuJoCo小车跟车仿真项目：引导车自动走S形曲线，跟随车自动调整位置，和引导车保持0.5米距离跟随。

## 运行环境
- Ubuntu 20.04
- MuJoCo 2.3.7
- Python 3.8

## 运行步骤
```bash
pip install mujoco==2.3.7
python3 main.py
```

## 实现思路
1. 场景搭建：地面平面 + 两个带双滑动关节的小车，蓝色为引导车，红色为跟随车
2. 引导车控制：x方向匀速前进，y方向做正弦摆动走出S形轨迹
3. 跟随车控制：实时计算和引导车的位置差，距离远了就加速靠近，距离近了就减速，保持0.5米跟车距离
4. 物理仿真：使用MuJoCo内置物理引擎计算两车运动和相对位置变化

## 运行效果
![运行截图](../img/ground/mujoco_car_follow_run.png)

![演示动图](../img/ground/mujoco_car_follow_demo.gif)
