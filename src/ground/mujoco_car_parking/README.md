# mujoco_car_parking

MuJoCo 四轮小车倒车入库仿真。

## 环境

- Ubuntu 20.04
- Python 3.8
- MuJoCo 2.3.7（pip install mujoco==2.3.7）

## 安装

    pip install mujoco==2.3.7

## 运行

    cd src/ground/mujoco_car_parking
    python3 main.py

## 预期现象

1. 小车从 (1.5, -1.0) 出发直行，x 回到 0.1 以内；
2. 原地掉头到车头朝南（yaw 约 -1.55 rad，即 -90 度）；
3. 直退入库，y >= 0.9 m 自动停车；
4. 停车后四轮速指令归零。

实测入库位姿：x=+0.091 m、y=0.907 m、yaw=-90.1 度，距后墙 0.253 m、横偏 0.091 m，零撞墙，车身最大倾角 1.4 度。

## 已知限制

- 简化为平面运动，freejoint 只控 x/y/yaw，没有真实侧倾/悬架模型；
- 四轮用速度伺服加 forcerange 限力，没有真实差速传动和 Ackermann 转向；
- 无激光雷达/摄像头传感器，靠 qpos 真值做状态机，没有在线避障；
- 三段速度指令为开环，车库尺寸改动后需重新整定相位切换阈值。
