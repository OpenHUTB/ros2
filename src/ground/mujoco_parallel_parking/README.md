# mujoco_parallel_parking

MuJoCo 侧方停车仿真。

## 环境

- Ubuntu 20.04
- Python 3.8
- MuJoCo 2.3.7（pip install mujoco==2.3.7）

## 安装

    pip install mujoco==2.3.7

## 运行

    cd src/ground/mujoco_parallel_parking
    python3 main.py

## 预期现象

1. 黄色小车从 x=0 出发，向右开过车位；
2. 开过车位后停住，倒车打方向；
3. 斜着倒进车位，回正方向；
4. 最终停在前后两辆停放车之间，车身平行于路边。

## 已知限制

- 用三个 slide/hinge 关节直接控制位置，不是真实阿克曼转向；
- 轨迹是预设的关键帧插值，没有实时避障；
- 没有相机传感器，直接读 qpos。
