# MuJoCo小车 双车自动跟车仿真

## 实验简介
本实验使用MuJoCo搭建两辆四轮小车，前车（蓝色）沿直线大幅变速行驶，后车（橙色）通过读取前车位置计算车间距，自动调整自身速度跟随前车，保持约 1.2 m 车距。整个过程中两车不转向、不翻车，仅通过车速调节完成跟车。

## 运行环境
Ubuntu20.04 + Python3 + mujoco

## 运行步骤
```bash
# 1. 安装依赖
pip install mujoco

# 2. 运行仿真（源码位于 src/ground/mujoco_car_following/main.py）
python3 src/ground/mujoco_car_following/main.py
```

运行后弹出 MuJoCo 可视化窗口：前车变速行驶，后车自动加减速跟随，两车间距保持稳定。

![双车自动跟车仿真动图](../img/ground/mujoco_car_following.gif)

## 说明
本作业的代码与文档在编写过程中使用了 AI 大模型辅助，本人已核对并确认提交内容，对提交内容负责。
