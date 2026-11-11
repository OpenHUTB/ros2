# MuJoCo小车滑斜坡仿真

小车从20度斜坡顶端自由滑下，到达平地后自动刹车停住。

## 运行环境
- Ubuntu 20.04
- MuJoCo 2.3.7
- Python 3.8

## 运行步骤
```bash
pip install mujoco==2.3.7
cd src/ground/mujoco_slope_car
python3 main.py
```

## 运行效果
![小车滑斜坡演示](../img/ground/mujoco_slope_car_demo.gif)

## 说明
- 刹车开始位置：x=0.5m（车心坐标）
- 实测停车位置约x=0.59m，误差约9cm
- 刹车逻辑：到达停车线后，每一步将x方向速度乘以0.9做速度衰减，直到速度趋近0

本项目由AI辅助生成，作者对全部内容负责。
