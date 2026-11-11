# MuJoCo小车滑斜坡仿真项目

小车从20度斜坡顶端自由滑下，到达平地后自动刹车停住。

## 运行环境
- Ubuntu 20.04
- MuJoCo 2.3.7
- Python 3.8

## 安装依赖
```bash
pip install mujoco==2.3.7
```

## 直接运行（纯MuJoCo，无需ROS编译）
```bash
cd src/ground/mujoco_slope_car
python3 main.py
```

## 停止仿真
关闭MuJoCo viewer窗口，或在终端按 `Ctrl+C` 终止进程。

## 运行效果
![演示动图](../../docs/img/ground/mujoco_slope_car_demo.gif)

本项目由AI辅助生成，作者对全部内容负责。
