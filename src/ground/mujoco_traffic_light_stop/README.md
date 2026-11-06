# mujoco_traffic_light_stop

MuJoCo 写的红绿灯小车，红灯停绿灯行，黄灯慢下来。

## 环境

- Ubuntu 20.04
- Python 3.8
- MuJoCo 2.3.7
- NumPy

## 跑起来

```bash
pip install mujoco==2.3.7 numpy
cd src/ground/mujoco_traffic_light_stop
python3 main.py
```

## 预期现象

- 红绿灯循环：红灯8s → 黄灯2s → 绿灯5s
- 小车从x=-6位置开始向右行驶
- 到达停止线前：
  - 红灯：小车刹车停住
  - 黄灯：小车缓慢行驶
  - 绿灯：小车继续向右行驶

## 已知限制

- 小车只有前后直线运动，没有转向
- 红绿灯是固定时间循环，没有车流量检测
- 没有相机和传感器，是直接读qpos位置

## 文件

- `main.py` 控制逻辑
- `scene.xml` 场景模型

## 声明

本项目使用了AI辅助编写代码和文档，作者对全部内容负责。
