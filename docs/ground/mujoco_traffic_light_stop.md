# MuJoCo 红绿灯启停小车

MuJoCo 写的红绿灯小车，红灯停绿灯行，黄灯慢下来。

## 环境

- Ubuntu 20.04
- Python 3.8
- MuJoCo 2.3.7
- NumPy

## 依赖

```bash
pip install mujoco==2.3.7 numpy
```

## 运行

```bash
cd src/ground/mujoco_traffic_light_stop
python3 main.py
```

## 预期现象

- 红绿灯循环：红灯5s → 黄灯2s → 绿灯5s
- 小车从x=-6位置开始向右行驶
- 到达x=2.0停止线后：
  - 红灯：小车刹车停住
  - 黄灯：小车减速慢行
  - 绿灯：小车继续向右行驶

## 已知限制

- 小车只有前后直线运动，没有转向
- 红绿灯是固定时间循环，没有车流量检测
- 没有相机和传感器，是直接读qpos位置

## 效果

![运行截图](../img/ground/mujoco_traffic_light_stop_running.png)

![动图](../img/ground/mujoco_traffic_light_stop_demo.gif)

## 声明

本项目使用了AI辅助编写代码和文档，作者对全部内容负责。
