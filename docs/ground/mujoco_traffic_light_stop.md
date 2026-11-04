# mujoco_traffic_light_stop

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

## 实现思路

### 场景搭建
场景在scene.xml中定义：
- 地面：12m×6m平面，摩擦系数1.2
- 红绿灯杆：位于x=3m处，垂直排列红黄绿三个球形指示灯
- 小车：位于x=-6m处，单自由度沿x轴滑动，带阻尼0.8，质量1.2kg
- 执行器：1个直线电机驱动小车，gear=1.0

### 控制逻辑
1. 信号灯按固定周期循环：红灯5秒→黄灯2秒→绿灯5秒，总周期12秒，用MuJoCo内部仿真时间data.time取模计算当前灯色
2. 距离停止线1.5米开始判断灯色：
   - 绿灯：施加0.6N正推力正常行驶
   - 黄灯：施加-0.1N反向推力减速慢行
   - 红灯：施加-2.0N大力刹车，直到小车速度接近0后锁死
3. 同步修改三个指示灯的rgba颜色，未点亮的灯调暗到0.25倍亮度，直观显示当前灯色

## 运行

```bash
cd src/ground/mujoco_traffic_light_stop
python3 main.py
```

## 预期现象

- 红绿灯循环：红灯5s → 黄灯2s → 绿灯5s
- 小车从x=-6位置开始向右行驶
- 到达x=2.0停止线前：
  - 红灯：小车稳稳刹车停住，不会冲过停止线
  - 黄灯：小车减速慢行，不会加速
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
