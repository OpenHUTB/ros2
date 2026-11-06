# MuJoCo 红绿灯启停小车仿真

## 项目简介
基于 MuJoCo 物理引擎的红绿灯响应小车仿真项目。小车沿直线行驶，到达停止线后根据信号灯状态做出停车、减速或继续行驶的决策。

## 环境依赖
- Python 3.8+
- MuJoCo
- NumPy

## 安装依赖
```bash
pip install mujoco numpy
```

## 运行方式
```bash
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

## 功能说明
- 红绿灯自动循环：红灯5s → 黄灯2s → 绿灯5s
- 小车从左侧起点向右行驶
- 到达 x=2.0 停止线后：
  - 红灯：刹车停车
  - 黄灯：减速慢行
  - 绿灯：正常前进
- 终端实时打印运行日志（时间、灯状态、位置、速度、推力）

## 文件结构
```
mujoco_traffic_light_stop/
├── main.py      # 主程序，控制逻辑+日志输出
├── scene.xml    # MuJoCo 场景模型
└── README.md    # 说明文档
```

## 参数调整
| 参数 | 位置 | 说明 |
|------|------|------|
| T_RED | main.py | 红灯时长（秒） |
| T_YELLOW | main.py | 黄灯时长（秒） |
| T_GREEN | main.py | 绿灯时长（秒） |
| stop_line | main.py | 停止线x坐标 |
| normal_force | main.py | 正常行驶推力 |
| brake_force | main.py | 刹车推力 |
| slow_force | main.py | 黄灯减速推力 |
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
