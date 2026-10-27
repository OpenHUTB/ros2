# 基于MuJoCo的自动紧急制动（AEB）TTC仿真
## 1 项目简介
本项目基于MuJoCo物理引擎搭建车辆动力学模型，实现**TTC碰撞时间型自动紧急制动AEB**仿真验证。
AEB核心目标是实时计算碰撞风险，当TTC低于安全阈值时输出比例制动力，对自车进行减速干预，避免与前车发生追尾碰撞。
项目采用Python实现控制逻辑，适配 Ubuntu20.04 + ROS Noetic，可输出自车速度、TTC、制动控制信号时序曲线。

## 2 原理介绍
### 2.1 TTC碰撞时间公式

TTC=\frac{d}{v_{ego}-v_{lead}}

- d：自车与前车纵向距离(m)
- v_{ego}：自车纵向速度(m/s)
- v_{lead}：前车纵向速度(m/s)

> 当相对速度 v_{ego}-v_{lead}\le0，两车距离不会继续缩小，无碰撞风险，TTC置为无穷大。

### 2.2 AEB比例制动控制逻辑
1. 若自车速度接近0，不输出制动；
2. 计算相对速度，无靠近风险则制动力置0；
3. 当 TTC < TTC_{threshold} 触发制动，采用比例制动：

brake = gain \cdot \left(1-\frac{TTC}{TTC_{threshold}}\right)


4. 制动输出限幅 brake\in[0,1]，映射为广义制动力施加至车身；
5. 制动降低自车速度，抬高TTC，抑制碰撞风险。

### 2.3 MuJoCo模型说明

车辆包含自车与前车两个刚体，使用自由关节实现纵向运动；通过`qfrc_applied`施加纵向制动力，viewer可视化整车运动场景。

## 3 环境依赖
Ubuntu 20.04 + ROS Noetic
依赖版本：
- mujoco==2.3.7
- numpy==1.21.6
- matplotlib==3.4.3

依赖安装：
```bash
pip3 install -r requirements.txt
```
## 4 编译与运行
```
# catkin工作空间编译
catkin_make
source devel/setup.bash

# 方式1：脚本启动
bash src/ground/mujoco_vehicle_aeb/scripts/main.sh

# 方式2：rosrun启动
rosrun mujoco_vehicle_aeb main.py

# 方式3：直接python调试运行
python3 scripts/main.py
```
## 5 运行效果

仿真现象：

- 仿真初期自车匀速向前行驶，前车保持静止，TTC 数值持续下降；
- TTC 下降至设定安全阈值后 AEB 系统触发，输出平滑增大的制动力，自车开始减速；
- 制动介入后 TTC 被抬升，有效规避追尾事故。

## 6 实验结果
程序运行结束自动生成 `result.png`，包含 3 组时序曲线：
1. 自车行驶速度
2. TTC 碰撞时间以及安全阈值虚线
3. AEB 制动输出信号（0~1）

## 7 文件结构

```
docs/
└── ground/
    └── mujoco_vehicle_aeb.md
└── img/
    └── aeb_result.png
    └── aeb_demo.gif
src/
└── ground/
    └── mujoco_vehicle_aeb/
        ├── models/
        │   └── aeb_car.xml
        ├── scripts/
        │   ├── main.py
        │   ├── aeb_controller.py
        │   └── main.sh
        ├── CMakeLists.txt
        ├── package.xml
        └── requirements.txt
```

## 8 大模型声明
	deepseek,豆包