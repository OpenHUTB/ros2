# MuJoCo 两轮自平衡机器人（倒立摆）LQR控制仿真
## 1.项目简介
本项目基于MuJoCo物理引擎搭建小车倒立摆动力学模型，使用LQR线性二次调节器实现摆杆直立稳定控制。
仿真给摆杆施加初始角度扰动，控制器实时计算输出力，维持倒立摆竖直平衡，可直观观察LQR控制效果。
![两轮自平衡倒立摆仿真效果](https://raw.githubusercontent.com/yi-lu-ping-an/ros2/update-readme/mujoco_inverted_pendulum_lqr/docs/img/balance_demo.gif)

## 2.环境依赖
1. Ubuntu 20.04
2. ROS Noetic
3. MuJoCo
4. Python3
5. numpy、scipy

## 3.安装步骤
1. 安装Python依赖
```bash
pip3 install -r requirements.txt
2. 编译 ROS 工作空间

```
cd ~/ground_ws
catkin_make
source devel/setup.bash
```
## 4. 文件结构

```
mujoco_inverted_pendulum_lqr/
├── src/
│   └── mujoco_inverted_pendulum_lqr/
│       ├── main.py
│       ├── balance.xml
│       ├── package.xml
│       ├── CMakeLists.txt
│       └── launch/
│           └── balance.launch
├── docs/
│   ├── img/
│   │   └── balance_demo.gif
│   └── mujoco_inverted_pendulum_lqr.md
├── README.md
└── requirements.txt

```
## 5. 运行方式

### 方式 1：项目根目录直接 Python 运行
```bash
python3 src/mujoco_inverted_pendulum_lqr/main.py

### 方式 2：ROS launch 启动

```
cd ~/ground_ws
catkin_make
source devel/setup.bash
roslaunch mujoco_inverted_pendulum_lqr balance_lqr.launch
```
## 6. 控制原理

1. 对小车倒立摆非线性动力学模型在平衡点附近线性化，得到状态空间方程
2. 设计 LQR 代价函数，求解状态反馈增益矩阵 K
3. 实时读取 MuJoCo 仿真状态，计算控制量施加给小车，抵消摆杆倾倒趋势

## 7. 仿真说明

- 摆杆总长度 0.3m，质心距离转轴 0.15m
- 仿真添加 0.2rad 初始倾斜扰动，用来触发控制器
- 代码加入 time.sleep 限制仿真速度，方便录制 GIF 动画

## 8. 常见问题

1. 找不到 balance.xml：运行 main.py 需要在 src 目录执行
2. mujoco.viewer 无法弹出：检查显卡与虚拟机加速设置
3. scipy 缺失：执行 pip3 install scipy 安装

## 9. 本地文件创建操作备忘
> 本段为项目搭建时的操作记录，非项目运行必要步骤
1. 创建各目录与文件，使用 nano 编辑器编写代码与文档
2. nano保存退出快捷键：`Ctrl+O` →回车保存 →`Ctrl+X` 退出编辑器

#### XML模型优化说明
1. 相机姿态优化：将相机姿态定义从`zaxis`改为`xyaxes`，直接指定相机X、Y轴方向，消除视图倾斜问题，录制GIF画面水平稳定。
2. 碰撞设置优化：摆杆几何体增加`contype="0" conaffinity="0"`，关闭摆杆与小车之间的碰撞检测。防止摆杆大角度时与车体发生突发碰撞冲击，干扰LQR控制器稳定；此设置仅禁用杆车碰撞，小车与地面碰撞不受影响。