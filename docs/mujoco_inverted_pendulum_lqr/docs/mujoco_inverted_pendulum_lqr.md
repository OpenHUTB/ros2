\# MuJoCo 小车倒立摆 LQR 控制项目文档

\## 1 项目概述

本项目基于MuJoCo物理仿真器，实现小车倒立摆的LQR线性二次调节器稳定控制。

采用质点模型简化摆杆，LQR控制器计算最优控制力，维持摆杆直立，小车保持平衡。



\## 2 系统建模说明

XML物理模型参数：

\- 小车质量 M = 1.0 kg

\- 摆杆总长 L = 0.3 m，质点位于杆中点，等效摆长 l = 0.15 m

\- 摆杆质量 m = 0.5 kg



> 说明：本项目将摆杆简化为质点模型，质点位置在几何中心，\*\*参数在数值上与XML保持一致，但数学模型存在近似\*\*。

> 若采用均匀细杆刚体模型，等效摆长为2L/3=0.2m，同时需要引入摆杆转动惯量；本项目选择质点简化方案。

> 本项目在LQR线性化推导中忽略了关节阻尼以简化计算。物理仿真中引入微小阻尼以提升稳定性，属于工程上的近似假设。



\## 3 环境依赖

\- Python 3.8.10

\- mujoco == 3.2.3

\- numpy == 1.24.3

\- scipy == 1.10.1

\- ROS Noetic



\## 4 文件结构

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



\## 5 运行方法

```bash

\# 方式一：项目根目录直接运行Python脚本

python3 src/mujoco\_inverted\_pendulum\_lqr/main.py



\# 方式二：通过ROS launch启动

cd \~/ground\_ws

catkin\_make

source devel/setup.bash

roslaunch mujoco\_inverted\_pendulum\_lqr balance\_lqr.launch

```



\## 6 运行效果



仿真启动后，小车倒立摆在受到扰动时，LQR 控制器输出控制力，快速将摆杆回正，维持直立平衡。

![两轮自平衡倒立摆仿真效果](https://raw.githubusercontent.com/yi-lu-ping-an/ros2/update-readme/mujoco_inverted_pendulum_lqr/docs/img/balance_demo.gif)


\## 7 常见问题

1\. XML 模型文件找不到：CMakeLists 已配置，将 balance.xml 安装至与 main.py 同级目录，脚本使用相对路径读取模型文件，直接运行 python 脚本或 roslaunch 均可正常加载。

2\. 小车震荡飞出：代码对控制输入做限幅 \[-10, 10]；可调整 LQR 的 Q、R 权重矩阵，改变控制器响应速度。

3\. Python 依赖缺失：使用`pip install -r requirements.txt`一键安装锁定版本依赖。



\## 8 声明

辅助大模型：豆包，deep seek

