```
# 作业1：R2D2小车往复运动仿真
## 一、实验简介
本实验基于Pybullet物理仿真库，搭建R2D2小车仿真环境，编写控制代码实现小车周期性前进、后退往复运动，观察机器人运动效果。

## 二、环境依赖
Ubuntu系统，Python3，Pybullet仿真库
安装命令：
```bash
pip install pybullet
```

## 三、文件说明

- car_move.py：小车运动控制源码
- car_demo.gif：仿真运行录屏动图

## 四、运行操作步骤

1. 打开 Ubuntu 终端，切换到代码目录

```
cd ~/homework1_car
```

2. 执行程序

```
python3 car_move.py
```

3. 回车后弹出 Pybullet 可视化 GUI 窗口，自动加载地面与 R2D2 小车模型。
4. 运动现象：程序启动开始计时，0~3 秒小车向前匀速行驶；3 秒时刻车轮反转，3~6 秒小车向后退回；6 秒完成一次运动周期，自动循环前进后退。
5. 结束仿真：

- 方式 1：关闭 Pybullet 图形窗口
- 方式 2：终端按下 `Ctrl + C` 终止程序

6. 使用 Peek 录屏工具，框选仿真窗口录制运动画面，保存为 car_demo.gif
7. ## 五、原理简述

利用 pybullet 加载 URDF 机器人模型，通过`setJointMotorControl2`控制车轮关节速度；使用取余运算实现周期计时，分段设置车轮正 / 反向速度，实现往复运动，`stepSimulation()`持续推进物理仿真。
