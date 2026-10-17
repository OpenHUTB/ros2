# 作业4：R2D2小车键盘手动遥控仿真

## 一、实验简介
本实验基于Pybullet物理仿真库，搭建R2D2小车仿真环境，实现键盘手动遥控小车运动：WASD/方向键控制小车前进、后退、左右转向，小车实时响应键盘输入。

## 二、环境依赖
Ubuntu系统，Python3，Pybullet仿真库

安装命令：
```bash
pip install pybullet
```

## 三、文件说明

- main.py：键盘遥控小车控制源码
- r2d2_keyboard_control.gif：仿真运行录屏动图（位于 docs/ground/）

## 四、运行操作步骤

1. 打开 Ubuntu 终端，切换到代码目录

```bash
cd src/ground/r2d2_keyboard_control
```

2. 执行程序

```bash
python3 main.py
```

3. 回车后弹出 Pybullet 可视化 GUI 窗口，自动加载地面与 R2D2 小车模型。
4. 聚焦仿真窗口，按键盘控制小车：

- W / ↑：前进
- S / ↓：后退
- A / ←：左转
- D / →：右转
- 空格：急停
- Q / Esc：退出程序

5. 使用 Peek 录屏工具，框选仿真窗口录制遥控运动画面，保存为 r2d2_keyboard_control.gif，并放入 docs/ground/ 目录。

## 五、原理简述

利用 pybullet 加载 URDF 机器人模型，开启实时仿真（`setRealTimeSimulation(1)`）；主循环中通过 `getKeyboardEvents()` 读取键盘按键状态，根据按键计算前进/后退速度与左右轮差速转向量，再用 `setJointMotorControl2` 设置四个车轮的目标速度，实现小车实时响应键盘手动遥控。
