# R2D2小车 前进后退循环仿真

## 实验简介
本实验使用PyBullet搭建R2D2四轮小车，小车自动循环：前进3秒 → 后退3秒，无限往复。

## 运行环境
Ubuntu20.04 + Python3 + pybullet

## 运行步骤
```bash
# 1. 安装依赖
pip install pybullet

# 2. 运行仿真（源码位于 src/homework/huangshengru/main.py）
python3 src/homework/huangshengru/main.py
```

运行后弹出 PyBullet 可视化窗口：程序启动开始计时，0~3 秒小车向前匀速行驶；3 秒时刻车轮反转，3~6 秒小车向后退回；6 秒完成一次运动周期，随后自动循环前进、后退。

![小车往复运动仿真动图](sim.gif)

## 说明
本作业的代码与文档在编写过程中使用了 AI 大模型辅助，本人已核对并确认提交内容，对提交内容负责。
