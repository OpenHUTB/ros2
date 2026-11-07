# MuJoCo小车 打靶仿真

## 实验简介
本实验使用MuJoCo搭建带旋转炮塔的四轮小车，固定在场地上自动打靶：前方扇形布置 6 个彩色靶（3 近 3 远），炮塔像雷达一样自动识别还立着的靶子，绿色激光瞄准并锁定（靶子上方亮起白色准星），根据距离自动控制发射力度，逐个击倒；偶尔打偏会自动加力补枪，全部击倒自动结束，无需人工干预。

## 运行环境
Ubuntu20.04 + Python3 + mujoco

## 运行步骤
```bash
# 1. 安装依赖
pip install mujoco

# 2. 运行仿真（源码位于 src/ground/mujoco_shooting/main.py）
python3 src/ground/mujoco_shooting/main.py
```

运行后弹出 MuJoCo 可视化窗口：绿色激光开场扫描，随后炮塔自动锁定靶子发射小球，靶子逐个倒下，全部击倒后自动停止。

![打靶仿真动图](../img/ground/mujoco_shooting.gif)

## 说明
本作业的代码与文档在编写过程中使用了 AI 大模型辅助，本人已核对并确认提交内容，对提交内容负责。
