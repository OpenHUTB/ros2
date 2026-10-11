# MuJoCo 小车倒立摆 LQR稳定控制
## 一、项目简介
本项目基于 MuJoCo 物理仿真引擎搭建小车倒立摆模型，使用**LQR线性二次调节器**实现倒立摆的直立稳定控制。
小车倒立摆是典型的非线性不稳定系统，在平衡点附近线性化后，通过状态反馈控制，在摆杆受到微小扰动时，控制器驱动小车移动，保持摆杆竖直不倒。
项目包含MuJoCo模型文件、LQR控制Python脚本、项目说明文档，可直接在Ubuntu20.04环境一键运行。

![倒立摆LQR仿真演示](balance_demo.gif)

## 二、环境依赖
- 操作系统：Ubuntu 20.04
- Python版本：Python3.8（Ubuntu20.04自带）
- 核心库：
    - mujoco 3.x：物理仿真引擎
    - numpy：矩阵运算
    - scipy：求解黎卡提方程，计算LQR反馈矩阵

## 三、环境安装
### 1. 安装Python依赖包
```bash
pip3 install mujoco numpy scipy -i https://pypi.tuna.tsinghua.edu.cn/simple
