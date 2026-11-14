# mujoco_ball_collect

MuJoCo 差速小车自动搜集小球仿真。

## 环境

- Ubuntu 20.04
- Python 3.8
- MuJoCo 2.3.7（pip install mujoco==2.3.7）

## 安装

    pip install mujoco==2.3.7

## 运行

    cd src/ground/mujoco_ball_collect
    python3 main.py

无显示环境（服务器 / SSH）加 HEADLESS 跳过窗口：

    HEADLESS=1 python3 main.py

## 预期现象

1. 蓝色小车从原点出发，自动寻找最近的未收集小球；
2. 差速转向驶近目标球，距离小于 0.35 m 时球被收入车上绿色收集筐；
3. 逐个收完 6 个散布小球（4 红 2 蓝）；
4. 全部收完后小车自动停车，球停在筐内。

## 已知限制

- 只读取车与球的平面坐标做最近目标选择，没有激光/视觉感知；
- 球用自由关节悬浮锁高度，不是真实滚动物理；
- 速度/转向增益为固定参数，不同初始布局下可能绕远或抖动。
