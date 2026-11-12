# 多车编队保持仿真（pygame 2D）

纯 Pygame 实现的 2D 平面编队保持仿真，不依赖 ROS，Gazebo 和 CARLA。领航者沿固定圆周轨迹运动，两个跟随者按比例控制保持在领航者坐标系下的固定三角队形。

## 环境

- Windows 11，Python 3.13.16，pygame 2.6.1，SDL 2.28.4
- 复现环境为 Python 3.13.16 加 pygame 2.6.1，代码只用到 pygame 基础 API

```bash
pip install -r src/ground/formation_keeping/requirements.txt
```

## 运行

代码位于 src/ground/formation_keeping 目录，在包目录下执行。命令里的 python 在 Linux 或 macOS 上通常写成 python3，Windows 也可以用 py -3。Windows 终端中文乱码时先执行 chcp 65001。

```bash
cd src/ground/formation_keeping
python main.py
```

Windows 也可用 main.bat，Linux 和 macOS 用 main.sh。

## 分层

- formation.py：负责偏移旋转和期望位置解算，计算队形误差均方根，并维护领航者圆周状态。
- team.py：编队状态机，逐帧推进领航者，并让跟随者按比例控制逼近期望位置。
- main.py：绘制参考圆，期望队形虚点，成员轨迹和朝向，并在左上角显示队形误差与增益。

## 控制律

跟随者每帧朝期望位置方向移动 GAIN 倍的距离差，单帧位移上限为 MAX_STEP。领航者线速度为半径 RADIUS 与角速度 OMEGA 的乘积，即每帧 2 像素，稳态滞后约 3 像素。

## 误差口径

队形误差为各成员与期望位置距离的均方根，即 RMS。参与统计的成员共 3 个，领航者加两个跟随者，分母为成员数 3。领航者自身偏移为 0，其位置偏差也一并计入 RMS。

## 参数

参数默认值集中在 formation_keeping/config.py：

| 参数 | 默认值 | 说明 |
|---|---|---|
| WORLD_W 与 WORLD_H | 800 乘 600 | 画布尺寸，单位像素 |
| FPS | 60 | 渲染帧率 |
| CENTER | 400, 300 | 领航者圆周圆心 |
| RADIUS | 160 像素 | 领航者圆周半径 |
| OMEGA | 0.0125 弧度每帧 | 领航者角速度 |
| FOLLOWER_OFFSETS | -95,-58 与 -95,58 | 跟随者相对领航者朝向的队形偏移 |
| INIT_POSITIONS | 120,520 与 700,480 | 两个跟随者初始位置 |
| GAIN | 0.40 | 跟随者比例控制增益 |
| MAX_STEP | 3.4 像素 | 跟随者单帧位移上限 |
| TOL | 6.0 像素 | 队形误差收敛阈值 |
| SETTLE_STEPS | 260 | 判定 settled 的最少步数 |

## 测试

```bash
cd src/ground/formation_keeping
python -m pytest tests/test_formation_keeping.py
```

7 个用例覆盖这些方面：偏移旋转，不同朝向下的期望位置，领航者圆周约束，队形误差定义，初始偏离后收敛到阈值以内。其中包含 100 步内误差降到初始一半以下的收敛过程断言，并验证收敛后能否长时间保持队形。

## 限制

- 领航者轨迹是固定参数的圆周，不是任意路径。
- 队形是固定的三角偏移，不支持队形切换。
- 没有成员间通信延迟，也没有成员间避碰，不考虑跟随者之间的碰撞约束。
- 初始位置为手工设定，用于演示收敛过程。

## 运行效果

第 60 帧：两个跟随者还在从初始散开位置追赶，队形误差 187.88 像素，灰色小圈是各自的期望位置。

![队形收敛中](../img/ground/formation_keeping_frame_60.png)

第 340 帧：跟随者与期望位置基本重合，三角队形随领航者朝向旋转，队形误差 2.98 像素。为了让稳态队形看得清楚，这一帧的轨迹从队形收敛后重新起算。

![队形稳定](../img/ground/formation_keeping_frame_340.png)

第 520 帧：绕行大半圈后仍保持队形，误差仍是 2.98 像素。三条轨迹圆心相同，半径不同，分别对应领航者与两个跟随者的队形偏移。

![绕行大半圈](../img/ground/formation_keeping_frame_520.png)

## 声明

过程中借助大模型工具辅助代码编写、整理包目录结构与本描述排版，核心逻辑与操作、场景参数设定与实际运行调试为本人完成，作者对提交内容负责。
