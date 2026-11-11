# R2D2小车 固定轮速差速圆周运动示例

## 实验简介
本示例使用PyBullet搭建R2D2四轮小车，通过左右车轮固定速度差实现差速圆周运动：左轮速度高、右轮速度低，小车沿圆弧轨迹持续自动行驶，无需人工干预，无限循环。

> 说明：本示例仅演示「固定轮速差速圆周运动」，不包含轨迹反馈、路径跟踪或避障，不能据此说明能够稳定保持给定巡航轨迹。

## 运行环境（已验证）
- Ubuntu 20.04 / Windows 10
- Python 3.8
- pybullet 3.2.5

PyPI 上的 pybullet 只提供 Linux 轮子，Windows 直接 `pip install pybullet` 会转本地编译（需要 MSVC）。Windows 建议用 conda-forge：

```bash
conda create -n r2d2 -c conda-forge python=3.8 pybullet
conda activate r2d2
```

## 运行步骤（从仓库根目录执行）
```bash
# 1. 安装依赖
pip install -r src/ground/r2d2_cruise/requirements.txt

# 2. 直接运行（推荐）
python3 src/ground/r2d2_cruise/main.py

# 3. 无图形环境自检：跑 5 秒并把轨迹写成 CSV
python3 src/ground/r2d2_cruise/main.py --direct --duration 5 --log run.csv

# 4. 或通过 ROS launch 启动（需 ROS 环境）
#    将 src/ground/r2d2_cruise 放入 catkin 工作空间的 src 后：
#    catkin_make && roslaunch r2d2_cruise main.launch
```

运行后弹出 PyBullet 可视化窗口：车体先落下站稳，随后小车沿圆弧轨迹持续圆周巡航行驶。关闭仿真窗口或按 Ctrl+C 即可退出，程序会自动释放仿真连接。

## 本地自检

```bash
python3 -m pytest tests/test_r2d2_cruise_local.py -v
```

用例覆盖三点：`--direct` 模式能跑出轨迹 CSV；5 秒内确实走出圆弧（折线弧长与解缠转角算出的半径约 0.80 m，稳态约 1.02 m）；车体高度稳定在 0.47 m 左右，不会因初始穿插被弹飞。

## ROS 接口说明
本模块为独立 PyBullet 仿真脚本，不发布/订阅 ROS 话题。`package.xml` 与 `launch/main.launch` 用于满足仓库「模块支持 launch 启动、入口为 main 开头」的约定：roslaunch 仅作为统一启动入口调用 `main.py`，不依赖 ROS 运行时。

## 验收方法
- 运行后弹出 PyBullet 可视化窗口，车体落下站稳后自动沿圆弧轨迹持续巡航，无需按键或人工干预；
- `--direct --duration 5 --log run.csv` 能正常结束并写出 10 Hz 轨迹，起步阶段按折线弧长算的转弯半径约 0.80 m（稳态 15–20 s 约 1.02 m）；
- `pytest tests/test_r2d2_cruise_local.py` 三个用例全部通过；
- 关闭窗口或按 Ctrl+C：程序正常退出，无残留进程，仿真连接已释放；
- 运行效果参考动图：

![自动巡航仿真动图](../img/ground/r2d2_cruise.gif)

- 可视化窗口运行截图（相机跟随车体）：

![可视化窗口运行截图](../img/ground/r2d2_cruise_run.png)

## 说明
本作业的代码与文档在编写过程中使用了 AI 大模型辅助，本人已核对并确认提交内容，对提交内容负责。
