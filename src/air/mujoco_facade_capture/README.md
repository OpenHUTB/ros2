# MuJoCo 无人机幕墙图像采集

模块在三平移自由度姿态稳定飞行器上执行定距蛇形扫描，使用九束测距传感器保持墙面距离，并在航点稳定后获取前视 RGB 图像。立面由简单几何体构建，提供网格、砖色和素色三种样式。

## 运行环境

- Python 3.10 或 3.11、MuJoCo 3.12.0、NumPy、Pillow。
- Ubuntu 22.04 / ROS 2 Humble 用于 `launch` 启动；Windows 可直接运行 Python 入口。
- 相机渲染需要 OpenGL。Linux 无显示环境可设置 `MUJOCO_GL=egl`，并安装 Mesa EGL 库。

在本模块目录执行：

```bash
python3 -m pip install -r requirements.txt
python3 main.py --output output/grid --style grid
python3 main.py --output output/brick --style brick --offset 0.5
python3 train.py --output output/training
python3 main.py --controller neural --output output/neural --style plain --offset 0.5
ros2 launch ./launch/main.launch.py output:=/tmp/facade_scan style:=grid
ros2 launch ./launch/main.launch.py output:=/tmp/facade_neural style:=plain controller:=neural
```

默认完成 12 个扫描航点。结果目录包含 `trajectory.csv`、`metrics.json`、12 张前视照片及 `overview.png`。`train.py` 重新生成训练数据、训练网络并保存评估。训练输入包括九束测距、航点偏差和当前速度；输出为三个方向的归一化速度指令。内置权重文件可由 `--model` 替换。`--no-images` 用于仅计算物理轨迹；`--duration` 控制最大仿真时间，单位为秒。

## 模型范围

墙面固定在世界坐标 `y=0`，相机朝向 `+y`。相机纵向视场角为 50 度，画幅比例为 4:3。扫描间隔由壁距和重叠率计算。飞行器的三个平移关节通过力驱动；姿态稳定作为模型假设，未模拟姿态角变化、桨叶气动和真实建筑表面反射。

## 验证

```bash
python3 -m pip install pytest
python3 -m pytest test -q
```

测试覆盖航线参数边界、蛇形遍历、测距投影、力驱动运动、场景级数据划分和权重读写。算法与完整操作步骤见[教程](../../../docs/air/mujoco_facade_capture/index.md)。
