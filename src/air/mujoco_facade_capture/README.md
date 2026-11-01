# MuJoCo 无人机幕墙图像采集

模块在三平移自由度姿态稳定飞行器上执行定距蛇形扫描，使用九束测距传感器保持墙面距离，并在航点稳定后获取前视 RGB 图像。立面由简单几何体构建，提供网格、砖色和素色三种样式。

## 运行环境

- Python 3.10 或 3.11、MuJoCo 3.12.0、NumPy、Pillow。
- Ubuntu 22.04 / ROS 2 Humble 用于 `launch` 启动；Windows 可直接运行 Python 入口。
- 相机渲染需要 OpenGL。Ubuntu 无显示环境使用 `MUJOCO_GL=osmesa`，并安装 `libosmesa6`。

在本模块目录执行：

```bash
python3 -m pip install -r requirements.txt
python3 main.py --output output/grid --style grid
python3 main.py --output output/brick --style brick --offset 0.5
python3 train.py --epochs 60 --output output/training
python3 main.py --controller neural --output output/neural --style plain --offset 0.5
```

默认完成 12 个扫描航点。结果目录包含 `trajectory.csv`、`metrics.json`、12 张前视照片及 `overview.png`。`train.py` 重新生成训练数据、训练网络并保存评估。训练输入包括九束测距、航点偏差和当前速度；输出为三个方向的归一化速度指令。内置权重文件可由 `--model` 替换。`--no-images` 用于仅计算物理轨迹；`--duration` 控制最大仿真时间，单位为秒。

## ROS 2 工作区

将本模块放入 ROS 2 工作区的 `src` 目录，在工作区根目录构建并启动：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select mujoco_facade_capture
source install/setup.bash
MUJOCO_GL=osmesa ros2 launch mujoco_facade_capture main.launch.py \
  style:=plain controller:=neural offset:=0.5 output:=/tmp/facade_neural
```

节点以 100 Hz 配置控制周期，发布 `/facade/image` RGB 图像、`/facade/range` 九束测距和 `/facade/cmd_vel` 线速度指令。向 `/facade/cmd_vel_override` 发布 `geometry_msgs/msg/Twist` 可短时覆盖自主指令；该指令经过速度限幅后进入力驱动内环，0.5 秒未收到覆盖消息后恢复自主扫描。任务结束后发布零速度并保留 ROS 节点。

```bash
ros2 topic echo /facade/range
ros2 topic info -v /facade/image
ros2 topic hz /facade/image
ros2 topic pub -r 10 --times 10 /facade/cmd_vel_override geometry_msgs/msg/Twist \
  '{linear: {x: 0.7, y: 0.0, z: 0.0}}'
```

`duration:=70` 设置最大仿真时长，`style:=grid|brick|plain` 选择立面外观。无显示容器需设置 `MUJOCO_GL=osmesa`。

## 模型范围

墙面固定在世界坐标 `y=0`，相机朝向 `+y`。相机纵向视场角为 50 度，画幅比例为 4:3。扫描间隔由壁距和重叠率计算。飞行器的三个平移关节通过力驱动；姿态稳定作为模型假设，未模拟姿态角变化、桨叶气动和真实建筑表面反射。

## 验证

```bash
python3 -m pip install pytest
python3 -m pytest test -q
```

测试覆盖航线参数边界、蛇形遍历、测距投影、力驱动运动、场景级数据划分和权重读写；在 ROS 2 环境中还检查标准消息结构、节点话题声明及外部 `Twist` 指令对物理位移的影响。三场景指标与动图可通过 `python3 evaluate_scenarios.py` 计算。算法与完整操作步骤见[教程](../../../docs/air/mujoco_facade_capture/index.md)。
