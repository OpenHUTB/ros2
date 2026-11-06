# passive_walker —— 无动力被动行走（轮辐式 Rimless Wheel）

基于 **MuJoCo** 的最小被动行走模型：12 根辐条的刚性轮，**没有任何执行器**（`nu = 0`），
只靠初始轻推 + 5° 斜坡的重力做功 + 落地碰撞耗能，持续稳定地向下坡滚动。
模块不依赖 ROS 也能跑物理仿真，ROS 2 节点只负责把累计步数发布到话题。

## 依赖

- Ubuntu 20.04.6 LTS + Python 3.8.10
- ROS 2 Humble（Humble 官方支持 22.04，20.04 下为**源码编译安装**）
- `mujoco` 没有 rosdep 键，必须 pip 安装，版本见 `requirements.txt`

```bash
pip3 install -r requirements.txt
```

## 编译

```bash
cd ~/ros2_ws
colcon build --packages-select passive_walker
source install/setup.bash
```

## 运行

无 ROS（只验证物理）：

```bash
python3 main.py --seconds 20 --push 0.3
python3 main.py doctor          # 检查 mujoco / 模型文件 / rclpy
```

ROS 2 节点（发布 `/passive_walker/steps`，`std_msgs/Int32`）：

```bash
ros2 run passive_walker main.py node --seconds 20
ros2 topic echo /passive_walker/steps
```

或使用 launch：

```bash
ros2 launch passive_walker main.launch.py seconds:=20 push:=0.3
```

节点支持 `--ros-args`，可以重映射话题或覆盖参数：

```bash
ros2 run passive_walker main.py node --seconds 20 --ros-args -r /passive_walker/steps:=/steps
```

## 测试

```bash
colcon test --packages-select passive_walker
```

没装 `mujoco` 时测试会自动跳过（`pytest.importorskip`），不会在干净机器上直接报错。

## 生成演示 GIF

```bash
python3 render.py --seconds 5.5 --fps 7 --width 960 --height 540 --out 01_simulation.gif
```

无显示器的服务器上先设置 `MUJOCO_GL=egl`（Linux）：

```bash
MUJOCO_GL=egl python3 render.py --out 01_simulation.gif
```

## 目录结构

```
src/passive_walker/
├── CMakeLists.txt
├── package.xml
├── requirements.txt
├── README.md
├── main.py              # 统一入口：run / node / doctor
├── render.py            # 生成演示 GIF
├── passive_walker/
│   ├── __init__.py
│   ├── sim.py           # MuJoCo 仿真核心（不依赖 ROS）
│   └── sim_node.py      # ROS 2 节点，发布累计步数
├── assets/passive_walker.xml
├── launch/main.launch.py
└── test/test_sim.py
```
