# MuJoCo 失落公园无人机端到端视觉转向

该模块面向林荫步道低空飞行，采用 MuJoCo 3.12.0 构建轻量公园场景和带前视 RGB 相机的四旋翼。当前工程提供确定性仿真、统一运行入口和后续数据采集及 ROS 2 Humble 集成所需的接口边界。

The module targets low-altitude flight along a tree-lined path. MuJoCo 3.12.0 provides a lightweight park scene and a quadrotor with a forward RGB camera. The current implementation includes deterministic simulation, one unified entry point, and interface boundaries for data collection and ROS 2 Humble integration.

## 场景与接口 / Scene and interfaces

- 程序化 MJCF 场景，不依赖大型地图和外部资产。
- 前视相机接口固定为 RGB8、320 x 180。
- 转向指令范围为 `[-1, 1]`，右转为正，左转为负。
- 固定随机种子和相对路径确保运行结果可复现。

- Procedural MJCF scene without large maps or external assets.
- Forward-camera contract fixed to RGB8 at 320 x 180.
- Steering range is `[-1, 1]`, positive for right and negative for left.
- A fixed seed and relative paths keep runs reproducible.

## 运行 / Run

```bash
cd src/air/mujoco_abandoned_park_uav_visual_steering
python -m pip install -r requirements.txt
python main.py --steps 120 --steering 0.0
python -m pytest tests -q
```

成功运行后，入口输出包含状态、随机种子、步数、位置和航向角的 JSON 记录。The successful entry-point output is a JSON record containing status, seed, step count, position, and yaw.


