# MuJoCo 无人机视觉着陆与同步建图导航

本模块使用 MuJoCo 演示无人机移动平台视觉着陆，并提供键盘遥控和二维激光建图导航。俯视 RGB 图像用于定位平台和机体中心，多层感知机（MLP）输出平面速度与下降指令；导航场景同步采集虚拟激光扫描、更新占据栅格并重规划路径。ROS 2 Humble 节点提供视觉特征输入与速度输出接口。

![MuJoCo 移动平台着陆场景](../img/air/mujoco_uav_moving_pad_landing.png){width=55%}

![MuJoCo 移动平台着陆过程](../img/air/mujoco_uav_moving_pad_landing.gif){width=55%}

## 模块结构

| 文件 | 用途 |
| --- | --- |
| `assets/moving_pad.xml` | 四旋翼位置伺服、移动平台、地面与俯视相机场景 |
| `landing.py` | HSV 颜色定位、控制特征构造、MLP 训练/测试与 MuJoCo 闭环仿真 |
| `main.py` | `train`、`demo`、`teleop` 和 `navigate` 命令行入口 |
| `teleop.py` | 键盘指令到 MuJoCo 速度控制的映射 |
| `navigation.py` | 虚拟激光扫描、占据栅格更新、A* 与同步重规划 |
| `ros_node.py` | ROS 2 速度控制节点 |
| `launch/landing.launch.py` | 启动控制节点 |

## 运行步骤

在模块目录安装依赖并运行：

```bash
python -m pip install -r requirements.txt
python main.py train --samples 5000
python main.py demo --steps 180 --output output
python main.py teleop --output output
python main.py navigate --output output
python -m pytest -q tests
```

键盘遥控提示出现后，每行输入一条指令并按回车：`W/A/S/D` 控制水平移动，`R/F` 升降，空行悬停，`Q` 退出。每次输入会推进 10 个 MuJoCo 步并显示当前位置。

着陆流程：MuJoCo 俯视相机生成 RGB 帧 → HSV 区域提取 → 计算平台与无人机中心偏差 → 合并高度和平台速度 → MLP 预测 `vx`、`vy`、`vz` → 输出 MuJoCo 控制量或 ROS 速度话题。

导航流程：虚拟 2D 激光雷达逐角度测距 → Bresenham 射线遍历标记自由与占据栅格 → A* 以 `f(n) = g(n) + h(n)` 选择路线，其中 `h(n)` 使用曼哈顿距离，未知格增加通行代价 → 无人机沿路径移动一个网格 → 更新仿真位姿并再次扫描建图 → 重新规划至目标。固定障碍场景下，里程计位姿递推、激光建图和航点导航在同一仿真循环中同步执行。

![键盘遥控后的 MuJoCo 无人机位置](../img/air/mujoco_uav_teleop_scene.png){width=55%}

![二维导航栅格地图、障碍物、起点与目标](../img/air/mujoco_uav_navigation_map.png){width=65%}

![激光建图与路径重规划过程](../img/air/mujoco_uav_navigation_run.gif){width=65%}

## Docker 运行

环境基于 ROS 2 Humble、MuJoCo 3.12.0、OpenCV 和 scikit-learn。首次运行构建容器；默认 Compose 演示以 1200 个样本训练控制器后运行 180 步仿真。

```bash
cd src/air/mujoco_uav_moving_pad_landing
docker compose build
docker compose run --rm landing
```

仿真运行后，`output/` 保存真实渲染的 `landing_scene.png`、`landing_run.gif`、`trajectory.json` 和 `metrics.json`。默认演示步数为 180，可用 `--steps` 调整：

```bash
docker compose run --rm landing python3 /workspace/mujoco_uav_moving_pad_landing/main.py demo --steps 300 --output /workspace/output
```

键盘与二维导航场景可在本机 Python 环境直接运行；导航结果保存为 `navigation_map.png`、`navigation_run.gif` 和 `navigation_metrics.json`。

启动 ROS 2 节点容器：

```bash
docker compose run --rm ros_adapter
```

## ROS 2 接口

在 ROS 2 Humble 工作区编译后启动控制节点：

```bash
colcon build --packages-select mujoco_uav_moving_pad_landing
source install/setup.bash
ros2 launch mujoco_uav_moving_pad_landing landing.launch.py
```

节点订阅 `/landing/features`（`std_msgs/Float32MultiArray`），按顺序接收归一化图像 x/y 偏差、高度、平台 x/y 速度及视觉有效标志；节点发布 `/cmd_vel`（`geometry_msgs/Twist`）。视觉无效时速度归零。可通过 `MUJOCO_LANDING_MODEL` 指定控制器文件。

Docker 话题联调：在一个终端启动节点容器，在另一个终端监听一次速度消息，再发布六维有效特征；监听到的 `Twist` 即为控制输出。

```bash
docker compose up -d ros_adapter
docker compose exec -T ros_adapter bash -lc 'source /opt/ros/humble/setup.bash && ros2 topic echo --once /cmd_vel geometry_msgs/msg/Twist'
docker compose exec -T ros_adapter bash -lc 'source /opt/ros/humble/setup.bash && ros2 topic pub --once /landing/features std_msgs/msg/Float32MultiArray "{data: [0.0, 0.0, 0.5, 0.0, 0.0, 1.0]}"'
docker compose stop ros_adapter
```

## 训练与评估

训练样本由固定随机种子生成，并按完整 episode 分组后划分训练集与测试集，比例为 80%/20%。MLP 使用标准化输入和两层隐藏层 `(24, 16)`，预测平面速度和下降速度。网络计算为 `h1 = tanh(W1 x + b1)`、`h2 = tanh(W2 h1 + b2)`、`y = W3 h2 + b3`；测试 RMSE 定义为 `sqrt(sum((y - y_hat)^2) / (3N))`。MuJoCo 演示在每个仿真步采集实际 RGB 图像并输出轨迹和落点指标。

当前记录的一次 180 步运行持续 3.6 秒，平台相对落点中心误差为 0.3387 m，最终机体中心高度为 0.2348 m。着陆成功判定要求机体投影完整处于 0.50 m 平台有效区域内，且中心高度距离接触高度不超过 0.03 m；该次判定为成功。固定随机种子的 1,200 个控制样本按完整轨迹分组，其中 960 条用于训练、240 条用于测试，测试 RMSE 为 0.036406 m/s。导航演示完成 38 个网格步到达目标；地图覆盖率按已知栅格数占总栅格数计算，为 99.22%。
