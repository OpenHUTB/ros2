# wall_clean_nn

MuJoCo 室内清扫车沿墙清扫：激光雷达 + 神经网络贴墙跟随，ROS Noetic 封装。

## 环境

- Ubuntu 20.04
- ROS Noetic（`ros-noetic-ros-base`）
- Python 3.8
- MuJoCo 3.2.3、NumPy、Matplotlib

## 安装

```bash
# 系统依赖（rosdep 可解析部分）
sudo apt install python3-numpy python3-matplotlib python3-pil python3-yaml python3-rospkg

# Python 依赖（MuJoCo 不在 rosdep 中，必须 pip 安装）
pip3 install -r requirements.txt
```

依赖清单见 `requirements.txt`，许可证见 `LICENSE`（MIT）。

## 构建（catkin 工作空间）

```bash
mkdir -p ~/wall_ws/src && cp -r wall_clean_nn ~/wall_ws/src/
cd ~/wall_ws && source /opt/ros/noetic/setup.bash && catkin_make
source devel/setup.bash
```

节点安装到 `devel/lib/wall_clean_nn/`，运行期脚本安装到
`devel/share/wall_clean_nn/scripts/`，`launch`、`config`、`models` 安装到
`devel/share/wall_clean_nn/`；**节点与 `train.py` / `evaluate.py` / `main.py` 都通过
`rospkg` 定位包路径**，因此在源码空间与 install 空间下都能 `rosrun` / `roslaunch`。

## 训练与评测

仓库已附带训练好的权重，可直接跳过训练。

```bash
cd ~/wall_ws/src/wall_clean_nn/scripts
python3 train.py --episodes 60 --steps 1200 --epochs 60 --noise 0.12   # 约 4 分钟
python3 evaluate.py --seeds 20      # 默认 seeds 100–119
python3 make_figures.py             # 缺少输入时会跳过并提示，不再直接报错
```

**种子划分**：训练起点为 `0, 1000, 2000, ...`（`TRAIN_SEED_BASE + TRAIN_SEED_STRIDE × ep`），
评测为 `100–119`（`EVAL_SEED_START`），两者不相交，测试起点不会出现在训练数据里。

**成功判据**：无碰撞且沿墙覆盖率 ≥ `COV_TARGET`（默认 0.95）。超时结束会单独统计为
`timeout`，不计入成功——早期版本把"未碰撞"记为成功，会把超时回合算成成功。

## 测试

```bash
cd ~/wall_ws/src/wall_clean_nn
python3 -m unittest discover -s test -v
```

7 项，覆盖：内壁常量与 `wall_error` 口径（防 0.30 m 系统偏差回归）、沿墙栅格不越界、
超时不得计为成功、达标判据、专家闭环覆盖率与误差、随包权重加载。未装 MuJoCo 时整组跳过。

## 可视化

```bash
cd ~/wall_ws/src/wall_clean_nn/scripts
python3 main.py --mode expert --save-gif wall.gif --every 40 --dpi 70
python3 main.py --mode nn
```

## ROS

```bash
roslaunch wall_clean_nn main.launch mode:=nn
```

`launch/main.launch` 先用 `<rosparam command="load">` 载入 `config/params.yaml`
（`mode` / `rate` / `seed`，可选 `model_dir`），再用 `<param>` 覆盖，因此命令行
`mode:=expert rate:=30` 仍然生效。

> **说明**：该节点内置 MuJoCo 仿真（自己产生 `/cleaner/scan` 与 `/cleaner/odom`），
> 不是接入真机或外部仿真的驱动节点；`manual` 模式会改为订阅外部 `/cleaner/cmd_vel`。

| 话题 | 类型 | 方向 |
|---|---|---|
| `/cleaner/scan` | `sensor_msgs/LaserScan` | 发布 |
| `/cleaner/odom` | `nav_msgs/Odometry` | 发布 |
| `/cleaner/coverage` | `std_msgs/Float32` | 发布（沿墙覆盖率） |
| `/cleaner/state` | `std_msgs/String` | 发布（approach/following/done） |
| `/cleaner/cmd_vel` | `geometry_msgs/Twist` | 订阅（manual 模式） |

## 已知限制

- 三自由度平面模型，未建模轮子打滑与悬挂；
- 激光雷达为理想射线模型，无噪声；
- 家具为静止矩形，未考虑动态障碍；
- 神经网络只在训练分布（房间内随机起点）内验证，未测试未建模的房间形状。

## 与仓库已有模块的区别

| 模块 | 任务 | 与本文的差异 |
|---|---|---|
| `src/ground/mujoco_car_nn_avoidance` | 避障行驶 | 目标是"不撞"，本文是"贴墙覆盖" |
| `src/ground/passive_walker` | 被动行走 | 无路径跟踪与清扫指标 |
| `src/ground/carla_platoon` | 多车编队 | CARLA 场景、无沿墙语义 |
| 本文 | 贴边作业 | 以覆盖率与贴墙误差为指标，专家/神经网络同一闭环对比 |