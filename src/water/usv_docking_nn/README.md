# usv_docking_nn

MuJoCo 无人艇自主靠泊：激光雷达 + 神经网络末段对准，ROS Noetic 封装。

## 环境

- Ubuntu 20.04
- ROS Noetic（`ros-noetic-ros-base`）
- Python 3.8
- MuJoCo 3.2.3、NumPy、Matplotlib

## 安装

```bash
# 系统依赖（rosdep 可解析部分）
sudo apt install python3-numpy python3-matplotlib python3-pil python3-yaml python3-rospkg

# Python 依赖（含 MuJoCo；它不在 rosdep 中，必须用 pip 安装）
pip3 install -r requirements.txt
```

依赖清单见 `requirements.txt`（mujoco>=3.1、numpy、matplotlib、pyyaml、rospkg），
许可证见 `LICENSE`（MIT）。

## 构建（catkin 工作空间）

```bash
mkdir -p ~/usv_ws/src && cp -r usv_docking_nn ~/usv_ws/src/
cd ~/usv_ws && source /opt/ros/noetic/setup.bash && catkin_make
source devel/setup.bash
```

`catkin_make` 会把节点安装到 `devel/lib/usv_docking_nn/`，把运行期脚本安装到
`devel/share/usv_docking_nn/scripts/`，把 `launch`、`config`、`models` 安装到
`devel/share/usv_docking_nn/`。节点通过 `rospkg` 定位包路径后导入这些脚本。

## 训练与评测

仓库已附带训练好的权重，可直接跳过训练。

```bash
cd ~/usv_ws/src/usv_docking_nn/scripts
python3 train.py --episodes 60 --epochs 40       # 约 3 分钟
python3 evaluate.py --seeds 20
```

产物：`models/dock_planner.npz`、`models/dock_norm.npz`、`results/dock_eval.json`。

**种子划分**：训练使用 `0, 1000, 2000, ...`（`TRAIN_SEED_BASE + TRAIN_SEED_STRIDE * ep`），
评测使用 `100–119`（`EVAL_SEED_START = 100`），两者不相交，测试起点不会出现在训练数据里。

## 测试

```bash
cd ~/usv_ws/src/usv_docking_nn
python3 -m unittest discover -s test -v
```

覆盖：专家在 3 个种子上完成靠泊并满足横向/艏向/速度容差、无碰撞、
同一种子完全可复现、随包权重可加载且前向输出有界、规划路径起点与仿真起点一致。
未安装 MuJoCo 时整组自动跳过（`skipUnless`）。

## 可视化

```bash
cd ~/usv_ws/src/usv_docking_nn/scripts
python3 main.py --mode nn --save-gif dock.gif --every 6
python3 main.py --mode expert
```

## ROS

```bash
cd ~/usv_ws && catkin_make && source devel/setup.bash
roslaunch usv_docking_nn main.launch mode:=nn
```

`launch/main.launch` 会先用 `<rosparam command="load">` 载入 `config/params.yaml`
（`mode` / `rate` / `seed`，可选 `model_dir`），随后用 `<param>` 覆盖为 launch 参数，
因此命令行 `mode:=expert rate:=30` 依然生效。改默认值改 `params.yaml` 即可。

`scripts/train.py`、`scripts/evaluate.py`、`scripts/main.py` 与节点一样通过 `rospkg`
定位 `share/<包>/scripts`，在 devel 与 install 空间下都能直接 `rosrun`。

| 话题 | 类型 | 方向 |
|---|---|---|
| `/usv/scan` | `sensor_msgs/LaserScan` | 发布 |
| `/usv/odom` | `nav_msgs/Odometry` | 发布 |
| `/usv/berth` | `std_msgs/Float32MultiArray` | 发布（横向/艏向/纵向偏差） |
| `/usv/state` | `std_msgs/String` | 发布（approach/align/docked） |
| `/usv/cmd_vel` | `geometry_msgs/Twist` | 订阅（manual 模式） |

## 已知限制

- 三自由度平面模型，未建模横摇、纵摇与侧推器；
- 激光雷达为理想射线模型，无噪声与雨雾衰减；
- 泊位为静止刚性结构，未考虑风、流等环境扰动；
- 神经网络为纯 NumPy 的 MLP，未使用 GPU 加速。
